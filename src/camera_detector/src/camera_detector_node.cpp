// SPDX-License-Identifier: Apache-2.0
// camera_detector_node.cpp
//
// DeepStream PGIE (RT-DETR) + nvtracker + SGIE (vehicle classifier) pipeline
// wrapped as a ROS 2 node.
//
// ── SOURCE MODES ─────────────────────────────────────────────────────────────
//
//  source_mode: file   (default / dev / bag replay)
//
//    filesrc → h264parse → nvv4l2decoder
//                                       └→ [nvstreammux sink_0]
//                                            → pgie → nvtracker → sgie
//                                            → nvvideoconvert → appsink
//
//  source_mode: native  (production — no ROS on the pixel path)
//
//    aravissrc → capsfilter → tee ──→ queue_inf → nvvideoconvert → [nvstreammux sink_0]
//                               │                                    → pgie → nvtracker → sgie
//                               │                                    → nvvideoconvert2 → appsink
//                               └──→ queue_viz → videoscale → capsfilter_viz
//                                              → videorate → videoconvert → jpegenc
//                                              → appsink_viz  (publishes CompressedImage to ROS)
//
// IMPORTANT: In native mode, triton_cam_driver must finish camera configuration
// and RELEASE the ArenaSDK device BEFORE this node starts. See triton_cam_driver
// launch ordering in bringup/launch/full_system.launch.py.
// ─────────────────────────────────────────────────────────────────────────────

#include <glib.h>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include "gstnvdsmeta.h"
#include "nvds_yml_parser.h"

#include "trackdot_msgs/msg/vehicle_detection.hpp"
#include "trackdot_msgs/msg/vehicle_detection_array.hpp"

namespace trackdot
{

class CameraDetectorNode : public rclcpp::Node
{
public:
  explicit CameraDetectorNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("camera_detector", options)
  {
    // ── Existing parameters (unchanged) ──────────────────────────────────────
    camera_id_      = declare_parameter<std::string>("camera_id",    "cam_left");
    frame_id_       = declare_parameter<std::string>("frame_id",     "cam_left_optical");
    config_file_    = declare_parameter<std::string>("config_file",  "");
    source_uri_     = declare_parameter<std::string>("source_uri",   "");
    output_topic_   = declare_parameter<std::string>("output_topic", "/detections");
    viz_img_topic_ = declare_parameter<std::string>("viz_img_topic", "debug_image");
    pgie_unique_id_ = declare_parameter<int>("pgie_unique_id", 1);
    sgie_unique_id_ = declare_parameter<int>("sgie_unique_id", 2);

    // ── New source-mode parameters ────────────────────────────────────────────
    // source_mode: "file"   — filesrc + h264parse + nvv4l2decoder (dev/replay)
    //              "native" — aravissrc direct from camera (production)
    source_mode_   = declare_parameter<std::string>("source_mode",  "file");
    camera_ip_     = declare_parameter<std::string>("camera_ip", "");

    // Native source resolution + rate — must match triton_cam_driver config
    camera_width_  = declare_parameter<int>("camera_width",  1920);
    camera_height_ = declare_parameter<int>("camera_height", 1080);
    camera_fps_    = declare_parameter<int>("camera_fps",    30);

    // Pixel format that triton_cam_driver configured on the camera.
    // Must be a GStreamer video/x-raw format string (e.g. "RGB", "BGR", "GRAY8")
    pixel_format_  = declare_parameter<std::string>("pixel_format", "RGB");

    // ── Validation ────────────────────────────────────────────────────────────
    if (config_file_.empty()) {
      RCLCPP_FATAL(get_logger(), "[%s] config_file parameter is required",
        camera_id_.c_str());
      throw std::runtime_error("missing config_file");
    }

    if (source_mode_ == "native" && camera_ip_.empty()) {
      RCLCPP_FATAL(get_logger(),
        "[%s] camera_ip is required when source_mode is 'native'",
        camera_id_.c_str());
      throw std::runtime_error("missing camera_ip for native mode");
    }

    if (source_mode_ != "file" && source_mode_ != "native") {
      RCLCPP_FATAL(get_logger(), "[%s] source_mode must be 'file' or 'native', got '%s'",
        camera_id_.c_str(), source_mode_.c_str());
      throw std::runtime_error("invalid source_mode");
    }

    // ── ROS Publishers ────────────────────────────────────────────────────────
    auto qos = rclcpp::SensorDataQoS().keep_last(10);
    pub_     = create_publisher<trackdot_msgs::msg::VehicleDetectionArray>(output_topic_, qos);
    viz_img_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(viz_img_topic_, qos);

    RCLCPP_INFO(get_logger(),
      "[%s] source_mode=%s  config=%s  frame_id=%s",
      camera_id_.c_str(), source_mode_.c_str(),
      config_file_.c_str(), frame_id_.c_str());

    // ── GStreamer ─────────────────────────────────────────────────────────────
    if (!build_pipeline()) {
      throw std::runtime_error("failed to build GStreamer pipeline");
    }
    pipeline_thread_ = std::thread([this]() { run_pipeline(); });
  }

  ~CameraDetectorNode() override { shutdown(); }

private:
  // ── Parameters ──────────────────────────────────────────────────────────────
  std::string camera_id_, frame_id_, config_file_, source_uri_;
  std::string output_topic_, viz_img_topic_;
  int         pgie_unique_id_{1}, sgie_unique_id_{2};

  std::string source_mode_;
  std::string camera_ip_;
  int         camera_width_{1920}, camera_height_{1080}, camera_fps_{30};
  std::string pixel_format_;
  int         viz_width_{960}, viz_height_{540}, viz_fps_{10}, viz_quality_{70};

  // ── ROS ─────────────────────────────────────────────────────────────────────
  rclcpp::Publisher<trackdot_msgs::msg::VehicleDetectionArray>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr         viz_img_pub_;

  // ── GStreamer ────────────────────────────────────────────────────────────────
  GstElement *       pipeline_{nullptr};
  GMainLoop *        loop_{nullptr};
  guint              bus_watch_id_{0};
  std::thread        pipeline_thread_;
  std::atomic<bool>  shutting_down_{false};

  // ===========================================================================
  // Pipeline construction
  // ===========================================================================

  bool build_pipeline()
  {
    if (!gst_is_initialized()) {
      int argc = 0;
      gst_init(&argc, nullptr);
    }

    pipeline_ = gst_pipeline_new(("camera-detector-" + camera_id_).c_str());
    if (!pipeline_) {
      RCLCPP_ERROR(get_logger(), "[%s] gst_pipeline_new failed", camera_id_.c_str());
      return false;
    }

    // ── Shared inference chain (identical in both source modes) ───────────────
    GstElement * streammux = gst_element_factory_make("nvstreammux",    "streammux");
    GstElement * pgie      = gst_element_factory_make("nvinfer",        "pgie");
    GstElement * tracker   = gst_element_factory_make("nvtracker",      "tracker");
    GstElement * sgie      = gst_element_factory_make("nvinfer",        "sgie");
    GstElement * vidconv   = gst_element_factory_make("nvvideoconvert", "vidconv");
    GstElement * appsink   = gst_element_factory_make("appsink",        "appsink");

    if (!streammux || !pgie || !tracker || !sgie || !vidconv || !appsink) {
      RCLCPP_ERROR(get_logger(), "[%s] failed to create inference chain elements",
        camera_id_.c_str());
      return false;
    }

    // Parse YAML config for streammux + inference elements
    char * cfg = const_cast<char *>(config_file_.c_str());
    if (nvds_parse_streammux(streammux, cfg, "streammux")      != NVDS_YAML_PARSER_SUCCESS
     || nvds_parse_gie(pgie,    cfg, "primary-gie")            != NVDS_YAML_PARSER_SUCCESS
     || nvds_parse_gie(sgie,    cfg, "secondary-gie1")         != NVDS_YAML_PARSER_SUCCESS
     || nvds_parse_tracker(tracker, cfg, "tracker")            != NVDS_YAML_PARSER_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "[%s] YAML parse failed for %s",
        camera_id_.c_str(), config_file_.c_str());
      return false;
    }

    // In native mode the camera params determine streammux resolution,
    // overriding whatever the YAML says.
    if (source_mode_ == "native") {
      g_object_set(G_OBJECT(streammux),
        "width",  camera_width_,
        "height", camera_height_,
        NULL);
    }

    // appsink: metadata only, no video rendering, no back-pressure
    g_object_set(G_OBJECT(appsink),
      "emit-signals", TRUE,
      "sync",         FALSE,
      "max-buffers",  1,
      "drop",         TRUE,
      NULL);
    g_signal_connect(appsink, "new-sample",
      G_CALLBACK(&CameraDetectorNode::on_new_sample_static), this);

    gst_bin_add_many(GST_BIN(pipeline_),
      streammux, pgie, tracker, sgie, vidconv, appsink, NULL);

    // ── Build source subgraph and obtain the element to link to streammux ─────
    GstElement * src_tail = nullptr;  // last elem before nvstreammux

    if (source_mode_ == "native") {
      src_tail = build_native_source();
    } else {
      src_tail = build_file_source(cfg);
    }

    if (!src_tail) { return false; }

    // Link source tail → nvstreammux request pad
    GstPad * mux_sink = gst_element_request_pad_simple(streammux, "sink_0");
    GstPad * src_src  = gst_element_get_static_pad(src_tail, "src");

    if (!mux_sink || !src_src
     || gst_pad_link(src_src, mux_sink) != GST_PAD_LINK_OK)
    {
      RCLCPP_ERROR(get_logger(), "[%s] source → streammux link failed",
        camera_id_.c_str());
      return false;
    }
    gst_object_unref(mux_sink);
    gst_object_unref(src_src);

    // Link inference chain: streammux → pgie → tracker → sgie → vidconv → appsink
    if (!gst_element_link_many(streammux, pgie, tracker, sgie, vidconv, appsink, NULL)) {
      RCLCPP_ERROR(get_logger(), "[%s] inference chain link failed", camera_id_.c_str());
      return false;
    }

    // ── Bus watch ─────────────────────────────────────────────────────────────
    GstBus * bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
    loop_        = g_main_loop_new(NULL, FALSE);
    bus_watch_id_ = gst_bus_add_watch(bus, &CameraDetectorNode::bus_call_static, this);
    gst_object_unref(bus);

    return true;
  }

  // ---------------------------------------------------------------------------
  // File source subgraph (dev / bag replay)
  //   filesrc → h264parse → nvv4l2decoder
  //   Returns: nvv4l2decoder  (linked to nvstreammux by caller)
  // ---------------------------------------------------------------------------
  GstElement * build_file_source(char * cfg)
  {
    GstElement * source    = gst_element_factory_make("filesrc",       "src");
    GstElement * h264parse = gst_element_factory_make("h264parse",     "h264parse");
    GstElement * decoder   = gst_element_factory_make("nvv4l2decoder", "decoder");

    if (!source || !h264parse || !decoder) {
      RCLCPP_ERROR(get_logger(), "[%s] failed to create file source elements",
        camera_id_.c_str());
      return nullptr;
    }

    if (nvds_parse_file_source(source, cfg, "source") != NVDS_YAML_PARSER_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "[%s] YAML parse failed for [source]", camera_id_.c_str());
      return nullptr;
    }

    // source_uri parameter overrides the YAML path — handy for launch-time bag selection
    if (!source_uri_.empty()) {
      g_object_set(G_OBJECT(source), "location", source_uri_.c_str(), NULL);
    }

    gst_bin_add_many(GST_BIN(pipeline_), source, h264parse, decoder, NULL);

    if (!gst_element_link_many(source, h264parse, decoder, NULL)) {
      RCLCPP_ERROR(get_logger(), "[%s] filesrc → decoder link failed", camera_id_.c_str());
      return nullptr;
    }

    RCLCPP_INFO(get_logger(), "[%s] file source ready (uri=%s)",
      camera_id_.c_str(), source_uri_.empty() ? "(from YAML)" : source_uri_.c_str());

    return decoder;  // caller links decoder → nvstreammux
  }

  // ---------------------------------------------------------------------------
  // Native source subgraph (production — pixels never touch ROS DDS)
  //
  //   aravissrc → capsfilter → tee ─→ [inference branch] → nvvideoconvert → nvstreammux
  //                                └→ [viz branch]       → jpegenc → appsink_viz (→ CompressedImage)
  //
  // Returns: nvvideoconvert  (linked to nvstreammux by caller)
  //
  // NOTE: triton_cam_driver MUST have released the ArenaSDK device before this
  // is called. aravissrc connects via GenICam/GigE and assumes exclusive stream
  // access. Camera parameters (gain, exposure, trigger) were already applied by
  // triton_cam_driver during its configure-and-release startup phase.
  // ---------------------------------------------------------------------------
  GstElement * build_native_source()
  {
    // ── Source + format negotiation ───────────────────────────────────────────
    GstElement * aravissrc  = gst_element_factory_make("aravissrc",      "aravissrc");
    GstElement * caps_src   = gst_element_factory_make("capsfilter",     "caps_src");
    GstElement * tee        = gst_element_factory_make("tee",            "tee");

    // ── Inference branch ──────────────────────────────────────────────────────
    GstElement * q_inf      = gst_element_factory_make("queue",          "q_inf");
    GstElement * nvvidconv  = gst_element_factory_make("nvvideoconvert", "nvvidconv_src");

    // Bayer to RGB
    GstElement * bayer2rgb = gst_element_factory_make("bayer2rgb", "bayer2rgb");

    if (!bayer2rgb) {
	    RCLCPP_ERROR(get_logger(), "[%s] failed to create bayer2rgb element",
	    camera_id_.c_str());
	  return nullptr;
	}

    // ── Viz branch ────────────────────────────────────────────────────────────
    GstElement * q_viz      = gst_element_factory_make("queue",          "q_viz");
    GstElement * videoscale = gst_element_factory_make("videoscale",     "videoscale");
    GstElement * caps_viz   = gst_element_factory_make("capsfilter",     "caps_viz");
    GstElement * videorate  = gst_element_factory_make("videorate",      "videorate");
    GstElement * videoconv  = gst_element_factory_make("videoconvert",   "videoconv_viz");
    GstElement * jpegenc    = gst_element_factory_make("jpegenc",        "jpegenc");
    GstElement * appsink_viz = gst_element_factory_make("appsink",       "appsink_viz");

    // Replace the combined null check block with individual checks:
	if (!aravissrc)   { RCLCPP_ERROR(get_logger(), "[%s] FAILED: aravissrc",    camera_id_.c_str()); return nullptr; }
	if (!caps_src)    { RCLCPP_ERROR(get_logger(), "[%s] FAILED: capsfilter",   camera_id_.c_str()); return nullptr; }
	if (!tee)         { RCLCPP_ERROR(get_logger(), "[%s] FAILED: tee",          camera_id_.c_str()); return nullptr; }
	if (!q_inf)       { RCLCPP_ERROR(get_logger(), "[%s] FAILED: queue(inf)",   camera_id_.c_str()); return nullptr; }
	if (!nvvidconv)   { RCLCPP_ERROR(get_logger(), "[%s] FAILED: nvvideoconv",  camera_id_.c_str()); return nullptr; }
	if (!bayer2rgb)   { RCLCPP_ERROR(get_logger(), "[%s] FAILED: bayer2rgb",    camera_id_.c_str()); return nullptr; }
	if (!q_viz)       { RCLCPP_ERROR(get_logger(), "[%s] FAILED: queue(viz)",   camera_id_.c_str()); return nullptr; }
	if (!videoscale)  { RCLCPP_ERROR(get_logger(), "[%s] FAILED: videoscale",   camera_id_.c_str()); return nullptr; }
	if (!caps_viz)    { RCLCPP_ERROR(get_logger(), "[%s] FAILED: capsfilter2",  camera_id_.c_str()); return nullptr; }
	if (!videorate)   { RCLCPP_ERROR(get_logger(), "[%s] FAILED: videorate",    camera_id_.c_str()); return nullptr; }
	if (!videoconv)   { RCLCPP_ERROR(get_logger(), "[%s] FAILED: videoconvert", camera_id_.c_str()); return nullptr; }
	if (!jpegenc)     { RCLCPP_ERROR(get_logger(), "[%s] FAILED: jpegenc",      camera_id_.c_str()); return nullptr; }
	if (!appsink_viz) { RCLCPP_ERROR(get_logger(), "[%s] FAILED: appsink_viz",  camera_id_.c_str()); return nullptr; }
    if (!aravissrc || !caps_src || !tee
     || !q_inf || !nvvidconv
     || !q_viz || !videoscale || !caps_viz || !videorate
     || !videoconv || !jpegenc || !appsink_viz)
    {
      RCLCPP_ERROR(get_logger(), "[%s] failed to create native source elements",
        camera_id_.c_str());
      return nullptr;
    }

    // ── aravissrc: connect to camera by serial ────────────────────────────────
    // camera-name is matched against the GenICam device model/serial string.
    // Corresponds to the serial_ parameter in triton_cam_driver.
    g_object_set(G_OBJECT(aravissrc), "camera-name", camera_ip_.c_str(), NULL);

    // ── Caps after aravissrc: enforce the format triton_cam_driver configured ─
    GstCaps * src_caps = gst_caps_new_simple("video/x-bayer",
      "width",     G_TYPE_INT,          camera_width_,
      "height",    G_TYPE_INT,          camera_height_,
      NULL);
    g_object_set(G_OBJECT(caps_src), "caps", src_caps, NULL);
    gst_caps_unref(src_caps);

    // ── nvvideoconvert: host memory → NVMM for nvstreammux ───────────────────
    // nvbuf-memory-type 0 = NVBUF_MEM_DEFAULT → NVMM on Jetson Orin
    g_object_set(G_OBJECT(nvvidconv), "nvbuf-memory-type", 0, NULL);

    // ── Viz branch caps: downscale + rate limit ────────────────────────────────
    GstCaps * viz_caps = gst_caps_new_simple("video/x-raw",
      "width",     G_TYPE_INT,          viz_width_,
      "height",    G_TYPE_INT,          viz_height_,
      "framerate", GST_TYPE_FRACTION,   viz_fps_, 1,
      NULL);
    g_object_set(G_OBJECT(caps_viz), "caps", viz_caps, NULL);
    gst_caps_unref(viz_caps);

    // ── jpegenc quality ───────────────────────────────────────────────────────
    g_object_set(G_OBJECT(jpegenc), "quality", viz_quality_, NULL);

    // ── appsink_viz: pull JPEG buffers and publish to ROS ─────────────────────
    g_object_set(G_OBJECT(appsink_viz),
      "emit-signals", TRUE,
      "sync",         FALSE,
      "max-buffers",  1,
      "drop",         TRUE,   // drop old frames; never block the viz branch
      NULL);
    g_signal_connect(appsink_viz, "new-sample",
      G_CALLBACK(&CameraDetectorNode::on_viz_sample_static), this);

    // ── Add all elements to pipeline ──────────────────────────────────────────
    gst_bin_add_many(GST_BIN(pipeline_),
      aravissrc, caps_src, bayer2rgb, tee,
      q_inf, nvvidconv,
      q_viz, videoscale, caps_viz, videorate, videoconv, jpegenc, appsink_viz,
      NULL);

    // ── Link: aravissrc → caps_src → tee ─────────────────────────────────────
    if (!gst_element_link_many(aravissrc, caps_src, bayer2rgb, tee, NULL)) {
      RCLCPP_ERROR(get_logger(), "[%s] aravissrc → tee link failed", camera_id_.c_str());
      return nullptr;
    }

    // ── Tee → inference branch: q_inf → nvvidconv ────────────────────────────
    {
      GstPad * tee_src  = gst_element_request_pad_simple(tee, "src_%u");
      GstPad * q_sink   = gst_element_get_static_pad(q_inf, "sink");
      if (!tee_src || !q_sink || gst_pad_link(tee_src, q_sink) != GST_PAD_LINK_OK) {
        RCLCPP_ERROR(get_logger(), "[%s] tee → q_inf link failed", camera_id_.c_str());
        return nullptr;
      }
      gst_object_unref(tee_src);
      gst_object_unref(q_sink);
    }
    if (!gst_element_link(q_inf, nvvidconv)) {
      RCLCPP_ERROR(get_logger(), "[%s] q_inf → nvvidconv link failed", camera_id_.c_str());
      return nullptr;
    }

    // ── Tee → viz branch: q_viz → videoscale → caps_viz → videorate → videoconv → jpegenc → appsink_viz
    {
      GstPad * tee_src = gst_element_request_pad_simple(tee, "src_%u");
      GstPad * q_sink  = gst_element_get_static_pad(q_viz, "sink");
      if (!tee_src || !q_sink || gst_pad_link(tee_src, q_sink) != GST_PAD_LINK_OK) {
        RCLCPP_ERROR(get_logger(), "[%s] tee → q_viz link failed", camera_id_.c_str());
        return nullptr;
      }
      gst_object_unref(tee_src);
      gst_object_unref(q_sink);
    }
    if (!gst_element_link_many(q_viz, videoscale, caps_viz, videorate,
                               videoconv, jpegenc, appsink_viz, NULL))
    {
      RCLCPP_ERROR(get_logger(), "[%s] viz branch link failed", camera_id_.c_str());
      return nullptr;
    }

    RCLCPP_INFO(get_logger(),
      "[%s] native source ready (serial=%s, %dx%d@%dfps → inf; %dx%d@%dfps JPEG → viz)",
      camera_id_.c_str(), camera_ip_.c_str(),
      camera_width_, camera_height_, camera_fps_,
      viz_width_, viz_height_, viz_fps_);

    // nvvidconv is the last inference-branch element; caller links it to nvstreammux
    return nvvidconv;
  }

  // ===========================================================================
  // Pipeline runtime
  // ===========================================================================

  void run_pipeline()
  {
    RCLCPP_INFO(get_logger(), "[%s] pipeline → PLAYING", camera_id_.c_str());
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    g_main_loop_run(loop_);
    RCLCPP_INFO(get_logger(), "[%s] main loop returned", camera_id_.c_str());
  }

  void shutdown()
  {
    if (shutting_down_.exchange(true)) return;
    RCLCPP_INFO(get_logger(), "[%s] shutting down", camera_id_.c_str());
    if (loop_ && g_main_loop_is_running(loop_)) {
      g_main_loop_quit(loop_);
    }
    if (pipeline_thread_.joinable()) {
      pipeline_thread_.join();
    }
    if (pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(GST_OBJECT(pipeline_));
      pipeline_ = nullptr;
    }
    if (bus_watch_id_) {
      g_source_remove(bus_watch_id_);
      bus_watch_id_ = 0;
    }
    if (loop_) {
      g_main_loop_unref(loop_);
      loop_ = nullptr;
    }
  }

  // ===========================================================================
  // GStreamer callbacks
  // ===========================================================================

  static gboolean bus_call_static(GstBus * bus, GstMessage * msg, gpointer user)
  {
    return static_cast<CameraDetectorNode *>(user)->bus_call(bus, msg);
  }

  gboolean bus_call(GstBus * /*bus*/, GstMessage * msg)
  {
    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_EOS:
        RCLCPP_INFO(get_logger(), "[%s] EOS", camera_id_.c_str());
        g_main_loop_quit(loop_);
        break;
      case GST_MESSAGE_ERROR: {
        gchar *  debug = nullptr;
        GError * err   = nullptr;
        gst_message_parse_error(msg, &err, &debug);
        RCLCPP_ERROR(get_logger(), "[%s] GStreamer error from %s: %s (%s)",
          camera_id_.c_str(),
          GST_OBJECT_NAME(msg->src),
          err   ? err->message : "?",
          debug ? debug        : "");
        if (debug) g_free(debug);
        if (err)   g_error_free(err);
        g_main_loop_quit(loop_);
        break;
      }
      default: break;
    }
    return TRUE;
  }

  // ---------------------------------------------------------------------------
  // Viz appsink callback — fires on the viz branch (native mode only)
  // Pulls a JPEG-encoded buffer and publishes sensor_msgs/CompressedImage.
  // This callback runs on a GStreamer streaming thread, NOT the ROS executor.
  // ---------------------------------------------------------------------------
  static GstFlowReturn on_viz_sample_static(GstAppSink * sink, gpointer user)
  {
    return static_cast<CameraDetectorNode *>(user)->on_viz_sample(sink);
  }

  GstFlowReturn on_viz_sample(GstAppSink * sink)
  {
    if (!viz_img_pub_) return GST_FLOW_OK;

    GstSample * sample = gst_app_sink_pull_sample(sink);
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer * buf = gst_sample_get_buffer(sample);
    if (!buf) { gst_sample_unref(sample); return GST_FLOW_OK; }

    GstMapInfo map;
    if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
      gst_sample_unref(sample);
      return GST_FLOW_OK;
    }

    auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
    msg->header.stamp    = now();
    msg->header.frame_id = frame_id_;
    msg->format          = "jpeg";
    // Single memcpy: JPEG-compressed buffer → ROS message data vector.
    // At 960×540 JPEG quality 70, this is ~30–80 KB per frame at 10 Hz.
    msg->data.assign(map.data, map.data + map.size);

    viz_img_pub_->publish(std::move(msg));

    gst_buffer_unmap(buf, &map);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  // ---------------------------------------------------------------------------
  // Inference appsink callback — unchanged from original
  // Extracts NvDsObjectMeta + NvDsClassifierMeta and publishes detections.
  // ---------------------------------------------------------------------------
  static GstFlowReturn on_new_sample_static(GstAppSink * sink, gpointer user)
  {
    return static_cast<CameraDetectorNode *>(user)->on_new_sample(sink);
  }

  GstFlowReturn on_new_sample(GstAppSink * sink)
  {
    GstSample * sample = gst_app_sink_pull_sample(sink);
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer * buf = gst_sample_get_buffer(sample);
    if (!buf) { gst_sample_unref(sample); return GST_FLOW_OK; }

    NvDsBatchMeta * batch_meta = gst_buffer_get_nvds_batch_meta(buf);
    if (!batch_meta) { gst_sample_unref(sample); return GST_FLOW_OK; }

    auto msg     = std::make_unique<trackdot_msgs::msg::VehicleDetectionArray>();
    auto viz_msg = std::make_unique<vision_msgs::msg::Detection2DArray>();

    // Timestamp: prefer GstBuffer PTS (maps to camera hardware timestamp via PTP)
    /*
    rclcpp::Time stamp;
    if (GST_BUFFER_PTS_IS_VALID(buf)) {
      stamp = rclcpp::Time(static_cast<int64_t>(GST_BUFFER_PTS(buf)), RCL_ROS_TIME);
    } else {
      stamp = now();
    }
    */
    rclcpp::Time stamp = now();
    msg->header.stamp        = stamp;
    msg->header.frame_id     = frame_id_;
    viz_msg->header.stamp    = stamp;
    viz_msg->header.frame_id = frame_id_;

    for (NvDsMetaList * l_frame = batch_meta->frame_meta_list;
         l_frame != nullptr; l_frame = l_frame->next)
    {
      auto * frame_meta = static_cast<NvDsFrameMeta *>(l_frame->data);

      for (NvDsMetaList * l_obj = frame_meta->obj_meta_list;
           l_obj != nullptr; l_obj = l_obj->next)
      {
        auto * obj = static_cast<NvDsObjectMeta *>(l_obj->data);

        trackdot_msgs::msg::VehicleDetection det;
        det.camera_id       = camera_id_;
        det.track_id        = static_cast<int64_t>(obj->object_id);
        det.class_id        = obj->class_id;
        det.pgie_confidence = obj->confidence;
        det.bbox_x          = obj->rect_params.left;
        det.bbox_y          = obj->rect_params.top;
        det.bbox_width      = obj->rect_params.width;
        det.bbox_height     = obj->rect_params.height;
        det.sgie_label      = "";
        det.sgie_confidence = 0.0f;

        for (NvDsMetaList * l_class = obj->classifier_meta_list;
             l_class != nullptr; l_class = l_class->next)
        {
          auto * cmeta = static_cast<NvDsClassifierMeta *>(l_class->data);
          if (cmeta->unique_component_id != sgie_unique_id_) continue;
          if (!cmeta->label_info_list) continue;
          auto * linfo = static_cast<NvDsLabelInfo *>(cmeta->label_info_list->data);
          if (linfo->result_label[0] != '\0') {
            det.sgie_label      = linfo->result_label;
            det.sgie_confidence = linfo->result_prob;
          }
          break;
        }
        msg->detections.push_back(std::move(det));

      }
    }

    pub_->publish(std::move(msg));
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }
};

}  // namespace trackdot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<trackdot::CameraDetectorNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
