#include "nvdsinfer_custom_impl.h"
#include <cmath>

static float sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));
}


extern "C" bool NvDsInferParseCustomRTDETR(
    std::vector<NvDsInferLayerInfo> const &outputLayersInfo,
    NvDsInferNetworkInfo const &networkInfo,
    NvDsInferParseDetectionParams const &detectionParams,
    std::vector<NvDsInferObjectDetectionInfo> &objectList)
{
    if (outputLayersInfo.size() < 2) return false;
    
    float* logits = (float*)outputLayersInfo[0].buffer;  // (300, 5)
    float* boxes = (float*)outputLayersInfo[1].buffer;   // (300, 4)
    
    int num_detections = 300;
//    float conf_threshold = 0.5f;  // Match your benchmark
    float conf_threshold = detectionParams.perClassPreclusterThreshold[0];
//    fprintf(stderr, "cofnidence thresholod: %.3f\n", conf_threshold);
    int count = 0;
    
    for (int i = 0; i < num_detections; i++) {
        float* logit = logits + i * 5;
        float* box = boxes + i * 4;
        
	// Print first detection's logits
	if (i == 0) {
//	fprintf(stderr, "Detection 0 raw logits: [%.3f, %.3f, %.3f, %.3f, %.3f]\n",
//		logit[0], logit[1], logit[2], logit[3], logit[4]);
	}
        // Find max logit and its index (this is the confidence)
        float max_logit = logit[0];
        int max_idx = 0;
        for (int c = 1; c < 5; c++) {
            if (logit[c] > max_logit) {
                max_logit = logit[c];
                max_idx = c;
            }
        }
        
        // Apply sigmoid to get confidence
        float confidence = sigmoid(max_logit);
        if (confidence < conf_threshold) continue;
        
        // max_idx is the class_id (0-4, but you only have 0-3)
        int class_id = max_idx;
        
        float cx_n = box[0];
        float cy_n = box[1];
        float w_n = box[2];
        float h_n = box[3];
        
        float left = (cx_n - w_n / 2.0f) * networkInfo.width;
        float top = (cy_n - h_n / 2.0f) * networkInfo.height;
        float width = w_n * networkInfo.width;
        float height = h_n * networkInfo.height;

        //fprintf(stderr, "Detection %d: class=%d conf=%.3f\n", count, class_id, confidence);
        count++;
        
        NvDsInferObjectDetectionInfo obj;
        obj.classId = class_id;
        obj.detectionConfidence = confidence;
        obj.left = left;
        obj.top = top;
        obj.width = width;
        obj.height = height;

	if (count < 3) {
		fprintf(stderr, "NET W,H = %d %d\n", networkInfo.width, networkInfo.height);
	}
        
        objectList.push_back(obj);
    }
    
    fprintf(stderr, "Total: %d\n", count);
    return true;
}
