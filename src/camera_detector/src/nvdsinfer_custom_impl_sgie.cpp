#include "nvdsinfer_custom_impl.h"
#include <cmath>
#include <algorithm>
#include <glib.h>

static const char* CLASS_LABELS[] = {
    "compact_cars", "large_cars", "midsize-large_station_wagons",
    "midsize_cars", "midsize_station_wagons", "minicompact_cars",
    "minivan_-_2wd", "small_pickup_trucks", "small_pickup_trucks_2wd",
    "small_sport_utility_vehicle_2wd", "small_sport_utility_vehicle_4wd",
    "small_station_wagons", "special_purpose_vehicle_2wd",
    "special_purpose_vehicle_4wd", "special_purpose_vehicles",
    "sport_utility_vehicle_-_2wd", "sport_utility_vehicle_-_4wd",
    "standard_pickup_trucks", "standard_pickup_trucks_2wd",
    "standard_pickup_trucks_4wd", "standard_sport_utility_vehicle_2wd",
    "standard_sport_utility_vehicle_4wd", "subcompact_cars",
    "two_seaters", "vans", "vans,_cargo_type", "vans,_passenger_type"
};

extern "C" bool NvDsInferClassiferParseCustomSoftmax(
    std::vector<NvDsInferLayerInfo> const &outputLayersInfo,
    NvDsInferNetworkInfo const &networkInfo,
    float classifierThreshold,
    std::vector<NvDsInferAttribute> &attrList,
    std::string &descString)
{
    if (outputLayersInfo.empty()) {
        fprintf(stderr, "SGIE: No output layers!\n");
        return false;
    }
    
    float *logits = (float *)outputLayersInfo[0].buffer; // 0 because only one output
    int num_classes = 27;
    
    // Print raw logits
//    fprintf(stderr, "SGIE raw logits: ");
//    for (int i = 0; i < num_classes; i++) {
//        fprintf(stderr, "%.2f ", logits[i]);
//    }
//    fprintf(stderr, "\n");
    
    // Manual softmax
    float max_logit = *std::max_element(logits, logits + num_classes);
    float sum = 0.0f;
    std::vector<float> probs(num_classes);
    for (int i = 0; i < num_classes; i++) {
        probs[i] = expf(logits[i] - max_logit);
        sum += probs[i];
    }
    for (int i = 0; i < num_classes; i++) probs[i] /= sum;
    
    int best = std::max_element(probs.begin(), probs.end()) - probs.begin();
    fprintf(stderr, "SGIE best class=%d prob=%.4f\n", best, probs[best]);
    
    if (probs[best] >= classifierThreshold) {
        NvDsInferAttribute attr;
        attr.attributeIndex = 0;
        attr.attributeValue = best;
        attr.attributeConfidence = probs[best];
	attr.attributeLabel = strdup(CLASS_LABELS[best]);
        attrList.push_back(attr);

	if (attr.attributeLabel) {
		descString.append(attr.attributeLabel).append(" ");
	    }
    }
    
    return true;
}
