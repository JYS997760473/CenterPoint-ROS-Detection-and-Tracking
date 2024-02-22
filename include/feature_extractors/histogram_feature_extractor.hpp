/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */

#ifndef FEATURE_EXTRACTORS_INCLUDE_FEATURE_EXTRACTORS_HISTOGRAM_FEATURE_EXTRACTOR_HPP_
#define FEATURE_EXTRACTORS_INCLUDE_FEATURE_EXTRACTORS_HISTOGRAM_FEATURE_EXTRACTOR_HPP_

#include <vector>
#include <string>

#include "common/types/feature.hpp"
#include "common/types/type.h"
#include "feature_extractors/base_feature_extractor.hpp"

namespace autosense {
namespace feature {

class HistogramFeatureExtractor : public BaseFeatureExtractor {
 public:
    // @brief intialize feature descriptor
    // @params[IN] cloud: given cloud for feature extraction
    // @return nothing
    HistogramFeatureExtractor() {}

    ~HistogramFeatureExtractor() {}

    void compute(const std::vector<PointICloudPtr> &cloud_clusters,
                 std::vector<Feature> &features);  // NOLINT

    // @brief compute histogram feature of given cloud
    // @params[OUT] feature: histogram feature of given cloud
    // @return nothing
    void compute(const PointICloud &cloud_in, Feature *feature);

    std::string name() const { return "HistogramFeatureExtractor"; }

    // @brief set histogram bin size
    // @params[IN] histogram_bin_size: histogram bin size
    // @return true if set successfully, otherwise return fasle
    void setHistogramBinSize(float histogram_bin_size);

 private:
    void getMinMaxCenter(const PointICloud &cloud_in);

    void computeHistogram(int bin_size, std::vector<float> *feature);

 private:
    float bin_size_;
    Point min_pt_;
    Point max_pt_;
    Point center_pt_;
};  // HistogramFeatureExtractor

}  // namespace feature
}  // namespace autosense

#endif  // FEATURE_EXTRACTORS_INCLUDE_FEATURE_EXTRACTORS_HISTOGRAM_FEATURE_EXTRACTOR_HPP_
