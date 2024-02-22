/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef FEATURE_EXTRACTORS_INCLUDE_FEATURE_EXTRACTORS_ESF_FEATURE_EXTRACTOR_HPP_
#define FEATURE_EXTRACTORS_INCLUDE_FEATURE_EXTRACTORS_ESF_FEATURE_EXTRACTOR_HPP_

#include <pcl/features/esf.h>
#include <string>
#include <vector>

#include "feature_extractors/base_feature_extractor.hpp"

namespace autosense {
namespace feature {

class ESFFeatureExtractor : public BaseFeatureExtractor {
 public:
    ESFFeatureExtractor();

    ~ESFFeatureExtractor();

    void compute(const std::vector<PointICloudPtr> &cloud_clusters,
                 std::vector<Feature> &features);  // NOLINT

    void compute(const PointICloud &cloud_in, Feature *feature);

    std::string name() const { return "ESFFeatureExtractor"; }

 private:
    pcl::ESFEstimation<Point, pcl::ESFSignature640> esf_estimator_;

    static constexpr unsigned int kESFDimension = 640u;
};

}  // namespace feature
}  // namespace autosense

#endif  // FEATURE_EXTRACTORS_INCLUDE_FEATURE_EXTRACTORS_ESF_FEATURE_EXTRACTOR_HPP_
