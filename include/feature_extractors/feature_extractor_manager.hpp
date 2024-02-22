/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef FEATURE_EXTRACTORS_INCLUDE_FEATURE_EXTRACTORS_FEATURE_EXTRACTOR_MANAGER_HPP_
#define FEATURE_EXTRACTORS_INCLUDE_FEATURE_EXTRACTORS_FEATURE_EXTRACTOR_MANAGER_HPP_

#include <ros/ros.h>
#include <memory>
#include <utility>

#include "common/types/type.h"
#include "feature_extractors/base_feature_extractor.hpp"
#include "feature_extractors/eigenvalue_feature_extractor.hpp"
#include "feature_extractors/esf_feature_extractor.hpp"
#include "feature_extractors/histogram_feature_extractor.hpp"

namespace autosense {
namespace feature {

static std::unique_ptr<BaseFeatureExtractor> createFeatureExtractor(
    const FeatureExtractorParams& params) {
    std::unique_ptr<BaseFeatureExtractor> feature_extractor;
    if (params.extractor_type == "ESF") {
        feature_extractor =
            std::unique_ptr<BaseFeatureExtractor>(new ESFFeatureExtractor);
        ROS_INFO("[feature] Feature Extractor using Ensemble of Shapes.");
    } else if (params.extractor_type == "EigenValueBased") {
        feature_extractor = std::unique_ptr<BaseFeatureExtractor>(
            new EigenValueFeatureExtractor);
        ROS_INFO(
            "[feature] Feature Extractor using Eigenvalue-based Features.");
    } else if (params.extractor_type == "Histogram") {
        std::unique_ptr<HistogramFeatureExtractor> inner_feature_extractor =
            std::unique_ptr<HistogramFeatureExtractor>(
                new HistogramFeatureExtractor);
        inner_feature_extractor->setHistogramBinSize(
            params.bin_size_for_histogram);
        feature_extractor = std::move(inner_feature_extractor);
        ROS_INFO("[feature] Feature Extractor using Histogram Features.");
    } else {
        feature_extractor = std::unique_ptr<BaseFeatureExtractor>(
            new EigenValueFeatureExtractor);
        ROS_INFO(
            "[feature] Feature Extractor using Eigenvalue-based Features.");
    }
    return feature_extractor;
}

}  // namespace feature
}  // namespace autosense

#endif  // FEATURE_EXTRACTORS_INCLUDE_FEATURE_EXTRACTORS_FEATURE_EXTRACTOR_MANAGER_HPP_
