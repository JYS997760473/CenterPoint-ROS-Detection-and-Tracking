/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef FEATURE_EXTRACTORS_INCLUDE_FEATURE_EXTRACTORS_EIGENVALUE_FEATURE_EXTRACTOR_HPP_
#define FEATURE_EXTRACTORS_INCLUDE_FEATURE_EXTRACTORS_EIGENVALUE_FEATURE_EXTRACTOR_HPP_

#include <string>
#include <vector>

#include "common/types/type.h"
#include "feature_extractors/base_feature_extractor.hpp"

namespace autosense {
namespace feature {

class EigenValueFeatureExtractor : public BaseFeatureExtractor {
 public:
    EigenValueFeatureExtractor();

    ~EigenValueFeatureExtractor();

    void compute(const std::vector<PointICloudPtr> &cloud_clusters,
                 std::vector<Feature> &features);  // NOLINT

    void compute(const PointICloud &cloud_in, Feature *feature);

    /// \brief Get the descriptor's dimension.
    unsigned int dimension() const { return kDimension; }

    std::string name() const { return "EigenValueFeatureExtractor"; }

 private:
    static constexpr unsigned int kDimension = 9u;
};

}  // namespace feature
}  // namespace autosense

#endif  // FEATURE_EXTRACTORS_INCLUDE_FEATURE_EXTRACTORS_EIGENVALUE_FEATURE_EXTRACTOR_HPP_
