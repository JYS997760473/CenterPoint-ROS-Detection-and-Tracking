/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef FEATURE_EXTRACTORS_INCLUDE_FEATURE_EXTRACTORS_BASE_FEATURE_EXTRACTOR_HPP_
#define FEATURE_EXTRACTORS_INCLUDE_FEATURE_EXTRACTORS_BASE_FEATURE_EXTRACTOR_HPP_

#include <string>
#include <vector>

#include "common/types/feature.hpp"
#include "common/types/type.h"

namespace autosense {
namespace feature {

class BaseFeatureExtractor {
 public:
    virtual void compute(const std::vector<PointICloudPtr> &cloud_clusters,
                         std::vector<Feature> &features) = 0;  // NOLINT

    virtual void compute(const PointICloud &cloud_in, Feature *feature) = 0;

    virtual std::string name() const = 0;

    void normalizeFeatures(std::vector<Feature> *features) {
        const unsigned int num_features = (*features).size();
        const unsigned int dim_feature = (*features)[0].size();

        std::vector<FeatureElementType> mins(dim_feature);
        std::vector<FeatureElementType> maxs(dim_feature);
        for (size_t dim = 0u; dim < dim_feature; ++dim) {
            mins[dim] = (*features)[0].at(dim).value;
            maxs[dim] = (*features)[0].at(dim).value;
        }
        for (size_t f = 1u; f < num_features; ++f) {
            for (size_t dim = 0u; dim < dim_feature; ++dim) {
                if ((*features)[f].at(dim).value < mins[dim]) {
                    mins[dim] = (*features)[f].at(dim).value;
                }
                if ((*features)[f].at(dim).value > maxs[dim]) {
                    maxs[dim] = (*features)[f].at(dim).value;
                }
            }
        }

        std::vector<double> intervals(dim_feature);
        for (size_t dim = 0u; dim < dim_feature; ++dim) {
            intervals[dim] = maxs[dim] - mins[dim];
        }

        for (size_t f = 0u; f < num_features; ++f) {
            for (size_t dim = 0u; dim < dim_feature; ++dim) {
                FeatureElementType value = (*features)[f].at(dim).value;
                value -= mins[dim];
                value /= intervals[dim];
                (*features)[f].setValueById(dim, value);
            }
        }
    }
};  // class BaseFeatureExtractor

// TODO(gary): 1-NN Classifier distance measure
/*void OpenCvRandomForest::computeFeaturesDistance(const Eigen::MatrixXd& f1,
                                                 const Eigen::MatrixXd& f2,
                                                 Eigen::MatrixXd *f_out) const {
    CHECK_EQ(f1.cols(), f2.cols());
    CHECK_EQ(f1.rows(), f2.rows());
    const unsigned int n_sample = f1.rows();

    std::vector<Eigen::MatrixXd> fs;
    if (params_.descriptor_types.empty()) {
        *f_out = (f1 - f2).cwiseAbs();
    } else {
        unsigned int f_index = 0;
        unsigned int final_dim = 0;

        for (size_t i = 0u; i < params_.descriptor_types.size(); ++i) {
            if (params_.descriptor_types[i] == "EigenvalueBased") {
                const unsigned int f_dim = 7u;
                unsigned int f_dim_out = 7u;
                CHECK_GE(f1.cols(), f_index + f_dim);

                Eigen::MatrixXd v1 = f1.block(0, f_index, n_sample, f_dim);
                Eigen::MatrixXd v2 = f2.block(0, f_index, n_sample, f_dim);
                Eigen::MatrixXd f_diff = (v1 - v2).cwiseAbs();

                fs.push_back(f_diff);

                MatrixXd f1_abs = v1.cwiseAbs();
                MatrixXd f2_abs = v2.cwiseAbs();

                MatrixXd f_diff_norm_2 = f_diff.cwiseQuotient(f2_abs);
                MatrixXd f_diff_norm_1 = f_diff.cwiseQuotient(f1_abs);

                if (!params_.apply_hard_threshold_on_feature_distance) {
                    // Augment the eigen feature vectors.
                    fs.push_back(f_diff_norm_2);
                    fs.push_back(f_diff_norm_1);
                    fs.push_back(f1_abs);
                    fs.push_back(f1_abs);
                    f_dim_out = 35u;
                }

                f_index += f_dim;
                final_dim += f_dim_out;
            } else if (params_.descriptor_types[i] ==
                       "EnsembleShapeFunctions") {
                const unsigned int f_dim = 640u;
                const unsigned int f_dim_out = 10u;
                const unsigned int bin_size = 64u;
                CHECK_GE(f1.cols(), f_index + f_dim);

                Eigen::MatrixXd f(n_sample, f_dim_out);
                Eigen::MatrixXd h1 = f1.block(0, f_index, n_sample, f_dim);
                Eigen::MatrixXd h2 = f2.block(0, f_index, n_sample, f_dim);

                for (size_t i = 0; i < f_dim_out; ++i) {
                    Eigen::MatrixXd intersection;
                    histogramIntersection(h1.block(0, i * bin_size, n_sample,
                                                   bin_size),
                                          h2.block(0, i * bin_size, n_sample,
                                                   bin_size),
                                          &intersection);
                    f.block(0, i, n_sample, 1) = intersection;
                }

                fs.push_back(f);
                f_index += f_dim;
                final_dim += f_dim_out;
            } else {
                CHECK(false) << "Distance not implemented.";
            }
        }

        // Reconstruct the feature vector.
        f_out->resize(n_sample, final_dim);
        f_index = 0;
        for (size_t i = 0u; i < fs.size(); ++i) {
            f_out->block(0, f_index, n_sample, fs[i].cols()) = fs[i];
            f_index += fs[i].cols();
        }
    }
}*/

}  // namespace feature
}  // namespace autosense

#endif  // FEATURE_EXTRACTORS_INCLUDE_FEATURE_EXTRACTORS_BASE_FEATURE_EXTRACTOR_HPP_
