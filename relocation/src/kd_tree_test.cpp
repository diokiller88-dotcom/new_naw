#include "relocation/kd_tree.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

void Require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void TestExactKNearest() {
    std::vector<Eigen::Vector3f> cloud;
    for (int x = -8; x <= 8; ++x) {
        for (int y = -5; y <= 5; ++y) {
            for (int z = -1; z <= 1; ++z) {
                cloud.emplace_back(
                    0.37f * static_cast<float>(x),
                    0.53f * static_cast<float>(y),
                    0.71f * static_cast<float>(z));
            }
        }
    }

    relocation::KDTree tree;
    Require(tree.Build(cloud), "KD-tree build failed");

    constexpr int k = 8;
    for (int query_index = 0; query_index < 100; ++query_index) {
        const Eigen::Vector3f query(
            -2.9f + 0.061f * static_cast<float>(query_index),
            2.3f * std::sin(0.17f * static_cast<float>(query_index)) + 0.013f,
            0.8f * std::cos(0.11f * static_cast<float>(query_index)) + 0.019f);

        std::vector<int> indices;
        std::vector<double> distances;
        tree.SearchKNearest(query, k, indices, distances);
        Require(indices.size() == k && distances.size() == k,
                "KD-tree returned an incorrect neighbor count");

        std::vector<std::pair<double, int>> brute_force;
        brute_force.reserve(cloud.size());
        for (std::size_t i = 0; i < cloud.size(); ++i) {
            brute_force.emplace_back(
                (cloud[i].cast<double>() - query.cast<double>()).norm(),
                static_cast<int>(i));
        }
        std::sort(brute_force.begin(), brute_force.end());

        for (int i = 0; i < k; ++i) {
            Require(std::abs(distances[i] - brute_force[i].first) < 1e-6,
                    "KD-tree distance differs from brute-force search");
            Require(indices[i] == brute_force[i].second,
                    "KD-tree index differs from brute-force search");
        }

        double best_metric = 0.0;
        double best_euclidean_distance = 0.0;
        const int metric_index = tree.GetBestIdxWithMetric(
            query, k,
            [](int index) { return std::abs(index - 173); },
            100.0, best_metric, best_euclidean_distance);
        const auto expected = *std::min_element(
            brute_force.begin(), brute_force.begin() + k,
            [](const auto& lhs, const auto& rhs) {
                const int lhs_metric = std::abs(lhs.second - 173);
                const int rhs_metric = std::abs(rhs.second - 173);
                return lhs_metric != rhs_metric ? lhs_metric < rhs_metric
                                                : lhs.first < rhs.first;
            });
        Require(metric_index == expected.second,
                "KD-tree metric selection returned the wrong index");
        Require(std::abs(best_euclidean_distance - expected.first) < 1e-6,
                "KD-tree metric selection returned the wrong distance");
    }
}

}  // namespace

int main() {
    try {
        TestExactKNearest();
        std::cout << "KD-tree exact-search tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "KD-tree test failed: " << error.what() << '\n';
        return 1;
    }
}
