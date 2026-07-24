#include "relocation/kd_tree.hpp"
#include <numeric>
#include <iostream>
#include <algorithm>

namespace relocation {

    bool KDTree::Build(const std::vector<Eigen::Vector3i>& cloud_) {
        std::vector<Eigen::Vector3f> float_cloud(cloud_.size());
        for (size_t i = 0; i < cloud_.size(); ++i) {
            float_cloud[i] = cloud_[i].cast<float>();
        }
        return Build(float_cloud);
    }

    void KDTree::SearchKNearest(const Eigen::Vector3i& point_, int k, std::vector<int>& result_idx_, std::vector<double>& result_dist_) const {
        Eigen::Vector3f pt = point_.cast<float>();
        SearchKNearest(pt, k, result_idx_, result_dist_);
    }

    int KDTree::GetNearestIdx(const Eigen::Vector3i& point_) const {
        Eigen::Vector3f pt = point_.cast<float>();
        return GetNearestIdx(pt);
    }

    bool KDTree::Build(const std::vector<Eigen::Vector3f>& cloud_) {
        if (cloud_.empty()) return false;
        Clear();
        Reset();
        m_Cloud = cloud_;
        std::vector<int> temp_index(m_Cloud.size());
        std::iota(temp_index.begin(), temp_index.end(), 0);
        m_Root = Insert(temp_index);
        return true;
    }

    void KDTree::Clear() {
        m_Cloud.clear();
        m_Root.reset();
        m_NodeNum = 0;
    }

    void KDTree::Reset() {
        m_NodeNum = 0;
    }

    std::unique_ptr<KDNode> KDTree::Insert(const std::vector<int>& idx_) {
        if (idx_.empty()) return nullptr;
        auto node_ = std::make_unique<KDNode>();
        node_->TreeNodeIdx = m_NodeNum++;

        if (idx_.size() == 1) {
            node_->CloudIdx = idx_[0];
            return node_;
        }

        Eigen::Vector3d temp_mean = Eigen::Vector3d::Zero();
        for (int i : idx_) temp_mean += m_Cloud[i].cast<double>();
        temp_mean /= idx_.size();

        Eigen::Vector3d temp_var = Eigen::Vector3d::Zero();
        for (int i : idx_) {
            Eigen::Vector3d diff = m_Cloud[i].cast<double>() - temp_mean;
            temp_var += diff.cwiseProduct(diff);
        }

        int max_axis;
        temp_var.maxCoeff(&max_axis);
        node_->Axis = max_axis;
        node_->AxisTh = static_cast<float>(temp_mean[max_axis]);

        std::vector<int> left_idx, right_idx;
        left_idx.reserve(idx_.size() / 2);
        right_idx.reserve(idx_.size() / 2);

        for (int i : idx_) {
            if (m_Cloud[i][max_axis] < node_->AxisTh) {
                left_idx.push_back(i);
            } else {
                right_idx.push_back(i);
            }
        }

        if (left_idx.empty() || right_idx.empty()) {
            std::vector<int> sorted_idx = idx_;
            std::sort(sorted_idx.begin(), sorted_idx.end(), [&](int lhs, int rhs) {
                return m_Cloud[lhs][max_axis] < m_Cloud[rhs][max_axis];
            });
            const size_t mid = sorted_idx.size() / 2;
            left_idx.assign(sorted_idx.begin(), sorted_idx.begin() + mid);
            right_idx.assign(sorted_idx.begin() + mid, sorted_idx.end());
            node_->AxisTh = m_Cloud[sorted_idx[mid]][max_axis];
        }

        node_->Left = Insert(left_idx);
        node_->Right = Insert(right_idx);
        return node_;
    }

    void KDTree::SearchKNearestHeapRecursive(
        const Eigen::Vector3f& point_, const KDNode* node_, int k,
        std::vector<Distance4Node>& result_) const {
        if (!node_) return;

        if (node_->CloudIdx != -1 && !node_->Left && !node_->Right) {
            const double squared_dist = CalculateSquaredDistance(
                m_Cloud[node_->CloudIdx], point_);
            if (result_.size() < static_cast<std::size_t>(k)) {
                result_.emplace_back(node_, squared_dist);
                std::push_heap(result_.begin(), result_.end());
            } else if (squared_dist < result_.front().SquaredDistance) {
                std::pop_heap(result_.begin(), result_.end());
                result_.back() = Distance4Node(node_, squared_dist);
                std::push_heap(result_.begin(), result_.end());
            }
            return;
        }

        const bool search_left_first = point_[node_->Axis] < node_->AxisTh;
        const KDNode* first_branch = search_left_first
                                         ? node_->Left.get()
                                         : node_->Right.get();
        const KDNode* second_branch = search_left_first
                                          ? node_->Right.get()
                                          : node_->Left.get();

        SearchKNearestHeapRecursive(point_, first_branch, k, result_);

        const double axis_delta =
            static_cast<double>(point_[node_->Axis]) -
            static_cast<double>(node_->AxisTh);
        const double squared_axis_dist = axis_delta * axis_delta;
        if (result_.size() < static_cast<std::size_t>(k) ||
            squared_axis_dist < result_.front().SquaredDistance) {
            SearchKNearestHeapRecursive(point_, second_branch, k, result_);
        }
    }

    void KDTree::SearchKNearest(const Eigen::Vector3f& point_, int k, std::vector<int>& result_idx_, std::vector<double>& result_dist_) const {
        result_idx_.clear();
        result_dist_.clear();
        if (k <= 0 || !m_Root) return;

        thread_local std::vector<Distance4Node> neighbors;
        neighbors.clear();
        if (neighbors.capacity() < static_cast<std::size_t>(k)) {
            neighbors.reserve(k);
        }
        SearchKNearestHeapRecursive(point_, m_Root.get(), k, neighbors);
        std::sort_heap(neighbors.begin(), neighbors.end());

        result_idx_.resize(neighbors.size());
        result_dist_.resize(neighbors.size());
        for (std::size_t i = 0; i < neighbors.size(); ++i) {
            result_idx_[i] = neighbors[i].Node->CloudIdx;
            result_dist_[i] = std::sqrt(neighbors[i].SquaredDistance);
        }
    }

    int KDTree::GetNearestIdx(const Eigen::Vector3f& point_) const {
        std::vector<int> idx;
        std::vector<double> dist;
        SearchKNearest(point_, 1, idx, dist);
        return idx.empty() ? -1 : idx[0];
    }

    int KDTree::GetBestIdxWithMetric(const Eigen::Vector3f& point_, int candidate_k,
                                     const std::function<double(int)>& metric_func,
                                     double max_euclidean_dist,
                                     double& best_metric,
                                     double& best_euclidean_dist) const {
        best_metric = std::numeric_limits<double>::max();
        best_euclidean_dist = std::numeric_limits<double>::max();
        if (candidate_k <= 0 || !metric_func) return -1;

        thread_local std::vector<Distance4Node> neighbors;
        neighbors.clear();
        if (neighbors.capacity() < static_cast<std::size_t>(candidate_k)) {
            neighbors.reserve(candidate_k);
        }
        SearchKNearestHeapRecursive(
            point_, m_Root.get(), candidate_k, neighbors);

        int best_idx = -1;
        double best_squared_euclidean_dist = std::numeric_limits<double>::max();
        const double max_squared_euclidean_dist =
            max_euclidean_dist * max_euclidean_dist;
        for (const auto& neighbor : neighbors) {
            if (neighbor.SquaredDistance > max_squared_euclidean_dist) continue;
            const int candidate_idx = neighbor.Node->CloudIdx;
            const double metric = metric_func(candidate_idx);
            if (!std::isfinite(metric)) continue;

            if (metric < best_metric ||
                (std::abs(metric - best_metric) < 1e-9 &&
                 neighbor.SquaredDistance < best_squared_euclidean_dist)) {
                best_metric = metric;
                best_squared_euclidean_dist = neighbor.SquaredDistance;
                best_idx = candidate_idx;
            }
        }

        if (best_idx >= 0) {
            best_euclidean_dist = std::sqrt(best_squared_euclidean_dist);
        }

        return best_idx;
    }
}
