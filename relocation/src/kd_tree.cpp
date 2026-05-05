#include "relocation/kd_tree.hpp"
#include <numeric>
#include <iostream>

namespace relocation {

    bool KDTree::Build(const std::vector<Eigen::Vector3i>& cloud_) {
        std::vector<Eigen::Vector3f> float_cloud(cloud_.size());
        for (size_t i = 0; i < cloud_.size(); ++i) {
            float_cloud[i] = cloud_[i].cast<float>();
        }
        return Build(float_cloud);
    }

    void KDTree::SearchKNearest(const Eigen::Vector3i& point_, int k, std::vector<int>& result_idx_, std::vector<double>& result_dist_) {
        Eigen::Vector3f pt = point_.cast<float>();
        SearchKNearest(pt, k, result_idx_, result_dist_);
    }

    int KDTree::GetNearestIdx(const Eigen::Vector3i& point_) {
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
        m_Root.reset(Insert(temp_index, nullptr));
        return true;
    }

    void KDTree::Clear() {
        m_Cloud.clear();
        m_Root = nullptr;
        m_NodeNum = 0;
    }

    void KDTree::Reset() {
        m_NodeNum = 0;
    }

    KDNode* KDTree::Insert(const std::vector<int>& idx_, KDNode* node_) {
        if (idx_.empty()) return nullptr;
        node_ = new KDNode();
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

        node_->Left = Insert(left_idx, node_->Left);
        node_->Right = Insert(right_idx, node_->Right);
        return node_;
    }

    void KDTree::SearchKNearestRecursive(const Eigen::Vector3f& point_, const KDNode* node_, int k, std::priority_queue<Distance4Node>& result_) {
        if (!node_) return;

        if (node_->CloudIdx != -1 && !node_->Left && !node_->Right) {
            double dist = CalculateDistance(m_Cloud[node_->CloudIdx], point_);
            if (result_.size() < static_cast<size_t>(k)) {
                result_.push(Distance4Node(node_, dist));
            } else if (dist < result_.top().Distance) {
                result_.pop();
                result_.push(Distance4Node(node_, dist));
            }
            return;
        }

        KDNode* first_branch = (point_[node_->Axis] < node_->AxisTh) ? node_->Left : node_->Right;
        KDNode* second_branch = (point_[node_->Axis] < node_->AxisTh) ? node_->Right : node_->Left;

        SearchKNearestRecursive(point_, first_branch, k, result_);

        double axis_dist = std::abs(static_cast<double>(point_[node_->Axis]) - static_cast<double>(node_->AxisTh));
        if (result_.size() < static_cast<size_t>(k) || axis_dist < result_.top().Distance) {
            SearchKNearestRecursive(point_, second_branch, k, result_);
        }
    }

    void KDTree::SearchKNearest(const Eigen::Vector3f& point_, int k, std::vector<int>& result_idx_, std::vector<double>& result_dist_) {
        std::priority_queue<Distance4Node> pq;
        SearchKNearestRecursive(point_, m_Root.get(), k, pq);
        result_idx_.resize(pq.size());
        result_dist_.resize(pq.size());
        for (int i = (int)pq.size() - 1; i >= 0; --i) {
            result_idx_[i] = pq.top().Node->CloudIdx;
            result_dist_[i] = pq.top().Distance;
            pq.pop();
        }
    }

    int KDTree::GetNearestIdx(const Eigen::Vector3f& point_) {
        std::vector<int> idx;
        std::vector<double> dist;
        SearchKNearest(point_, 1, idx, dist);
        return idx.empty() ? -1 : idx[0];
    }
}