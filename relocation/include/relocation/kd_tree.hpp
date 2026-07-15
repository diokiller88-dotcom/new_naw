#pragma once
#include <vector>
#include <memory>
#include <cmath>
#include <limits>
#include <functional>
#include <Eigen/Dense>

namespace relocation {

    class KDNode {
    public:
        int TreeNodeIdx = -1;
        int CloudIdx = -1;
        int Axis = -1;
        float AxisTh = 0.0f; 
        std::unique_ptr<KDNode> Left;
        std::unique_ptr<KDNode> Right;
    };

    class Distance4Node {
    public:
        const KDNode* Node;
        double SquaredDistance;
        Distance4Node(const KDNode* node_, const double& squared_dist_)
            : Node(node_), SquaredDistance(squared_dist_) {}
        bool operator<(const Distance4Node& other_) const {
            return SquaredDistance < other_.SquaredDistance;
        }
    };

    class KDTree {
    public:
        KDTree() = default;
        ~KDTree() { Clear(); }

        bool Build(const std::vector<Eigen::Vector3f>& cloud_);
        void SearchKNearest(const Eigen::Vector3f& point_, int k, std::vector<int>& result_idx_, std::vector<double>& result_dist_);
        int GetNearestIdx(const Eigen::Vector3f& point_);
        int GetBestIdxWithMetric(const Eigen::Vector3f& point_, int candidate_k,
                                 const std::function<double(int)>& metric_func,
                                 double max_euclidean_dist,
                                 double& best_metric,
                                 double& best_euclidean_dist);

        bool Build(const std::vector<Eigen::Vector3i>& cloud_);
        void SearchKNearest(const Eigen::Vector3i& point_, int k, std::vector<int>& result_idx_, std::vector<double>& result_dist_);
        int GetNearestIdx(const Eigen::Vector3i& point_);

        void Clear();

    protected:
        void Reset();
        std::unique_ptr<KDNode> Insert(const std::vector<int>& idx_);
        void SearchKNearestHeapRecursive(const Eigen::Vector3f& point_,
                                         const KDNode* node_, int k,
                                         std::vector<Distance4Node>& result_);
        
        inline double CalculateSquaredDistance(const Eigen::Vector3f& p1,
                                               const Eigen::Vector3f& p2) const {
            return (p1.cast<double>() - p2.cast<double>()).squaredNorm();
        }

    private:
        std::vector<Eigen::Vector3f> m_Cloud;
        std::unique_ptr<KDNode> m_Root;
        int m_NodeNum = 0;
    };

}
