#pragma once
#include <vector>
#include <queue>
#include <memory>
#include <cmath>
#include <limits>
#include <Eigen/Dense>

namespace relocation {

    class KDNode {
    public:
        int TreeNodeIdx = -1;
        int CloudIdx = -1;
        int Axis = -1;
        float AxisTh = 0.0f; 
        KDNode* Left = nullptr;
        KDNode* Right = nullptr;
    };

    class Distance4Node {
    public:
        const KDNode* Node;
        double Distance;
        Distance4Node(const KDNode* node_, const double& dist_) : Node(node_), Distance(dist_) {}
        bool operator<(const Distance4Node& other_) const {
            return Distance < other_.Distance; 
        }
    };

    class KDTree {
    public:
        KDTree() = default;
        ~KDTree() { Clear(); }

        bool Build(const std::vector<Eigen::Vector3f>& cloud_);
        void SearchKNearest(const Eigen::Vector3f& point_, int k, std::vector<int>& result_idx_, std::vector<double>& result_dist_);
        int GetNearestIdx(const Eigen::Vector3f& point_);

        bool Build(const std::vector<Eigen::Vector3i>& cloud_);
        void SearchKNearest(const Eigen::Vector3i& point_, int k, std::vector<int>& result_idx_, std::vector<double>& result_dist_);
        int GetNearestIdx(const Eigen::Vector3i& point_);

        void Clear();

    protected:
        void Reset();
        KDNode* Insert(const std::vector<int>& idx_, KDNode* node_);
        void SearchKNearestRecursive(const Eigen::Vector3f& point_, const KDNode* node_, int k, std::priority_queue<Distance4Node>& result_);
        
        inline double CalculateDistance(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2) const {
            return (p1.cast<double>() - p2.cast<double>()).norm();
        }

    private:
        std::vector<Eigen::Vector3f> m_Cloud;
        std::shared_ptr<KDNode> m_Root = nullptr;
        int m_NodeNum = 0;
    };

}