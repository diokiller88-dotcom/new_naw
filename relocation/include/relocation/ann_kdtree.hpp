#pragma once

#include <vector>
#include <memory>
#include <cstdint>
#include <queue>

namespace relocation {

    constexpr int ann_leaf_size = 32;
    constexpr int ann_max_checks = 100;

    struct DistIndex {
        int index;
        int dist;
        DistIndex(int i, int d) : index(i), dist(d) {}
        bool operator<(const DistIndex& other) const { return dist < other.dist; } 
    };

    struct KdTreeNode {
        bool is_leaf = false;
        int axis = -1;
        int start_idx = -1;
        int end_idx = -1; 
        std::unique_ptr<KdTreeNode> left = nullptr;
        std::unique_ptr<KdTreeNode> right = nullptr;
    };

    class ann_kdtree {
    public:
        ann_kdtree(int leaf_size = ann_leaf_size, int max_checks = ann_max_checks);
        bool Build(const std::vector<std::vector<uint8_t>>& cloud);
        void SearchKNearest(const std::vector<uint8_t>& query, int k, std::vector<int>& out_indices, std::vector<int>& out_dists);

    private:
        std::unique_ptr<KdTreeNode> BuildRecursive(int start, int end);
        void SearchRecursive(const std::vector<uint8_t>& query, const KdTreeNode* node, int k, std::priority_queue<DistIndex>& pq, int& checks);
        
        int m_LeafSize;
        int m_MaxChecks;
        int m_Dim;
        std::vector<std::vector<uint8_t>> m_Cloud;
        std::vector<int> m_Indices; 
        std::unique_ptr<KdTreeNode> m_Root;
    };

}