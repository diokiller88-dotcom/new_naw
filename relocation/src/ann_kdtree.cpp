#include "relocation/ann_kdtree.hpp"
#include <numeric>
#include <algorithm>
#include <limits>
#include <cmath>

namespace relocation {

    ann_kdtree::ann_kdtree(int leaf_size, int max_checks) 
        : m_LeafSize(leaf_size), m_MaxChecks(max_checks), m_Dim(0) {}

    bool ann_kdtree::Build(const std::vector<std::vector<uint8_t>>& cloud) {
        if (cloud.empty() || cloud[0].empty()) return false;
        m_Cloud = cloud; 
        m_Dim = cloud[0].size();
        for (const auto& point : m_Cloud) {
            if (static_cast<int>(point.size()) != m_Dim) return false;
        }
        m_Indices.resize(cloud.size()); 
        std::iota(m_Indices.begin(), m_Indices.end(), 0);
        m_Root = BuildRecursive(0, m_Indices.size());
        return true;
    }

    std::unique_ptr<KdTreeNode> ann_kdtree::BuildRecursive(int start, int end) {
        if (start >= end) return nullptr;
        auto node = std::make_unique<KdTreeNode>();
        
        if (end - start <= m_LeafSize) {
            node->is_leaf = true; 
            node->start_idx = start; 
            node->end_idx = end; 
            return node;
        }
        
        int best_axis = 0, min_diff = std::numeric_limits<int>::max();
        for (int d = 0; d < m_Dim; ++d) {
            int ones = 0;
            for (int i = start; i < end; ++i) {
                if (m_Cloud[m_Indices[i]][d] == 1) ones++;
            }
            int diff = std::abs(ones - (end - start) / 2);
            if (diff < min_diff) { 
                min_diff = diff; 
                best_axis = d; 
            }
        }
        
        auto bound = std::partition(m_Indices.begin() + start, m_Indices.begin() + end,
            [&](int a) { return m_Cloud[a][best_axis] == 0; });
        int mid = std::distance(m_Indices.begin(), bound);
        
        if (mid == start || mid == end) {
            mid = start + (end - start) / 2;
        }
        
        node->axis = best_axis;
        node->left = BuildRecursive(start, mid);
        node->right = BuildRecursive(mid, end);
        return node;
    }

    void ann_kdtree::SearchKNearest(const std::vector<uint8_t>& query, int k, std::vector<int>& out_indices, std::vector<int>& out_dists) {
        out_indices.clear();
        out_dists.clear();
        if (static_cast<int>(query.size()) != m_Dim) return;

        std::priority_queue<DistIndex> pq; 
        for (size_t i = 0; i < m_Cloud.size(); ++i) {
            int dist = 0;
            const auto& target = m_Cloud[i];
            for (int d = 0; d < m_Dim; ++d) {
                if (target[d] != query[d]) dist++;
            }
            if (pq.size() < static_cast<size_t>(k)) {
                pq.emplace(static_cast<int>(i), dist);
            } else if (dist < pq.top().dist) {
                pq.pop();
                pq.emplace(static_cast<int>(i), dist);
            }
        }
        
        out_indices.resize(pq.size()); 
        out_dists.resize(pq.size());
        for (int i = static_cast<int>(pq.size()) - 1; i >= 0; --i) {
            out_indices[i] = pq.top().index; 
            out_dists[i] = pq.top().dist; 
            pq.pop();
        }
    }

    void ann_kdtree::SearchRecursive(const std::vector<uint8_t>& query, const KdTreeNode* node, int k, std::priority_queue<DistIndex>& pq, int& checks) {
        if (!node) return;
        if (node->is_leaf) {
            for (int i = node->start_idx; i < node->end_idx; ++i) {
                int dist = 0; 
                const auto& target = m_Cloud[m_Indices[i]];
                for (int d = 0; d < m_Dim; ++d) {
                    if (target[d] != query[d]) dist++;
                }
                if (pq.size() < static_cast<size_t>(k)) {
                    pq.emplace(m_Indices[i], dist);
                } else if (dist < pq.top().dist) { 
                    pq.pop(); 
                    pq.emplace(m_Indices[i], dist); 
                }
            }
            return;
        }
        
        bool left = (query[node->axis] == 0);
        SearchRecursive(query, left ? node->left.get() : node->right.get(), k, pq, checks);
        
        if (checks >= m_MaxChecks) return;
        
        if (pq.size() < static_cast<size_t>(k) || pq.top().dist >= 1) {
            checks++; 
            SearchRecursive(query, left ? node->right.get() : node->left.get(), k, pq, checks);
        }
    }

}
