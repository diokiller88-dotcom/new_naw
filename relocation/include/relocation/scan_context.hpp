#pragma once
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstddef>
#include <memory>
#include <unordered_map>
#include <vector>

namespace relocation {

enum class ScanContextType {
    Polar,
    Cartesian
};

enum class ScanContextVariant {
    Original,
    PolarLeftShift,
    PolarRightShift,
    CartesianDoubleFlip
};

struct ScanContextConfig {
    ScanContextType type = ScanContextType::Polar;
    int rows = 10;
    int columns = 360;

    float min_radius = 0.0f;
    float max_radius = 10.0f;

    float min_x = -10.0f;
    float max_x = 10.0f;
    float min_y = -10.0f;
    float max_y = 10.0f;

    float min_z = 0.0f;
    float max_z = 2.0f;
    float height_offset = 0.0f;
    float voxel_leaf_size = 0.0f;

    int candidate_count = 8;
    int alignment_search_radius = 2;
    float distance_threshold = 0.30f;

    bool enable_augmentation = true;
    float polar_lateral_augmentation = 2.0f;

    static ScanContextConfig IrisPolar();
    static ScanContextConfig IrisCartesian();
    static ScanContextConfig PaperPolar();
    static ScanContextConfig PaperCartesian();
};

struct ScanContextDescriptor {
    Eigen::MatrixXf matrix;
    Eigen::VectorXf retrieval_key;
    Eigen::VectorXf alignment_key;

    int place_id = -1;
    ScanContextVariant variant = ScanContextVariant::Original;
    float virtual_lateral_shift = 0.0f;
    float heading_offset_rad = 0.0f;
};

struct ScanContextMatch {
    bool matched = false;
    int place_id = -1;
    int descriptor_index = -1;
    ScanContextVariant variant = ScanContextVariant::Original;

    float distance = 1.0f;
    float retrieval_key_distance = 0.0f;
    int column_shift = 0;
    float relative_yaw_rad = 0.0f;
    float relative_lateral_m = 0.0f;
};

class ScanContextPlusPlus {
public:
    explicit ScanContextPlusPlus(ScanContextConfig config = ScanContextConfig::IrisPolar());
    ~ScanContextPlusPlus();

    ScanContextPlusPlus(const ScanContextPlusPlus&) = delete;
    ScanContextPlusPlus& operator=(const ScanContextPlusPlus&) = delete;
    ScanContextPlusPlus(ScanContextPlusPlus&&) noexcept;
    ScanContextPlusPlus& operator=(ScanContextPlusPlus&&) noexcept;

    const ScanContextConfig& GetConfig() const { return m_Config; }

    ScanContextDescriptor MakeDescriptor(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
        int place_id = -1) const;

    std::vector<ScanContextDescriptor> MakeAugmentedDescriptors(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
        int place_id = -1) const;

    void AddPlace(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, int place_id);
    void AddDescriptor(const ScanContextDescriptor& descriptor);
    void BuildIndex();
    void Clear();

    ScanContextMatch Query(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
    ScanContextMatch QueryDescriptor(const ScanContextDescriptor& query);

    std::size_t DescriptorCount() const { return m_Descriptors.size(); }
    std::size_t PlaceCount() const;
    const std::vector<ScanContextDescriptor>& Descriptors() const { return m_Descriptors; }

    static Eigen::VectorXf MakeRetrievalKey(const Eigen::MatrixXf& descriptor);
    static Eigen::VectorXf MakeAlignmentKey(const Eigen::MatrixXf& descriptor);
    static Eigen::MatrixXf CircularShiftColumns(const Eigen::MatrixXf& descriptor, int shift);
    static int FindBestAlignment(
        const Eigen::VectorXf& query_key,
        const Eigen::VectorXf& map_key);
    static float DescriptorDistance(
        const Eigen::MatrixXf& query,
        const Eigen::MatrixXf& map,
        int query_column_shift);

private:
    struct KeyIndexNode;
    struct KeyNeighbor;

    ScanContextDescriptor MakeDescriptorAtRoot(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
        int place_id,
        ScanContextVariant variant,
        float virtual_lateral_shift) const;
    ScanContextDescriptor MakeDescriptorFromMatrix(
        const Eigen::MatrixXf& matrix,
        int place_id,
        ScanContextVariant variant,
        float virtual_lateral_shift,
        float heading_offset_rad) const;

    void ValidateConfig() const;
    void ValidateDescriptor(const ScanContextDescriptor& descriptor) const;
    std::unique_ptr<KeyIndexNode> BuildIndexRecursive(
        std::vector<int>& indices, int begin, int end);
    void SearchIndex(
        const Eigen::VectorXf& query,
        int candidate_count,
        std::vector<KeyNeighbor>& neighbors) const;
    void SearchIndexRecursive(
        const KeyIndexNode* node,
        const Eigen::VectorXf& query,
        int candidate_count,
        std::vector<KeyNeighbor>& heap) const;
    std::vector<KeyNeighbor> RetrieveCandidates(const Eigen::VectorXf& query_key) const;
    ScanContextMatch CompareCandidate(
        const ScanContextDescriptor& query,
        int descriptor_index,
        float retrieval_key_distance) const;

    ScanContextConfig m_Config;
    std::vector<ScanContextDescriptor> m_Descriptors;
    std::unordered_map<int, std::vector<int>> m_PlaceToDescriptors;
    std::unique_ptr<KeyIndexNode> m_IndexRoot;
    bool m_IndexDirty = true;
};

} // namespace relocation
