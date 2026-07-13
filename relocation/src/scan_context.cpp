#include "relocation/scan_context.hpp"
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace relocation {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kNormEpsilon = 1e-6f;
constexpr char kDatabaseMagic[8] = {'S', 'C', 'D', 'B', '0', '0', '1', '\0'};
constexpr std::uint32_t kDatabaseVersion = 1;

template<typename T>
bool WriteBinary(std::ostream& stream, const T& value) {
    stream.write(reinterpret_cast<const char*>(&value), sizeof(T));
    return stream.good();
}

template<typename T>
bool ReadBinary(std::istream& stream, T& value) {
    stream.read(reinterpret_cast<char*>(&value), sizeof(T));
    return stream.good();
}

bool NearlyEqual(float lhs, float rhs) {
    return std::abs(lhs - rhs) <= 1e-6f;
}

bool DescriptorConfigMatches(const ScanContextConfig& lhs, const ScanContextConfig& rhs) {
    return lhs.type == rhs.type &&
           lhs.rows == rhs.rows &&
           lhs.columns == rhs.columns &&
           NearlyEqual(lhs.min_radius, rhs.min_radius) &&
           NearlyEqual(lhs.max_radius, rhs.max_radius) &&
           NearlyEqual(lhs.min_x, rhs.min_x) &&
           NearlyEqual(lhs.max_x, rhs.max_x) &&
           NearlyEqual(lhs.min_y, rhs.min_y) &&
           NearlyEqual(lhs.max_y, rhs.max_y) &&
           NearlyEqual(lhs.min_z, rhs.min_z) &&
           NearlyEqual(lhs.max_z, rhs.max_z) &&
           NearlyEqual(lhs.height_offset, rhs.height_offset) &&
           NearlyEqual(lhs.voxel_leaf_size, rhs.voxel_leaf_size) &&
           lhs.enable_augmentation == rhs.enable_augmentation &&
           NearlyEqual(lhs.polar_lateral_augmentation, rhs.polar_lateral_augmentation);
}

int WrapIndex(int index, int size) {
    index %= size;
    if (index < 0) {
        index += size;
    }
    return index;
}

int SignedShift(int shift, int columns) {
    shift = WrapIndex(shift, columns);
    if (shift > columns / 2) {
        shift -= columns;
    }
    return shift;
}

float SquaredDistance(const Eigen::VectorXf& lhs, const Eigen::VectorXf& rhs) {
    return (lhs - rhs).squaredNorm();
}

Eigen::VectorXf CircularShiftVector(const Eigen::VectorXf& vector, int shift) {
    Eigen::VectorXf shifted(vector.size());
    const int size = static_cast<int>(vector.size());
    for (int i = 0; i < size; ++i) {
        shifted[WrapIndex(i + shift, size)] = vector[i];
    }
    return shifted;
}

} // namespace

std::string MakeScanContextDatabasePath(const std::string& iris_database_path) {
    const std::filesystem::path iris_path(iris_database_path);
    const std::string stem = iris_path.stem().empty() ? "history_db" : iris_path.stem().string();
    return (iris_path.parent_path() / (stem + "_sc.bin")).string();
}

struct ScanContextPlusPlus::KeyIndexNode {
    int descriptor_index = -1;
    int axis = 0;
    float split_value = 0.0f;
    std::unique_ptr<KeyIndexNode> left;
    std::unique_ptr<KeyIndexNode> right;
};

struct ScanContextPlusPlus::KeyNeighbor {
    int descriptor_index = -1;
    float squared_distance = std::numeric_limits<float>::max();

    bool operator<(const KeyNeighbor& other) const {
        return squared_distance < other.squared_distance;
    }
};

ScanContextConfig ScanContextConfig::IrisPolar() {
    ScanContextConfig config;
    config.type = ScanContextType::Polar;
    return config;
}

ScanContextConfig ScanContextConfig::IrisCartesian() {
    ScanContextConfig config;
    config.type = ScanContextType::Cartesian;
    config.rows = 20;
    config.columns = 20;
    return config;
}

ScanContextConfig ScanContextConfig::PaperPolar() {
    ScanContextConfig config;
    config.type = ScanContextType::Polar;
    config.rows = 20;
    config.columns = 60;
    config.min_radius = 0.0f;
    config.max_radius = 80.0f;
    config.min_z = -2.0f;
    config.max_z = 10.0f;
    config.height_offset = 2.0f;
    config.voxel_leaf_size = 0.5f;
    config.candidate_count = 1;
    config.alignment_search_radius = 1;
    config.enable_augmentation = true;
    config.polar_lateral_augmentation = 2.0f;
    return config;
}

ScanContextConfig ScanContextConfig::PaperCartesian() {
    ScanContextConfig config;
    config.type = ScanContextType::Cartesian;
    config.rows = 40;
    config.columns = 40;
    config.min_x = -100.0f;
    config.max_x = 100.0f;
    config.min_y = -40.0f;
    config.max_y = 40.0f;
    config.min_z = -2.0f;
    config.max_z = 10.0f;
    config.height_offset = 2.0f;
    config.voxel_leaf_size = 0.5f;
    config.candidate_count = 1;
    config.alignment_search_radius = 1;
    config.enable_augmentation = true;
    return config;
}

ScanContextPlusPlus::ScanContextPlusPlus(ScanContextConfig config)
    : m_Config(std::move(config)) {
    ValidateConfig();
}

ScanContextPlusPlus::~ScanContextPlusPlus() = default;
ScanContextPlusPlus::ScanContextPlusPlus(ScanContextPlusPlus&&) noexcept = default;
ScanContextPlusPlus& ScanContextPlusPlus::operator=(ScanContextPlusPlus&&) noexcept = default;

void ScanContextPlusPlus::ValidateConfig() const {
    if (m_Config.rows <= 0 || m_Config.columns <= 0) {
        throw std::invalid_argument("Scan Context rows and columns must be positive");
    }
    if (m_Config.candidate_count <= 0 || m_Config.alignment_search_radius < 0) {
        throw std::invalid_argument("Scan Context search parameters are invalid");
    }
    if (m_Config.min_z >= m_Config.max_z || m_Config.max_z + m_Config.height_offset <= 0.0f) {
        throw std::invalid_argument("Scan Context height limits are invalid");
    }
    if (m_Config.voxel_leaf_size < 0.0f || m_Config.distance_threshold < 0.0f) {
        throw std::invalid_argument("Scan Context filter or threshold is invalid");
    }
    if (m_Config.type == ScanContextType::Polar) {
        if (m_Config.min_radius < 0.0f || m_Config.min_radius >= m_Config.max_radius) {
            throw std::invalid_argument("Polar Context radius limits are invalid");
        }
        if (m_Config.polar_lateral_augmentation < 0.0f) {
            throw std::invalid_argument("Polar Context augmentation must be non-negative");
        }
    } else if (m_Config.min_x >= m_Config.max_x || m_Config.min_y >= m_Config.max_y) {
        throw std::invalid_argument("Cart Context bounds are invalid");
    }
}

void ScanContextPlusPlus::ValidateDescriptor(const ScanContextDescriptor& descriptor) const {
    if (descriptor.matrix.rows() != m_Config.rows ||
        descriptor.matrix.cols() != m_Config.columns) {
        throw std::invalid_argument("Scan Context descriptor dimensions do not match the configuration");
    }
    if (descriptor.retrieval_key.size() != m_Config.rows ||
        descriptor.alignment_key.size() != m_Config.columns) {
        throw std::invalid_argument("Scan Context key dimensions do not match the configuration");
    }
    if (!descriptor.matrix.allFinite() ||
        !descriptor.retrieval_key.allFinite() ||
        !descriptor.alignment_key.allFinite()) {
        throw std::invalid_argument("Scan Context descriptor contains non-finite values");
    }
}

ScanContextDescriptor ScanContextPlusPlus::MakeDescriptor(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    int place_id) const {
    return MakeDescriptorAtRoot(
        cloud, place_id, ScanContextVariant::Original, 0.0f);
}

ScanContextDescriptor ScanContextPlusPlus::MakeDescriptorAtRoot(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    int place_id,
    ScanContextVariant variant,
    float virtual_lateral_shift) const {
    if (!cloud || cloud->empty()) {
        throw std::invalid_argument("Cannot create a Scan Context descriptor from an empty cloud");
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr processed(new pcl::PointCloud<pcl::PointXYZ>());
    if (m_Config.voxel_leaf_size > 0.0f) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setLeafSize(
            m_Config.voxel_leaf_size,
            m_Config.voxel_leaf_size,
            m_Config.voxel_leaf_size);
        voxel.setInputCloud(cloud);
        voxel.filter(*processed);
    } else {
        *processed = *cloud;
    }

    Eigen::MatrixXf matrix = Eigen::MatrixXf::Zero(m_Config.rows, m_Config.columns);
    const float radius_scale = static_cast<float>(m_Config.rows) /
                               (m_Config.max_radius - m_Config.min_radius);
    const float x_scale = static_cast<float>(m_Config.rows) /
                          (m_Config.max_x - m_Config.min_x);
    const float y_scale = static_cast<float>(m_Config.columns) /
                          (m_Config.max_y - m_Config.min_y);
    const float max_encoded_height = m_Config.max_z + m_Config.height_offset;

    for (const auto& point : processed->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }
        if (point.z < m_Config.min_z || point.z > m_Config.max_z) {
            continue;
        }

        const float local_x = point.x;
        const float local_y = point.y - virtual_lateral_shift;
        int row = -1;
        int column = -1;

        if (m_Config.type == ScanContextType::Polar) {
            const float radius = std::hypot(local_x, local_y);
            if (radius < m_Config.min_radius || radius >= m_Config.max_radius) {
                continue;
            }
            const float angle = std::atan2(local_y, local_x) + kPi;
            row = static_cast<int>((radius - m_Config.min_radius) * radius_scale);
            column = static_cast<int>(std::floor(
                                          angle / kTwoPi * static_cast<float>(m_Config.columns) + 0.5f));
        } else {
            if (local_x < m_Config.min_x || local_x >= m_Config.max_x ||
                local_y < m_Config.min_y || local_y >= m_Config.max_y) {
                continue;
            }
            row = static_cast<int>((local_x - m_Config.min_x) * x_scale);
            column = static_cast<int>((local_y - m_Config.min_y) * y_scale);
        }

        row = std::clamp(row, 0, m_Config.rows - 1);
        column = std::clamp(column, 0, m_Config.columns - 1);
        const float encoded_height = std::clamp(
            point.z + m_Config.height_offset, kNormEpsilon, max_encoded_height);
        matrix(row, column) = std::max(matrix(row, column), encoded_height);
    }

    return MakeDescriptorFromMatrix(
        matrix, place_id, variant, virtual_lateral_shift, 0.0f);
}

ScanContextDescriptor ScanContextPlusPlus::MakeDescriptorFromMatrix(
    const Eigen::MatrixXf& matrix,
    int place_id,
    ScanContextVariant variant,
    float virtual_lateral_shift,
    float heading_offset_rad) const {
    ScanContextDescriptor descriptor;
    descriptor.matrix = matrix;
    descriptor.retrieval_key = MakeRetrievalKey(matrix);
    descriptor.alignment_key = MakeAlignmentKey(matrix);
    descriptor.place_id = place_id;
    descriptor.variant = variant;
    descriptor.virtual_lateral_shift = virtual_lateral_shift;
    descriptor.heading_offset_rad = heading_offset_rad;
    return descriptor;
}

std::vector<ScanContextDescriptor> ScanContextPlusPlus::MakeAugmentedDescriptors(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    int place_id) const {
    std::vector<ScanContextDescriptor> descriptors;
    descriptors.push_back(MakeDescriptor(cloud, place_id));
    if (!m_Config.enable_augmentation) {
        return descriptors;
    }

    if (m_Config.type == ScanContextType::Polar &&
        m_Config.polar_lateral_augmentation > 0.0f) {
        descriptors.push_back(
            MakeDescriptorAtRoot(
                cloud, place_id, ScanContextVariant::PolarLeftShift,
                m_Config.polar_lateral_augmentation));
        descriptors.push_back(
            MakeDescriptorAtRoot(
                cloud, place_id, ScanContextVariant::PolarRightShift,
                -m_Config.polar_lateral_augmentation));
    } else if (m_Config.type == ScanContextType::Cartesian) {
        const Eigen::MatrixXf& original = descriptors.front().matrix;
        Eigen::MatrixXf flipped(original.rows(), original.cols());
        for (int row = 0; row < original.rows(); ++row) {
            for (int column = 0; column < original.cols(); ++column) {
                flipped(row, column) = original(
                    original.rows() - 1 - row,
                    original.cols() - 1 - column);
            }
        }
        descriptors.push_back(
            MakeDescriptorFromMatrix(
                flipped, place_id, ScanContextVariant::CartesianDoubleFlip,
                0.0f, kPi));
    }
    return descriptors;
}

Eigen::VectorXf ScanContextPlusPlus::MakeRetrievalKey(const Eigen::MatrixXf& descriptor) {
    Eigen::VectorXf key(descriptor.rows());
    for (int row = 0; row < descriptor.rows(); ++row) {
        key[row] = descriptor.row(row).cwiseAbs().sum() /
                   static_cast<float>(descriptor.cols());
    }
    return key;
}

Eigen::VectorXf ScanContextPlusPlus::MakeAlignmentKey(const Eigen::MatrixXf& descriptor) {
    Eigen::VectorXf key(descriptor.cols());
    for (int column = 0; column < descriptor.cols(); ++column) {
        key[column] = descriptor.col(column).cwiseAbs().sum() /
                      static_cast<float>(descriptor.rows());
    }
    return key;
}

Eigen::MatrixXf ScanContextPlusPlus::CircularShiftColumns(
    const Eigen::MatrixXf& descriptor,
    int shift) {
    Eigen::MatrixXf shifted(descriptor.rows(), descriptor.cols());
    const int columns = descriptor.cols();
    for (int column = 0; column < columns; ++column) {
        shifted.col(WrapIndex(column + shift, columns)) = descriptor.col(column);
    }
    return shifted;
}

int ScanContextPlusPlus::FindBestAlignment(
    const Eigen::VectorXf& query_key,
    const Eigen::VectorXf& map_key) {
    if (query_key.size() == 0 || query_key.size() != map_key.size()) {
        throw std::invalid_argument("Scan Context alignment keys have incompatible dimensions");
    }

    int best_shift = 0;
    float best_distance = std::numeric_limits<float>::max();
    for (int shift = 0; shift < query_key.size(); ++shift) {
        const float distance = SquaredDistance(
            CircularShiftVector(query_key, shift), map_key);
        if (distance < best_distance) {
            best_distance = distance;
            best_shift = shift;
        }
    }
    return best_shift;
}

float ScanContextPlusPlus::DescriptorDistance(
    const Eigen::MatrixXf& query,
    const Eigen::MatrixXf& map,
    int query_column_shift) {
    if (query.rows() != map.rows() || query.cols() != map.cols() || query.cols() == 0) {
        throw std::invalid_argument("Scan Context matrices have incompatible dimensions");
    }

    const Eigen::MatrixXf aligned_query = CircularShiftColumns(query, query_column_shift);
    float similarity_sum = 0.0f;
    int effective_columns = 0;
    for (int column = 0; column < query.cols(); ++column) {
        const auto query_column = aligned_query.col(column);
        const auto map_column = map.col(column);
        const float query_norm = query_column.norm();
        const float map_norm = map_column.norm();
        if (query_norm <= kNormEpsilon || map_norm <= kNormEpsilon) {
            continue;
        }

        float cosine_similarity = query_column.dot(map_column) / (query_norm * map_norm);
        cosine_similarity = std::clamp(cosine_similarity, -1.0f, 1.0f);
        similarity_sum += cosine_similarity;
        effective_columns++;
    }

    if (effective_columns == 0) {
        return 1.0f;
    }
    return 1.0f - similarity_sum / static_cast<float>(effective_columns);
}

void ScanContextPlusPlus::AddPlace(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    int place_id) {
    if (place_id < 0) {
        throw std::invalid_argument("Scan Context place id must be non-negative");
    }
    if (m_PlaceToDescriptors.find(place_id) != m_PlaceToDescriptors.end()) {
        throw std::invalid_argument("Scan Context place id already exists");
    }
    for (const auto& descriptor : MakeAugmentedDescriptors(cloud, place_id)) {
        AddDescriptor(descriptor);
    }
}

void ScanContextPlusPlus::AddDescriptor(const ScanContextDescriptor& descriptor) {
    ValidateDescriptor(descriptor);
    if (descriptor.place_id < 0) {
        throw std::invalid_argument("A database descriptor must have a non-negative place id");
    }
    const int descriptor_index = static_cast<int>(m_Descriptors.size());
    m_Descriptors.push_back(descriptor);
    m_PlaceToDescriptors[descriptor.place_id].push_back(descriptor_index);
    m_IndexDirty = true;
}

bool ScanContextPlusPlus::SaveDatabase(const std::string& filename) const {
    if (filename.empty() || m_Descriptors.empty()) {
        return false;
    }

    std::ofstream output(filename, std::ios::binary | std::ios::trunc);
    if (!output.is_open()) {
        return false;
    }

    output.write(kDatabaseMagic, sizeof(kDatabaseMagic));
    const std::int32_t type = static_cast<std::int32_t>(m_Config.type);
    const std::int32_t rows = m_Config.rows;
    const std::int32_t columns = m_Config.columns;
    const std::int32_t candidate_count = m_Config.candidate_count;
    const std::int32_t alignment_radius = m_Config.alignment_search_radius;
    const std::uint8_t augmentation = m_Config.enable_augmentation ? 1 : 0;
    const std::uint64_t descriptor_count = m_Descriptors.size();
    if (!WriteBinary(output, kDatabaseVersion) ||
        !WriteBinary(output, type) ||
        !WriteBinary(output, rows) ||
        !WriteBinary(output, columns) ||
        !WriteBinary(output, m_Config.min_radius) ||
        !WriteBinary(output, m_Config.max_radius) ||
        !WriteBinary(output, m_Config.min_x) ||
        !WriteBinary(output, m_Config.max_x) ||
        !WriteBinary(output, m_Config.min_y) ||
        !WriteBinary(output, m_Config.max_y) ||
        !WriteBinary(output, m_Config.min_z) ||
        !WriteBinary(output, m_Config.max_z) ||
        !WriteBinary(output, m_Config.height_offset) ||
        !WriteBinary(output, m_Config.voxel_leaf_size) ||
        !WriteBinary(output, candidate_count) ||
        !WriteBinary(output, alignment_radius) ||
        !WriteBinary(output, m_Config.distance_threshold) ||
        !WriteBinary(output, augmentation) ||
        !WriteBinary(output, m_Config.polar_lateral_augmentation) ||
        !WriteBinary(output, descriptor_count)) {
        return false;
    }

    for (const auto& descriptor : m_Descriptors) {
        const std::int32_t place_id = descriptor.place_id;
        const std::int32_t variant = static_cast<std::int32_t>(descriptor.variant);
        const std::int32_t matrix_rows = descriptor.matrix.rows();
        const std::int32_t matrix_columns = descriptor.matrix.cols();
        if (!WriteBinary(output, place_id) ||
            !WriteBinary(output, variant) ||
            !WriteBinary(output, descriptor.virtual_lateral_shift) ||
            !WriteBinary(output, descriptor.heading_offset_rad) ||
            !WriteBinary(output, matrix_rows) ||
            !WriteBinary(output, matrix_columns)) {
            return false;
        }
        for (int row = 0; row < matrix_rows; ++row) {
            for (int column = 0; column < matrix_columns; ++column) {
                if (!WriteBinary(output, descriptor.matrix(row, column))) {
                    return false;
                }
            }
        }
    }
    return output.good();
}

bool ScanContextPlusPlus::LoadDatabase(const std::string& filename) {
    std::ifstream input(filename, std::ios::binary);
    if (!input.is_open()) {
        return false;
    }

    char magic[sizeof(kDatabaseMagic)]{};
    input.read(magic, sizeof(magic));
    if (!input.good() || !std::equal(std::begin(magic), std::end(magic), std::begin(kDatabaseMagic))) {
        return false;
    }

    std::uint32_t version = 0;
    std::int32_t type = 0;
    std::int32_t rows = 0;
    std::int32_t columns = 0;
    std::int32_t candidate_count = 0;
    std::int32_t alignment_radius = 0;
    std::uint8_t augmentation = 0;
    std::uint64_t descriptor_count = 0;
    ScanContextConfig file_config;
    if (!ReadBinary(input, version) ||
        !ReadBinary(input, type) ||
        !ReadBinary(input, rows) ||
        !ReadBinary(input, columns) ||
        !ReadBinary(input, file_config.min_radius) ||
        !ReadBinary(input, file_config.max_radius) ||
        !ReadBinary(input, file_config.min_x) ||
        !ReadBinary(input, file_config.max_x) ||
        !ReadBinary(input, file_config.min_y) ||
        !ReadBinary(input, file_config.max_y) ||
        !ReadBinary(input, file_config.min_z) ||
        !ReadBinary(input, file_config.max_z) ||
        !ReadBinary(input, file_config.height_offset) ||
        !ReadBinary(input, file_config.voxel_leaf_size) ||
        !ReadBinary(input, candidate_count) ||
        !ReadBinary(input, alignment_radius) ||
        !ReadBinary(input, file_config.distance_threshold) ||
        !ReadBinary(input, augmentation) ||
        !ReadBinary(input, file_config.polar_lateral_augmentation) ||
        !ReadBinary(input, descriptor_count)) {
        return false;
    }
    file_config.type = static_cast<ScanContextType>(type);
    file_config.rows = rows;
    file_config.columns = columns;
    file_config.candidate_count = candidate_count;
    file_config.alignment_search_radius = alignment_radius;
    file_config.enable_augmentation = augmentation != 0;
    if (version != kDatabaseVersion ||
        descriptor_count == 0 ||
        descriptor_count > 100000000 ||
        !DescriptorConfigMatches(file_config, m_Config)) {
        return false;
    }

    Clear();
    try {
        for (std::uint64_t i = 0; i < descriptor_count; ++i) {
            std::int32_t place_id = -1;
            std::int32_t variant = 0;
            std::int32_t matrix_rows = 0;
            std::int32_t matrix_columns = 0;
            float lateral_shift = 0.0f;
            float heading_offset = 0.0f;
            if (!ReadBinary(input, place_id) ||
                !ReadBinary(input, variant) ||
                !ReadBinary(input, lateral_shift) ||
                !ReadBinary(input, heading_offset) ||
                !ReadBinary(input, matrix_rows) ||
                !ReadBinary(input, matrix_columns) ||
                matrix_rows != m_Config.rows ||
                matrix_columns != m_Config.columns ||
                variant < static_cast<std::int32_t>(ScanContextVariant::Original) ||
                variant > static_cast<std::int32_t>(ScanContextVariant::CartesianDoubleFlip)) {
                Clear();
                return false;
            }

            Eigen::MatrixXf matrix(matrix_rows, matrix_columns);
            for (int row = 0; row < matrix_rows; ++row) {
                for (int column = 0; column < matrix_columns; ++column) {
                    if (!ReadBinary(input, matrix(row, column))) {
                        Clear();
                        return false;
                    }
                }
            }
            AddDescriptor(MakeDescriptorFromMatrix(
                matrix,
                place_id,
                static_cast<ScanContextVariant>(variant),
                lateral_shift,
                heading_offset));
        }
    } catch (const std::exception&) {
        Clear();
        return false;
    }
    BuildIndex();
    return true;
}

std::unique_ptr<ScanContextPlusPlus::KeyIndexNode>
ScanContextPlusPlus::BuildIndexRecursive(
    std::vector<int>& indices,
    int begin,
    int end) {
    if (begin >= end) {
        return nullptr;
    }

    const int dimension = m_Config.rows;
    Eigen::VectorXf mean = Eigen::VectorXf::Zero(dimension);
    for (int i = begin; i < end; ++i) {
        mean += m_Descriptors[indices[i]].retrieval_key;
    }
    mean /= static_cast<float>(end - begin);

    Eigen::VectorXf variance = Eigen::VectorXf::Zero(dimension);
    for (int i = begin; i < end; ++i) {
        const Eigen::VectorXf difference = m_Descriptors[indices[i]].retrieval_key - mean;
        variance += difference.cwiseProduct(difference);
    }
    Eigen::Index axis = 0;
    variance.maxCoeff(&axis);

    const int middle = begin + (end - begin) / 2;
    std::nth_element(
        indices.begin() + begin,
        indices.begin() + middle,
        indices.begin() + end,
        [&](int lhs, int rhs) {
            return m_Descriptors[lhs].retrieval_key[axis] <
                   m_Descriptors[rhs].retrieval_key[axis];
        });

    auto node = std::make_unique<KeyIndexNode>();
    node->descriptor_index = indices[middle];
    node->axis = static_cast<int>(axis);
    node->split_value = m_Descriptors[indices[middle]].retrieval_key[axis];
    node->left = BuildIndexRecursive(indices, begin, middle);
    node->right = BuildIndexRecursive(indices, middle + 1, end);
    return node;
}

void ScanContextPlusPlus::BuildIndex() {
    if (m_Descriptors.empty()) {
        m_IndexRoot.reset();
        m_IndexDirty = false;
        return;
    }
    std::vector<int> indices(m_Descriptors.size());
    for (int i = 0; i < static_cast<int>(indices.size()); ++i) {
        indices[i] = i;
    }
    m_IndexRoot = BuildIndexRecursive(indices, 0, static_cast<int>(indices.size()));
    m_IndexDirty = false;
}

void ScanContextPlusPlus::SearchIndexRecursive(
    const KeyIndexNode* node,
    const Eigen::VectorXf& query,
    int candidate_count,
    std::vector<KeyNeighbor>& heap) const {
    if (!node) {
        return;
    }

    const float squared_distance = SquaredDistance(
        query, m_Descriptors[node->descriptor_index].retrieval_key);
    KeyNeighbor neighbor{node->descriptor_index, squared_distance};
    if (static_cast<int>(heap.size()) < candidate_count) {
        heap.push_back(neighbor);
        std::push_heap(heap.begin(), heap.end());
    } else if (squared_distance < heap.front().squared_distance) {
        std::pop_heap(heap.begin(), heap.end());
        heap.back() = neighbor;
        std::push_heap(heap.begin(), heap.end());
    }

    const float axis_difference = query[node->axis] - node->split_value;
    const KeyIndexNode* near_node = axis_difference < 0.0f ?
                                    node->left.get() :
                                    node->right.get();
    const KeyIndexNode* far_node = axis_difference < 0.0f ?
                                   node->right.get() :
                                   node->left.get();
    SearchIndexRecursive(near_node, query, candidate_count, heap);

    const float worst_distance = static_cast<int>(heap.size()) < candidate_count ?
                                 std::numeric_limits<float>::max() :
                                 heap.front().squared_distance;
    if (axis_difference * axis_difference <= worst_distance) {
        SearchIndexRecursive(far_node, query, candidate_count, heap);
    }
}

void ScanContextPlusPlus::SearchIndex(
    const Eigen::VectorXf& query,
    int candidate_count,
    std::vector<KeyNeighbor>& neighbors) const {
    neighbors.clear();
    if (!m_IndexRoot || candidate_count <= 0) {
        return;
    }
    candidate_count = std::min(candidate_count, static_cast<int>(m_Descriptors.size()));
    SearchIndexRecursive(m_IndexRoot.get(), query, candidate_count, neighbors);
    std::sort(
        neighbors.begin(), neighbors.end(), [](const auto& lhs, const auto& rhs) {
            if (lhs.squared_distance != rhs.squared_distance) {
                return lhs.squared_distance < rhs.squared_distance;
            }
            return lhs.descriptor_index < rhs.descriptor_index;
        });
}

std::vector<ScanContextPlusPlus::KeyNeighbor>
ScanContextPlusPlus::RetrieveCandidates(
    const Eigen::VectorXf& query_key,
    int place_count) const {
    const int variants_per_place = m_Config.type == ScanContextType::Polar ? 3 : 2;
    const int probe_count = std::min(
        static_cast<int>(m_Descriptors.size()),
        std::max(place_count, place_count * variants_per_place * 2));
    std::vector<KeyNeighbor> nearest;
    SearchIndex(query_key, probe_count, nearest);

    std::vector<int> selected_places;
    std::unordered_set<int> seen_places;
    for (const auto& neighbor : nearest) {
        const int place_id = m_Descriptors[neighbor.descriptor_index].place_id;
        if (seen_places.insert(place_id).second) {
            selected_places.push_back(place_id);
            if (static_cast<int>(selected_places.size()) >= place_count) {
                break;
            }
        }
    }

    std::vector<KeyNeighbor> candidates;
    for (int place_id : selected_places) {
        const auto place_iter = m_PlaceToDescriptors.find(place_id);
        if (place_iter == m_PlaceToDescriptors.end()) {
            continue;
        }
        for (int descriptor_index : place_iter->second) {
            candidates.push_back(
                KeyNeighbor{
                    descriptor_index,
                    SquaredDistance(query_key, m_Descriptors[descriptor_index].retrieval_key)});
        }
    }
    return candidates;
}

ScanContextMatch ScanContextPlusPlus::CompareCandidate(
    const ScanContextDescriptor& query,
    int descriptor_index,
    float retrieval_key_distance) const {
    const auto& candidate = m_Descriptors[descriptor_index];
    const int initial_shift = FindBestAlignment(
        query.alignment_key, candidate.alignment_key);

    int best_shift = initial_shift;
    float best_distance = std::numeric_limits<float>::max();
    for (int delta = -m_Config.alignment_search_radius;
         delta <= m_Config.alignment_search_radius;
         ++delta) {
        const int shift = WrapIndex(initial_shift + delta, m_Config.columns);
        const float distance = DescriptorDistance(query.matrix, candidate.matrix, shift);
        if (distance < best_distance) {
            best_distance = distance;
            best_shift = shift;
        }
    }

    const int signed_shift = SignedShift(best_shift, m_Config.columns);
    ScanContextMatch match;
    match.place_id = candidate.place_id;
    match.descriptor_index = descriptor_index;
    match.variant = candidate.variant;
    match.distance = best_distance;
    match.retrieval_key_distance = std::sqrt(retrieval_key_distance);
    match.column_shift = signed_shift;
    if (m_Config.type == ScanContextType::Polar) {
        const float radians_per_column = kTwoPi / static_cast<float>(m_Config.columns);
        match.relative_yaw_rad = -static_cast<float>(signed_shift) * radians_per_column;
        match.relative_lateral_m = candidate.virtual_lateral_shift;
    } else {
        const float meters_per_column =
            (m_Config.max_y - m_Config.min_y) / static_cast<float>(m_Config.columns);
        match.relative_yaw_rad = candidate.heading_offset_rad;
        match.relative_lateral_m = -static_cast<float>(signed_shift) * meters_per_column;
    }
    match.matched = best_distance <= m_Config.distance_threshold;
    return match;
}

ScanContextMatch ScanContextPlusPlus::Query(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    return QueryDescriptor(MakeDescriptor(cloud));
}

ScanContextMatch ScanContextPlusPlus::QueryDescriptor(
    const ScanContextDescriptor& query) {
    const auto candidates = QueryCandidates(query, m_Config.candidate_count);
    if (candidates.empty()) {
        return ScanContextMatch{};
    }
    return candidates.front();
}

std::vector<ScanContextMatch> ScanContextPlusPlus::QueryCandidates(
    const ScanContextDescriptor& query,
    int place_count) {
    ValidateDescriptor(query);
    if (m_IndexDirty) {
        BuildIndex();
    }
    if (m_Descriptors.empty() || place_count <= 0) {
        return {};
    }

    std::unordered_map<int, ScanContextMatch> best_by_place;
    for (const auto& candidate : RetrieveCandidates(query.retrieval_key, place_count)) {
        ScanContextMatch match = CompareCandidate(
            query, candidate.descriptor_index, candidate.squared_distance);
        const auto iter = best_by_place.find(match.place_id);
        if (iter == best_by_place.end() ||
            match.distance < iter->second.distance ||
            (match.distance == iter->second.distance &&
             match.retrieval_key_distance < iter->second.retrieval_key_distance)) {
            best_by_place[match.place_id] = match;
        }
    }

    std::vector<ScanContextMatch> matches;
    matches.reserve(best_by_place.size());
    for (const auto& entry : best_by_place) {
        matches.push_back(entry.second);
    }
    std::sort(matches.begin(), matches.end(), [](const auto& lhs, const auto& rhs) {
        if (lhs.distance != rhs.distance) return lhs.distance < rhs.distance;
        if (lhs.retrieval_key_distance != rhs.retrieval_key_distance) {
            return lhs.retrieval_key_distance < rhs.retrieval_key_distance;
        }
        return lhs.place_id < rhs.place_id;
    });
    if (static_cast<int>(matches.size()) > place_count) {
        matches.resize(place_count);
    }
    return matches;
}

ScanContextMatch ScanContextPlusPlus::ComparePlace(
    const ScanContextDescriptor& query,
    int place_id) const {
    ValidateDescriptor(query);
    const auto place_iter = m_PlaceToDescriptors.find(place_id);
    if (place_iter == m_PlaceToDescriptors.end()) {
        return ScanContextMatch{};
    }

    ScanContextMatch best_match;
    best_match.distance = std::numeric_limits<float>::max();
    for (int descriptor_index : place_iter->second) {
        const float retrieval_distance = SquaredDistance(
            query.retrieval_key,
            m_Descriptors[descriptor_index].retrieval_key);
        const ScanContextMatch match = CompareCandidate(
            query, descriptor_index, retrieval_distance);
        if (match.distance < best_match.distance ||
            (match.distance == best_match.distance &&
             match.retrieval_key_distance < best_match.retrieval_key_distance)) {
            best_match = match;
        }
    }
    best_match.matched = best_match.distance <= m_Config.distance_threshold;
    return best_match;
}

void ScanContextPlusPlus::Clear() {
    m_Descriptors.clear();
    m_PlaceToDescriptors.clear();
    m_IndexRoot.reset();
    m_IndexDirty = true;
}

std::size_t ScanContextPlusPlus::PlaceCount() const {
    return m_PlaceToDescriptors.size();
}

} // namespace relocation
