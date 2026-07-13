#include "relocation/scan_context.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

constexpr float kPi = 3.14159265358979323846f;

pcl::PointCloud<pcl::PointXYZ>::Ptr MakePolarScene(bool alternate = false) {
    const std::vector<float> radii =
        alternate ? std::vector<float>{6.0f, 14.0f, 34.0f, 54.0f, 70.0f} :
                    std::vector<float>{10.0f, 18.0f, 30.0f, 46.0f, 66.0f};
    const std::vector<float> angles_deg =
        alternate ? std::vector<float>{9.0f, 39.0f, 159.0f, 207.0f, 333.0f} :
                    std::vector<float>{15.0f, 81.0f, 147.0f, 231.0f, 315.0f};
    const std::vector<float> heights = {0.0f, 0.8f, 1.6f, 2.4f, 3.2f};

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (std::size_t i = 0; i < radii.size(); ++i) {
        const float angle = angles_deg[i] * kPi / 180.0f;
        cloud->push_back(
            pcl::PointXYZ(
                radii[i] * std::cos(angle),
                radii[i] * std::sin(angle),
                heights[i]));
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MakeCartesianScene() {
    const std::vector<float> xs = {-47.5f, -12.5f, 22.5f, 57.5f};
    const std::vector<float> ys = {-15.0f, -3.0f, 9.0f, 21.0f};
    const std::vector<float> zs = {0.2f, 1.0f, 2.0f, 3.0f};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (std::size_t i = 0; i < xs.size(); ++i) {
        cloud->push_back(pcl::PointXYZ(xs[i], ys[i], zs[i]));
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RotateCloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    float yaw_rad) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated(new pcl::PointCloud<pcl::PointXYZ>());
    const float cosine = std::cos(yaw_rad);
    const float sine = std::sin(yaw_rad);
    for (const auto& point : cloud->points) {
        rotated->push_back(
            pcl::PointXYZ(
                cosine * point.x - sine * point.y,
                sine * point.x + cosine * point.y,
                point.z));
    }
    return rotated;
}

void Require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void TestPolarRotationAndRetrieval() {
    auto config = relocation::ScanContextConfig::PaperPolar();
    config.voxel_leaf_size = 0.0f;
    config.candidate_count = 2;
    config.distance_threshold = 0.01f;
    config.alignment_search_radius = 1;

    relocation::ScanContextPlusPlus scan_context(config);
    const auto map_cloud = MakePolarScene(false);
    const auto other_cloud = MakePolarScene(true);
    const auto rotated_query = RotateCloud(map_cloud, 60.0f * kPi / 180.0f);

    const auto map_descriptor = scan_context.MakeDescriptor(map_cloud);
    const auto query_descriptor = scan_context.MakeDescriptor(rotated_query);
    Require(
        (map_descriptor.retrieval_key - query_descriptor.retrieval_key).norm() < 1e-5f,
        "Polar retrieval key is not rotation invariant");

    const int shift = relocation::ScanContextPlusPlus::FindBestAlignment(
        query_descriptor.alignment_key, map_descriptor.alignment_key);
    const float descriptor_distance = relocation::ScanContextPlusPlus::DescriptorDistance(
        query_descriptor.matrix, map_descriptor.matrix, shift);
    Require(descriptor_distance < 1e-5f, "Polar descriptor alignment failed");

    const auto augmented = scan_context.MakeAugmentedDescriptors(map_cloud, 10);
    Require(augmented.size() == 3, "A-PC must contain original and two shifted variants");

    scan_context.AddPlace(other_cloud, 20);
    scan_context.AddPlace(map_cloud, 10);
    scan_context.BuildIndex();
    const auto match = scan_context.Query(rotated_query);
    Require(match.matched, "Polar query was rejected");
    Require(match.place_id == 10, "Polar query retrieved the wrong place");
    Require(
        std::abs(std::abs(match.relative_yaw_rad) - 60.0f * kPi / 180.0f) < 0.11f,
        "Polar relative yaw estimate is outside one column");
}

void TestCartesianDoubleFlip() {
    auto config = relocation::ScanContextConfig::PaperCartesian();
    config.voxel_leaf_size = 0.0f;
    config.distance_threshold = 0.01f;
    config.alignment_search_radius = 0;

    relocation::ScanContextPlusPlus scan_context(config);
    const auto map_cloud = MakeCartesianScene();
    const auto reversed_query = RotateCloud(map_cloud, kPi);
    const auto augmented = scan_context.MakeAugmentedDescriptors(map_cloud, 7);
    Require(augmented.size() == 2, "A-CC must contain original and double-flipped variants");
    Require(
        augmented[1].variant == relocation::ScanContextVariant::CartesianDoubleFlip,
        "A-CC variant type is incorrect");

    scan_context.AddPlace(map_cloud, 7);
    const auto match = scan_context.Query(reversed_query);
    Require(match.matched, "Cartesian reversed query was rejected");
    Require(match.place_id == 7, "Cartesian query retrieved the wrong place");
    Require(
        match.variant == relocation::ScanContextVariant::CartesianDoubleFlip,
        "Cartesian reversed query did not use the double-flip augmentation");
    Require(
        std::abs(std::abs(match.relative_yaw_rad) - kPi) < 1e-5f,
        "Cartesian double-flip heading estimate is incorrect");
}

void TestIndexMaintenance() {
    auto config = relocation::ScanContextConfig::PaperPolar();
    config.voxel_leaf_size = 0.0f;
    relocation::ScanContextPlusPlus scan_context(config);
    scan_context.AddPlace(MakePolarScene(false), 1);
    Require(scan_context.PlaceCount() == 1, "Place count is incorrect");
    Require(scan_context.DescriptorCount() == 3, "Augmented descriptor count is incorrect");
    scan_context.Clear();
    Require(scan_context.PlaceCount() == 0, "Place database clear failed");
    Require(scan_context.DescriptorCount() == 0, "Descriptor database clear failed");
}

void TestIrisCompatibleConfig() {
    const auto polar = relocation::ScanContextConfig::IrisPolar();
    Require(polar.type == relocation::ScanContextType::Polar, "IRIS-compatible PC type is incorrect");
    Require(polar.rows == 10 && polar.columns == 360, "IRIS-compatible PC resolution is incorrect");
    Require(polar.min_radius == 0.0f && polar.max_radius == 10.0f,
            "IRIS-compatible PC range is incorrect");
    Require(polar.min_z == 0.0f && polar.max_z == 2.0f,
            "IRIS-compatible PC height range is incorrect");
    Require(polar.voxel_leaf_size == 0.0f, "IRIS-compatible PC must not downsample internally");
    Require(polar.candidate_count == 8 && polar.alignment_search_radius == 2,
            "IRIS-compatible PC search parameters are incorrect");

    const auto cartesian = relocation::ScanContextConfig::IrisCartesian();
    Require(cartesian.type == relocation::ScanContextType::Cartesian,
            "IRIS-compatible CC type is incorrect");
    Require(cartesian.rows == 20 && cartesian.columns == 20,
            "IRIS-compatible CC resolution is incorrect");
    Require(cartesian.min_x == -10.0f && cartesian.max_x == 10.0f &&
            cartesian.min_y == -10.0f && cartesian.max_y == 10.0f,
            "IRIS-compatible CC bounds are incorrect");
}

} // namespace

int main() {
    try {
        TestPolarRotationAndRetrieval();
        TestCartesianDoubleFlip();
        TestIndexMaintenance();
        TestIrisCompatibleConfig();
    } catch (const std::exception& exception) {
        std::cerr << "SC++ test failed: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "SC++ standalone tests passed\n";
    return 0;
}
