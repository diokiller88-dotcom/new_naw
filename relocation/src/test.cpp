#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <limits>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>
#include "relocation/location.hpp" 

using namespace std;
using namespace relocation;

constexpr float test_resolution = 0.01f;   
constexpr int test_padding = 150;
constexpr int test_display_max_width = 1600;
constexpr int test_display_max_height = 1000;
constexpr float test_arrow_length = 35.0f;
constexpr float test_z_min = 0.2f;
constexpr float test_z_max = 2.5f;
constexpr float test_grid_step = 0.5f;

constexpr float test_max_distance = 10.0f; 

constexpr float test_collision_radius = 0.5f;
constexpr int test_collision_max_pts = 10;
constexpr float test_empty_radius = 10.0f;
constexpr int test_empty_min_pts = 500;
constexpr int test_sector_empty_threshold = 10;
constexpr int test_sector_max_empty = 4;

constexpr float test_edit_delete_radius = 1.5f;
constexpr float test_edit_add_min_dist = 0.45f;
constexpr float test_drag_min_dist = 5.0f;

constexpr float test_prior_offset_x = 0.5f;
constexpr float test_prior_offset_y = -0.8f;
constexpr double test_min_rough_score_gap = 0.02;
constexpr double test_min_rough_score_ratio = 1.10;
constexpr double test_min_gicp_error_gap = 0.05;
constexpr double test_min_gicp_error_ratio = 1.08;
constexpr int test_min_gicp_valid_count = 30;
constexpr double test_min_gicp_valid_ratio = 0.02;
constexpr size_t test_max_gicp_candidates = 3;
constexpr double test_max_rough_score_ratio_for_gicp = 1.35;
constexpr double test_early_accept_gicp_error = 0.25;
constexpr int test_candidate_gicp_max_iterations = 6;
constexpr float test_candidate_gicp_voxel_leaf_size = 0.5f;
constexpr double test_same_solution_translation = 0.5;
constexpr double test_same_solution_yaw_deg = 5.0;

struct TestGicpCandidateResult {
    RoughPoseCandidate rough;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d T = Eigen::Vector3d::Zero();
    double gicp_error = std::numeric_limits<double>::max();
    int valid_count = 0;
    int source_point_count = 0;
    int target_point_count = 0;
    int iterations = 0;
    double crop_time_ms = 0.0;
    double init_time_ms = 0.0;
    double solve_time_ms = 0.0;
    float effective_voxel_leaf_size = 0.0f;
    bool xicp_triggered = false;
    bool final_quality = false;
};

enum AppState { EDIT_MODE, TEST_MODE };
AppState current_state = EDIT_MODE;

cv::Mat global_map_img_base; 
cv::Mat display_img;         

float min_x, max_x, min_y, max_y;
int global_img_w, global_img_h;
double current_display_scale = 1.0;

pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr obstacle_kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>());

// GUI 交互时用于维护的本地数据库库
vector<HistNode> history_db; 
bool sc_database_dirty = false;

// 【重构核心】高内聚定位模块，掌管一切！
relocation::location loc_module;

bool is_dragging = false;
cv::Point drag_start;
bool trigger_relocation = false;
bool headless_mode = false;
bool benchmark_mode = false;
bool ambiguous_benchmark_mode = false;
bool last_pipeline_success = false;
double last_pipeline_time_ms = 0.0;
double last_gicp_time_ms = 0.0;
double last_position_error_m = 0.0;
int last_gicp_source_points = 0;
int last_gicp_target_points = 0;
int last_gicp_iterations = 0;
float last_gicp_effective_voxel_leaf_size = 0.0f;
float test_gt_x = 0, test_gt_y = 0, test_gt_yaw = 0;

// ======================= 坐标转换与渲染 =======================
cv::Point world2pixel(float x, float y) {
    int u = test_padding + static_cast<int>((x - min_x) / test_resolution);
    int v = global_img_h - test_padding - static_cast<int>((y - min_y) / test_resolution);
    return cv::Point(u, v);
}

float pixel2world_x(int u) {
    return (u - test_padding) * test_resolution + min_x;
}

float pixel2world_y(int v) {
    return min_y + (global_img_h - test_padding - v) * test_resolution;
}

void update_display_scale() {
    if (global_img_w <= 0 || global_img_h <= 0) {
        current_display_scale = 1.0;
        return;
    }
    double scale_w = static_cast<double>(test_display_max_width) / static_cast<double>(global_img_w);
    double scale_h = static_cast<double>(test_display_max_height) / static_cast<double>(global_img_h);
    current_display_scale = std::min(1.0, std::min(scale_w, scale_h));
}

cv::Point display2pixel(int x, int y) {
    int u = static_cast<int>(std::lround(x / current_display_scale));
    int v = static_cast<int>(std::lround(y / current_display_scale));
    u = std::max(0, std::min(global_img_w - 1, u));
    v = std::max(0, std::min(global_img_h - 1, v));
    return cv::Point(u, v);
}

void show_relocation_window(const cv::Mat& img) {
    if (headless_mode) return;
    if (current_display_scale >= 0.999) {
        cv::imshow("Interactive Relocation", img);
        return;
    }

    cv::Mat scaled;
    cv::resize(img, scaled, cv::Size(), current_display_scale, current_display_scale, cv::INTER_AREA);
    cv::imshow("Interactive Relocation", scaled);
}

void draw_arrow(cv::Mat& img, float x, float y, float yaw_deg, cv::Scalar color, int length = test_arrow_length) {
    cv::Point center = world2pixel(x, y);
    float rad = yaw_deg * M_PI / 180.0f;
    cv::Point end_pt(center.x + length * cos(rad), center.y - length * sin(rad));
    cv::arrowedLine(img, center, end_pt, color, 3, 8, 0, 0.3);
    cv::circle(img, center, 6, color, -1);
}

bool HasEnoughScoreSeparation(double best, double second, double min_gap, double min_ratio) {
    double gap = second - best;
    double ratio = second / std::max(best, 1e-6);
    return gap >= min_gap || ratio >= min_ratio;
}

bool HasEnoughFusionDescriptorSeparation(
    const vector<RoughPoseCandidate>& candidates,
    double min_gap,
    double min_ratio) {
    if (candidates.empty() || !candidates.front().sc_matched) return false;

    const auto& best = candidates.front();
    double second_iris_score = numeric_limits<double>::max();
    double second_sc_score = numeric_limits<double>::max();
    for (size_t i = 1; i < candidates.size(); ++i) {
        second_iris_score = min(second_iris_score, candidates[i].iris_score);
        if (candidates[i].sc_available) {
            second_sc_score = min(second_sc_score, candidates[i].sc_score);
        }
    }

    const bool iris_unique = isfinite(second_iris_score) &&
                             HasEnoughScoreSeparation(
                                 best.iris_score, second_iris_score, min_gap, min_ratio);
    const bool sc_unique = isfinite(second_sc_score) &&
                           HasEnoughScoreSeparation(
                               best.sc_score, second_sc_score, min_gap, min_ratio);
    return iris_unique && sc_unique &&
           best.descriptor_yaw_diff_deg <= loc_fusion_yaw_consistency_limit_deg;
}

bool IsSameGicpSolution(
    const TestGicpCandidateResult& lhs,
    const TestGicpCandidateResult& rhs) {
    const double translation_difference =
        (lhs.T.head<2>() - rhs.T.head<2>()).norm();
    const double lhs_yaw = atan2(lhs.R(1, 0), lhs.R(0, 0));
    const double rhs_yaw = atan2(rhs.R(1, 0), rhs.R(0, 0));
    const double yaw_difference_deg =
        abs(remainder(lhs_yaw - rhs_yaw, 2.0 * M_PI)) * 180.0 / M_PI;
    return translation_difference <= test_same_solution_translation &&
           yaw_difference_deg <= test_same_solution_yaw_deg;
}

const TestGicpCandidateResult* FindCompetingGicpSolution(
    const vector<TestGicpCandidateResult>& results) {
    if (results.empty()) return nullptr;
    for (size_t i = 1; i < results.size(); ++i) {
        if (!IsSameGicpSolution(results.front(), results[i])) {
            return &results[i];
        }
    }
    return nullptr;
}

void redraw_edit_mode() {
    cv::Mat temp = global_map_img_base.clone();
    for (const auto& node : history_db) {
        cv::circle(temp, world2pixel(node.x, node.y), 3, cv::Scalar(255, 0, 0), -1);
    }
    cv::putText(temp, "EDIT MODE: L-Click DELETE | R-Click ADD (with collision check)", cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    cv::putText(temp, "Press 'S' to SAVE changes, build Location Module and test.", cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 200), 2);
    display_img = temp.clone();
    show_relocation_window(display_img);
}

void redraw_test_mode_base() {
    cv::Mat temp = global_map_img_base.clone();
    for (const auto& node : history_db) {
        cv::circle(temp, world2pixel(node.x, node.y), 3, cv::Scalar(255, 0, 0), -1);
    }
    cv::putText(temp, "TEST MODE: Left Click & Drag to set Pose.", cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
    display_img = temp.clone();
    show_relocation_window(display_img);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractLocalCloud(float cx, float cy, float yaw_deg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr local(new pcl::PointCloud<pcl::PointXYZ>);
    float yaw_rad = yaw_deg * M_PI / 180.0f;
    float cos_y = cos(yaw_rad);
    float sin_y = sin(yaw_rad);
    constexpr int yaw_bins = 3600; // 0.1度水平分辨率 (模拟高线束雷达)
    constexpr int z_bins = 50;     // Z轴切分为50层
    constexpr float thickness_tolerance = 0.2f; // 表面厚度容忍度(米)，保留墙皮表面

    std::vector<std::vector<float>> depth_buffer(yaw_bins, std::vector<float>(z_bins, test_max_distance));
    
    for (const auto& pt : global_map_cloud->points) {
        float dx = pt.x - cx;
        float dy = pt.y - cy;
        float dist = std::hypot(dx, dy);
        
        if (dist <= test_max_distance) {
            float angle = std::atan2(dy, dx);
            if (angle < 0) angle += 2 * M_PI;
            
            int y_idx = static_cast<int>((angle / (2 * M_PI)) * yaw_bins) % yaw_bins;
            float z_norm = (pt.z - test_z_min) / (test_z_max - test_z_min);
            int z_idx = std::max(0, std::min(z_bins - 1, static_cast<int>(z_norm * z_bins)));
            
            if (dist < depth_buffer[y_idx][z_idx]) {
                depth_buffer[y_idx][z_idx] = dist;
            }
        }
    }

    for (const auto& pt : global_map_cloud->points) {
        float dx = pt.x - cx;
        float dy = pt.y - cy;
        float dist = std::hypot(dx, dy);
        
        if (dist <= test_max_distance) {
            float angle = std::atan2(dy, dx);
            if (angle < 0) angle += 2 * M_PI;
            
            int y_idx = static_cast<int>((angle / (2 * M_PI)) * yaw_bins) % yaw_bins;
            float z_norm = (pt.z - test_z_min) / (test_z_max - test_z_min);
            int z_idx = std::max(0, std::min(z_bins - 1, static_cast<int>(z_norm * z_bins)));
            
            // 只有距离 <= “该方向最近距离 + 厚度容差”的点才被认为是表面！
            if (dist <= depth_buffer[y_idx][z_idx] + thickness_tolerance) {
                pcl::PointXYZ p_local;
                // 顺便进行坐标系转换，转到车体局部坐标系
                p_local.x = dx * cos_y + dy * sin_y;
                p_local.y = -dx * sin_y + dy * cos_y;
                p_local.z = pt.z;
                local->push_back(p_local);
            }
        }
    }
    return local;
}

void SaveDatabase(const string& filename, const vector<HistNode>& db) {
    ofstream out(filename);
    if (!out.is_open()) return;
    out << db.size() << "\n";
    for (const auto& node : db) {
        out << node.x << " " << node.y << " " << node.yaw << "\n";
        out << node.binary_vec.size() << "\n";
        for (auto b : node.binary_vec) out << (int)b << " ";
        out << "\n";
        auto save_mat = [&](const cv::Mat1b& mat) {
            out << mat.rows << " " << mat.cols << "\n";
            for (int r = 0; r < mat.rows; ++r) {
                for (int c = 0; c < mat.cols; ++c) { out << (int)mat(r, c) << " "; }
            }
            out << "\n";
        };
        save_mat(node.desc.img); save_mat(node.desc.T); save_mat(node.desc.M);
    }
    out.close();
}

bool LoadDatabase(const string& filename, vector<HistNode>& db) {
    ifstream in(filename);
    if (!in.is_open()) return false;
    size_t size; if (!(in >> size)) return false;
    db.resize(size);
    for (size_t i = 0; i < size; ++i) {
        in >> db[i].x >> db[i].y >> db[i].yaw;
        size_t vec_size; in >> vec_size;
        db[i].binary_vec.resize(vec_size);
        for (size_t j = 0; j < vec_size; ++j) { int val; in >> val; db[i].binary_vec[j] = val; }
        if (db[i].binary_vec.size() != iris_binary_vec_size) {
            in.close();
            db.clear();
            return false;
        }
        auto load_mat = [&](cv::Mat1b& mat) {
            int rows, cols; in >> rows >> cols; mat.create(rows, cols);
            for (int r = 0; r < rows; ++r) { for (int c = 0; c < cols; ++c) { int val; in >> val; mat(r, c) = val; } }
        };
        load_mat(db[i].desc.img); load_mat(db[i].desc.T); load_mat(db[i].desc.M);
    }
    in.close(); return true;
}

const char* ScanContextVariantName(ScanContextVariant variant) {
    switch (variant) {
        case ScanContextVariant::Original: return "original";
        case ScanContextVariant::PolarLeftShift: return "left";
        case ScanContextVariant::PolarRightShift: return "right";
        case ScanContextVariant::CartesianDoubleFlip: return "double_flip";
    }
    return "unknown";
}

bool EnsureScanContextDatabase(const string& iris_database_path, const vector<HistNode>& db) {
    if (db.empty()) return false;
    const string sc_database_path = MakeScanContextDatabasePath(iris_database_path);
    std::error_code time_error;
    const bool files_exist = std::filesystem::exists(iris_database_path) &&
                             std::filesystem::exists(sc_database_path);
    std::filesystem::file_time_type sc_write_time;
    std::filesystem::file_time_type iris_write_time;
    if (files_exist) {
        sc_write_time = std::filesystem::last_write_time(sc_database_path, time_error);
        if (!time_error) {
            iris_write_time = std::filesystem::last_write_time(iris_database_path, time_error);
        }
    }
    const bool sidecar_is_current = files_exist && !time_error &&
                                    sc_write_time >= iris_write_time;
    if (!sc_database_dirty && sidecar_is_current) {
        ScanContextPlusPlus existing(ScanContextConfig::IrisPolar());
        if (existing.LoadDatabase(sc_database_path) && existing.PlaceCount() == db.size()) {
            cout << "[SC++数据库] 使用已有文件: " << sc_database_path
                 << ", Places: " << existing.PlaceCount()
                 << ", Descriptors: " << existing.DescriptorCount() << endl;
            return true;
        }
    }

    const auto start = chrono::steady_clock::now();
    ScanContextPlusPlus scan_context(ScanContextConfig::IrisPolar());
    for (size_t i = 0; i < db.size(); ++i) {
        auto local_cloud = ExtractLocalCloud(db[i].x, db[i].y, db[i].yaw);
        if (!local_cloud || local_cloud->empty()) {
            cerr << "[SC++数据库] Hist " << i << " 无法生成局部点云。" << endl;
            return false;
        }
        try {
            scan_context.AddPlace(local_cloud, static_cast<int>(i));
        } catch (const exception& error) {
            cerr << "[SC++数据库] Hist " << i << " 描述子生成失败: "
                 << error.what() << endl;
            return false;
        }
    }
    scan_context.BuildIndex();
    if (!scan_context.SaveDatabase(sc_database_path)) {
        cerr << "[SC++数据库] 保存失败: " << sc_database_path << endl;
        return false;
    }
    const auto end = chrono::steady_clock::now();
    const double elapsed_ms = chrono::duration_cast<chrono::milliseconds>(end - start).count();
    sc_database_dirty = false;
    cout << "[SC++数据库] 已生成: " << sc_database_path
         << ", Places: " << scan_context.PlaceCount()
         << ", Descriptors: " << scan_context.DescriptorCount()
         << ", Time: " << elapsed_ms << "ms" << endl;
    return true;
}

void RunRelocationPipeline(float gt_x, float gt_y, float gt_yaw) {
    last_pipeline_success = false;
    last_pipeline_time_ms = 0.0;
    last_gicp_time_ms = 0.0;
    last_position_error_m = 0.0;
    last_gicp_source_points = 0;
    last_gicp_target_points = 0;
    last_gicp_iterations = 0;
    last_gicp_effective_voxel_leaf_size = 0.0f;
    cout << "\n=================================================" << endl;
    cout << "[GT 真实位姿] X: " << gt_x << ", Y: " << gt_y << ", Yaw: " << gt_yaw << "°" << endl;

    auto query_cloud = ExtractLocalCloud(gt_x, gt_y, gt_yaw);
    if (query_cloud->empty()) return;

    float prior_x = gt_x + test_prior_offset_x; 
    float prior_y = gt_y + test_prior_offset_y;
    double prior_yaw = gt_yaw * M_PI / 180.0;
    loc_module.GetPrePose(prior_x, prior_y, prior_yaw);

    auto t_coarse_start = chrono::steady_clock::now();
    vector<RoughPoseCandidate> rough_candidates;
    bool rough_ok = loc_module.GetRoughPoseCandidatesWithPrePose(query_cloud, prior_x, prior_y, prior_yaw, rough_candidates);
    
    auto t_coarse_end = chrono::steady_clock::now();
    double coarse_time = chrono::duration_cast<chrono::milliseconds>(t_coarse_end - t_coarse_start).count();

    if (!rough_ok) {
        cout << "[失败] 粗定位未能找到有效匹配（可能超出先验范围）。" << endl;
        return;
    }
    const bool fusion_active = rough_candidates.front().fusion_active;
    cout << "[Stage1: IRIS+SC++] Mode: "
         << (fusion_active ? "fusion" : "IRIS-only")
         << ", 候选数: " << rough_candidates.size()
         << ", Best X: " << rough_candidates.front().x
         << ", Y: " << rough_candidates.front().y
         << ", Yaw: " << rough_candidates.front().yaw_deg
         << "°, Score: " << rough_candidates.front().rough_score
         << " (" << coarse_time << "ms)" << endl;
    if (!benchmark_mode) {
        cout << fixed << setprecision(4);
        cout << "  Rank Hist Src     X       Y    Fused    IRIS      SC"
                "   Yaw-I   Yaw-S   YawDiff  Shift Variant" << endl;
        for (size_t i = 0; i < rough_candidates.size(); ++i) {
            const auto& candidate = rough_candidates[i];
            const string source = candidate.iris_retrieved && candidate.sc_retrieved
                                      ? "I+S"
                                      : candidate.sc_retrieved ? "SC" : "IRIS";
            cout << "  " << setw(4) << i
                 << " " << setw(4) << candidate.hist_index
                 << " " << setw(4) << source
                 << " " << setw(7) << candidate.x
                 << " " << setw(7) << candidate.y
                 << " " << setw(8) << candidate.rough_score
                 << " " << setw(8) << candidate.iris_score
                 << " " << setw(8) << candidate.sc_score
                 << " " << setw(7) << candidate.iris_yaw_deg
                 << " " << setw(7) << candidate.sc_yaw_deg
                 << " " << setw(8) << candidate.descriptor_yaw_diff_deg
                 << " " << setw(6) << candidate.sc_lateral_shift
                 << " " << ScanContextVariantName(candidate.sc_variant) << endl;
        }
        cout << defaultfloat;
    }

    auto t_fine_start = chrono::steady_clock::now();
    vector<TestGicpCandidateResult> gicp_results;
    gicp_results.reserve(rough_candidates.size());
    double best_rough_score = rough_candidates.front().rough_score;
    size_t attempted_gicp_count = 0;
    const bool fusion_descriptor_unique = fusion_active &&
        HasEnoughFusionDescriptorSeparation(
            rough_candidates,
            test_min_rough_score_gap,
            test_min_rough_score_ratio);

    for (const auto& candidate : rough_candidates) {
        if (attempted_gicp_count >= test_max_gicp_candidates) break;
        double rough_ratio = candidate.rough_score / std::max(best_rough_score, 1e-6);
        if (!fusion_active && attempted_gicp_count > 0 &&
            rough_ratio > test_max_rough_score_ratio_for_gicp) {
            break;
        }

        const bool use_final_quality =
            fusion_descriptor_unique && attempted_gicp_count == 0;
        attempted_gicp_count++;
        Eigen::Matrix3d R_gicp = Eigen::AngleAxisd(candidate.yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Vector3d T_gicp(candidate.x, candidate.y, 0.0);
        double gicp_error = numeric_limits<double>::max();
        int valid_count = 0;

        const bool gicp_ok = use_final_quality
                                 ? loc_module.SetPrecisePose(
                                       query_cloud, R_gicp, T_gicp,
                                       gicp_error, valid_count)
                                 : loc_module.SetPrecisePose(
                                       query_cloud, R_gicp, T_gicp,
                                       gicp_error, valid_count,
                                       test_candidate_gicp_max_iterations,
                                       test_candidate_gicp_voxel_leaf_size);
        if (!gicp_ok) {
            continue;
        }

        TestGicpCandidateResult result;
        result.rough = candidate;
        result.R = R_gicp;
        result.T = T_gicp;
        result.gicp_error = gicp_error;
        result.valid_count = valid_count;
        result.source_point_count = loc_module.GetLastSourcePointCount();
        result.target_point_count = loc_module.GetLastTargetPointCount();
        result.iterations = loc_module.GetLastGicpIterations();
        result.crop_time_ms = loc_module.GetLastGicpCropTimeMs();
        result.init_time_ms = loc_module.GetLastGicpInitTimeMs();
        result.solve_time_ms = loc_module.GetLastGicpSolveTimeMs();
        result.effective_voxel_leaf_size =
            loc_module.GetLastGicpEffectiveVoxelLeafSize();
        result.xicp_triggered = loc_module.WasLastXicpTriggered();
        result.final_quality = use_final_quality;
        gicp_results.push_back(result);

        double valid_ratio = result.source_point_count <= 0
                                 ? 0.0
                                 : static_cast<double>(valid_count) / static_cast<double>(result.source_point_count);
        bool enough_inliers = valid_count >= test_min_gicp_valid_count &&
                              valid_ratio >= test_min_gicp_valid_ratio;
        bool rough_unique_now = fusion_active
                                    ? attempted_gicp_count == 1 &&
                                          fusion_descriptor_unique
                                    : rough_candidates.size() < 2 ||
                                          HasEnoughScoreSeparation(
                                              rough_candidates[0].rough_score,
                                              rough_candidates[1].rough_score,
                                              test_min_rough_score_gap,
                                              test_min_rough_score_ratio);
        if (rough_unique_now && enough_inliers && gicp_error <= test_early_accept_gicp_error) {
            break;
        }
    }
    
    auto t_fine_end = chrono::steady_clock::now();
    double fine_time = chrono::duration_cast<chrono::milliseconds>(t_fine_end - t_fine_start).count();

    cv::Mat temp = display_img.clone();
    for (const auto& candidate : rough_candidates) {
        cv::Scalar color(0, 165, 255);
        if (candidate.iris_retrieved && candidate.sc_retrieved) {
            color = cv::Scalar(255, 0, 255);
        } else if (candidate.sc_retrieved) {
            color = cv::Scalar(255, 255, 0);
        }
        draw_arrow(temp, candidate.x, candidate.y, candidate.yaw_deg,
                   color, test_arrow_length / 2);
    }
    draw_arrow(temp, gt_x, gt_y, gt_yaw, cv::Scalar(0, 0, 255));
    cv::putText(temp,
                "Red: GT | Orange: IRIS | Cyan: SC++ | Magenta: both",
                cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(50, 50, 50), 2);

    if (gicp_results.empty()) {
        cout << "[失败] 粗定位候选数: " << rough_candidates.size()
             << ", GicpTried: " << attempted_gicp_count
             << "，但全部 GICP 精定位失败。" << endl;
        cv::putText(temp, "Rejected: all GICP candidates failed", cv::Point(20, 70), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 200), 2);
        show_relocation_window(temp);
        return;
    }

    sort(gicp_results.begin(), gicp_results.end(), [](const auto& lhs, const auto& rhs) {
        if (abs(lhs.gicp_error - rhs.gicp_error) > 1e-4) return lhs.gicp_error < rhs.gicp_error;
        if (lhs.valid_count != rhs.valid_count) return lhs.valid_count > rhs.valid_count;
        return lhs.rough.rough_score < rhs.rough.rough_score;
    });

    const auto& best = gicp_results.front();
    const bool gicp_best_is_rough_best =
        best.rough.hist_index == rough_candidates.front().hist_index;
    bool rough_unique = gicp_best_is_rough_best &&
                        (fusion_active
                             ? fusion_descriptor_unique
                             : rough_candidates.size() < 2 ||
                                   HasEnoughScoreSeparation(
                                       rough_candidates[0].rough_score,
                                       rough_candidates[1].rough_score,
                                       test_min_rough_score_gap,
                                       test_min_rough_score_ratio));
    const TestGicpCandidateResult* competing_solution =
        FindCompetingGicpSolution(gicp_results);
    bool gicp_unique = competing_solution == nullptr ||
                       HasEnoughScoreSeparation(
                           best.gicp_error,
                           competing_solution->gicp_error,
                           test_min_gicp_error_gap,
                           test_min_gicp_error_ratio);
    double valid_ratio = best.source_point_count <= 0
                             ? 0.0
                             : static_cast<double>(best.valid_count) / static_cast<double>(best.source_point_count);
    bool enough_inliers = best.valid_count >= test_min_gicp_valid_count &&
                          valid_ratio >= test_min_gicp_valid_ratio;

    cout << "[Stage2: GICP] GicpTried: " << attempted_gicp_count
         << ", GicpOK: " << gicp_results.size()
         << ", BestHist: " << best.rough.hist_index
         << ", Error: " << best.gicp_error
         << ", Valid: " << best.valid_count
         << ", Source: " << best.source_point_count
         << ", Ratio: " << valid_ratio
         << ", RoughUnique: " << rough_unique
         << ", GicpUnique: " << gicp_unique
         << ", CoarseXICP: " << best.xicp_triggered
         << " (" << fine_time << "ms)" << endl;
    cout << "[GICP内部] Crop: " << best.crop_time_ms
         << "ms, Init: " << best.init_time_ms
         << "ms, Solve: " << best.solve_time_ms
         << "ms, Source: " << best.source_point_count
         << ", Target: " << best.target_point_count
         << ", Iter: " << best.iterations
         << ", Leaf: " << best.effective_voxel_leaf_size << "m" << endl;

    if (!enough_inliers) {
        cout << "[拒绝] GICP 有效匹配不足。" << endl;
        cv::putText(temp, "Rejected: insufficient GICP inliers", cv::Point(20, 70), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 200), 2);
        show_relocation_window(temp);
        return;
    }

    if (!rough_unique && !gicp_unique) {
        double rough_gap = rough_candidates[1].rough_score - rough_candidates[0].rough_score;
        double gicp_gap = competing_solution
                              ? competing_solution->gicp_error - best.gicp_error
                              : 0.0;
        cout << "[拒绝] 粗匹配和 GICP 均低置信。RoughGap: " << rough_gap << ", GicpGap: " << gicp_gap << endl;
        cv::putText(temp, "Rejected: ambiguous rough and GICP candidates", cv::Point(20, 70), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 200), 2);
        show_relocation_window(temp);
        return;
    }

    Eigen::Matrix3d final_R = best.R;
    Eigen::Vector3d final_T = best.T;
    double final_gicp_error = best.gicp_error;
    int final_valid_count = best.valid_count;
    bool final_xicp_triggered = best.xicp_triggered;
    bool final_gicp_ok = true;
    double final_gicp_time = 0.0;
    int final_source_point_count = best.source_point_count;
    int final_target_point_count = best.target_point_count;
    int final_iterations = best.iterations;
    float final_effective_voxel_leaf_size = best.effective_voxel_leaf_size;
    auto t_final_gicp_end = t_fine_end;
    if (!best.final_quality) {
        const auto t_final_gicp_start = chrono::steady_clock::now();
        final_gicp_ok = loc_module.SetPrecisePose(
            query_cloud, final_R, final_T, final_gicp_error, final_valid_count);
        t_final_gicp_end = chrono::steady_clock::now();
        final_gicp_time = chrono::duration<double, milli>(
            t_final_gicp_end - t_final_gicp_start).count();
        final_xicp_triggered = loc_module.WasLastXicpTriggered();
        final_source_point_count = loc_module.GetLastSourcePointCount();
        final_target_point_count = loc_module.GetLastTargetPointCount();
        final_iterations = loc_module.GetLastGicpIterations();
        final_effective_voxel_leaf_size =
            loc_module.GetLastGicpEffectiveVoxelLeafSize();
    }
    if (!final_gicp_ok) {
        cout << "[拒绝] 最佳候选通过粗 GICP 筛选，但最终精 GICP 失败。" << endl;
        cv::putText(temp, "Rejected: final GICP failed", cv::Point(20, 70), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 200), 2);
        show_relocation_window(temp);
        return;
    }

    const double relocation_time = chrono::duration<double, milli>(
        t_final_gicp_end - t_coarse_start).count();

    float fine_x = final_T.x();
    float fine_y = final_T.y();
    float fine_yaw = atan2(final_R(1, 0), final_R(0, 0)) * 180.0 / M_PI;
    if (fine_yaw < 0) fine_yaw += 360.0f;

    cout << "[接受] X: " << fine_x << ", Y: " << fine_y << ", Yaw: " << fine_yaw
         << "°, FinalGICP: " << final_gicp_error
         << ", FinalValid: " << final_valid_count
         << ", CoarseXICP: " << best.xicp_triggered
         << ", FinalXICP: " << final_xicp_triggered << endl;
    cout << "[耗时] Rough: " << coarse_time
         << "ms, CandidateGICP: " << fine_time
         << "ms, FinalGICP: " << final_gicp_time
         << "ms, Total: " << relocation_time << "ms" << endl;
    const double position_error = std::hypot(fine_x - gt_x, fine_y - gt_y);
    cout << "[误差分析] 距离误差: " << position_error << "m" << endl;

    last_pipeline_time_ms = relocation_time;
    last_gicp_time_ms = fine_time + final_gicp_time;
    last_position_error_m = position_error;
    last_gicp_source_points = final_source_point_count;
    last_gicp_target_points = final_target_point_count;
    last_gicp_iterations = final_iterations;
    last_gicp_effective_voxel_leaf_size = final_effective_voxel_leaf_size;

    draw_arrow(temp, fine_x, fine_y, fine_yaw, cv::Scalar(0, 200, 0));         
    
    string info1 = "Red: GT | Orange: IRIS | Cyan: SC++ | Magenta: both | Green: GICP";
    string info2 = "Fusion: " + to_string((int)coarse_time) + "ms, GICP: " + to_string((int)(fine_time + final_gicp_time)) + "ms, GicpTried: " + to_string((int)attempted_gicp_count) + ", GicpOK: " + to_string((int)gicp_results.size());
    string info3 = string("XICP: coarse ") + (best.xicp_triggered ? "ON" : "OFF") + ", final " + (final_xicp_triggered ? "ON" : "OFF");
    cv::putText(temp, info1, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(50, 50, 50), 2);
    cv::putText(temp, info2, cv::Point(20, 70), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(50, 50, 50), 2);
    cv::putText(temp, info3, cv::Point(20, 110), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                final_xicp_triggered || best.xicp_triggered ? cv::Scalar(0, 0, 200) : cv::Scalar(50, 50, 50), 2);
    last_pipeline_success = true;
    show_relocation_window(temp);
}

void onMouse(int event, int x, int y, int /*flags*/, void* /*userdata*/) {
    cv::Point pixel = display2pixel(x, y);
    x = pixel.x;
    y = pixel.y;

    if (current_state == EDIT_MODE) {
        if (event == cv::EVENT_LBUTTONDOWN) {
            float wx = pixel2world_x(x);
            float wy = pixel2world_y(y);
            float min_dist = test_edit_delete_radius; 
            int best_idx = -1;
            for (size_t i = 0; i < history_db.size(); ++i) {
                float dist = std::hypot(history_db[i].x - wx, history_db[i].y - wy);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_idx = i;
                }
            }
            if (best_idx != -1) {
                history_db.erase(history_db.begin() + best_idx);
                sc_database_dirty = true;
                redraw_edit_mode(); 
            }
        } 
        else if (event == cv::EVENT_RBUTTONDOWN) {
            float wx = pixel2world_x(x);
            float wy = pixel2world_y(y);
            float min_dist = numeric_limits<float>::max();
            for (const auto& node : history_db) {
                float dist = std::hypot(node.x - wx, node.y - wy);
                if (dist < min_dist) min_dist = dist;
            }

            if (min_dist < test_edit_add_min_dist) return;

            pcl::PointXYZ searchPoint(wx, wy, 1.0f);
            vector<int> pointIdx;
            vector<float> pointDist;
            
            if (obstacle_kdtree->radiusSearch(searchPoint, test_collision_radius, pointIdx, pointDist) > test_collision_max_pts) return;
            if (obstacle_kdtree->radiusSearch(searchPoint, test_empty_radius, pointIdx, pointDist) < test_empty_min_pts) return;

            auto local_cloud = ExtractLocalCloud(wx, wy, 0.0f);
            if (local_cloud->empty()) return;

            cv::Mat1b iris_img = iris::GetIris(*local_cloud);
            HistNode new_node;
            new_node.x = wx;
            new_node.y = wy;
            new_node.yaw = 0.0f;
            new_node.desc = iris::GetFeature(iris_img);
            new_node.binary_vec = iris::IrisToBinaryVec(iris_img);

            history_db.push_back(new_node);
            sc_database_dirty = true;
            redraw_edit_mode();
        }

    } else if (current_state == TEST_MODE) {
        if (event == cv::EVENT_LBUTTONDOWN) {
            is_dragging = true;
            drag_start = cv::Point(x, y);
        } else if (event == cv::EVENT_MOUSEMOVE && is_dragging) {
            cv::Mat temp = display_img.clone();
            cv::arrowedLine(temp, drag_start, cv::Point(x, y), cv::Scalar(0, 0, 255), 2, 8, 0, 0.2);
            show_relocation_window(temp);
        } else if (event == cv::EVENT_LBUTTONUP) {
            is_dragging = false;
            cv::Point drag_end(x, y);
            test_gt_x = pixel2world_x(drag_start.x);
            test_gt_y = pixel2world_y(drag_start.y);

            float dx = drag_end.x - drag_start.x;
            float dy = -(drag_end.y - drag_start.y); 
            test_gt_yaw = atan2(dy, dx) * 180.0 / M_PI;
            if (test_gt_yaw < 0) test_gt_yaw += 360.0;

            if (sqrt(dx*dx + dy*dy) < test_drag_min_dist) test_gt_yaw = 0.0f; 

            trigger_relocation = true;
        }
    }
}

int main(int argc, char** argv) {
    const string run_mode = argc > 1 ? string(argv[1]) : string();
    ambiguous_benchmark_mode = run_mode == "--headless-ambiguous-benchmark";
    benchmark_mode = run_mode == "--headless-benchmark" ||
                     ambiguous_benchmark_mode;
    headless_mode = run_mode == "--headless-smoke" || benchmark_mode;
    string pcd_path = "relocation/PCD/1.pcd";
    string db_path = "history_db.txt";

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *global_map_cloud) == -1) return -1;
    
    pcl::PointXYZ pmin, pmax;
    pcl::getMinMax3D(*global_map_cloud, pmin, pmax);
    min_x = pmin.x; max_x = pmax.x; min_y = pmin.y; max_y = pmax.y;

    global_img_w = std::ceil((max_x - min_x) / test_resolution) + test_padding * 2;
    global_img_h = std::ceil((max_y - min_y) / test_resolution) + test_padding * 2;
    update_display_scale();
    cout << "[可视化] 原始地图图像: " << global_img_w << "x" << global_img_h
         << ", 显示缩放: " << current_display_scale
         << ", 窗口约: " << static_cast<int>(global_img_w * current_display_scale)
         << "x" << static_cast<int>(global_img_h * current_display_scale) << endl;
    global_map_img_base = cv::Mat(global_img_h, global_img_w, CV_8UC3, cv::Scalar(255, 255, 255));

    for (const auto& pt : global_map_cloud->points) {
        if (pt.z >= test_z_min && pt.z <= test_z_max) {
            obstacle_cloud->push_back(pt); 
            cv::Point p = world2pixel(pt.x, pt.y);
            if (p.x >= 0 && p.x < global_img_w && p.y >= 0 && p.y < global_img_h) {
                global_map_img_base.at<cv::Vec3b>(p) = cv::Vec3b(180, 180, 180);
            }
        }
    }
    
    obstacle_kdtree->setInputCloud(obstacle_cloud);

    if (!LoadDatabase(db_path, history_db)) {
        cout << "未发现 history_db.txt，开始从 PCD 自动进行网格采样生成..." << endl;
        int sampled_count = 0;
        for (float x = min_x; x <= max_x; x += test_grid_step) {
            for (float y = min_y; y <= max_y; y += test_grid_step) {
                pcl::PointXYZ searchPoint(x, y, 1.0f);
                vector<int> pointIdx;
                vector<float> pointDist;
                
                if (obstacle_kdtree->radiusSearch(searchPoint, test_collision_radius, pointIdx, pointDist) > test_collision_max_pts) continue;
                if (obstacle_kdtree->radiusSearch(searchPoint, test_empty_radius, pointIdx, pointDist) < test_empty_min_pts) continue;

                int sector_counts[8] = {0};
                for (size_t i = 0; i < pointIdx.size(); ++i) {
                    const auto& pt = obstacle_cloud->points[pointIdx[i]];
                    float angle = std::atan2(pt.y - y, pt.x - x); 
                    if (angle < 0) angle += 2 * M_PI; 
                    sector_counts[static_cast<int>(angle / (M_PI / 4.0)) % 8]++;
                }
                
                int max_contiguous_empty = 0, current_empty = 0;
                for (int i = 0; i < 16; ++i) { 
                    if (sector_counts[i % 8] < test_sector_empty_threshold) {
                        current_empty++;
                        if (current_empty > max_contiguous_empty) max_contiguous_empty = current_empty;
                    } else {
                        current_empty = 0;
                    }
                }
                if (max_contiguous_empty >= test_sector_max_empty) continue; 

                auto local_cloud = ExtractLocalCloud(x, y, 0.0f);
                if (local_cloud->empty()) continue;

                cv::Mat1b iris_img = iris::GetIris(*local_cloud);
                HistNode node;
                node.x = x; node.y = y; node.yaw = 0.0f;
                node.desc = iris::GetFeature(iris_img);
                node.binary_vec = iris::IrisToBinaryVec(iris_img);
                
                history_db.push_back(node);
                sc_database_dirty = true;
                sampled_count++;
            }
        }
        SaveDatabase(db_path, history_db);
    }

    if (!EnsureScanContextDatabase(db_path, history_db)) {
        cerr << "[错误] 启动完整测试前无法准备 SC++ 数据库。" << endl;
        return -1;
    }

    if (headless_mode) {
        if (!loc_module.Init(db_path, global_map_cloud)) {
            cerr << "[Headless] Location 模块初始化失败。" << endl;
            return -1;
        }
        display_img = global_map_img_base.clone();
        if (benchmark_mode) {
            size_t success_count = 0;
            size_t accurate_count = 0;
            double total_position_error = 0.0;
            double max_pipeline_time = 0.0;
            double max_gicp_time = 0.0;
            double max_position_error = 0.0;
            size_t max_pipeline_index = 0;
            size_t max_gicp_index = 0;
            size_t max_error_index = 0;
            const size_t benchmark_count = ambiguous_benchmark_mode
                                               ? history_db.size() - 1
                                               : history_db.size();
            for (size_t i = 0; i < benchmark_count; ++i) {
                const auto& sample = history_db[i];
                float benchmark_x = sample.x;
                float benchmark_y = sample.y;
                float benchmark_base_yaw = sample.yaw;
                if (ambiguous_benchmark_mode) {
                    const auto& next = history_db[i + 1];
                    benchmark_x = 0.5f * (sample.x + next.x);
                    benchmark_y = 0.5f * (sample.y + next.y);
                    benchmark_base_yaw = static_cast<float>(
                        sample.yaw + 0.5 * std::remainder(
                                               static_cast<double>(next.yaw - sample.yaw),
                                               360.0));
                }
                const float benchmark_yaw = std::fmod(
                    benchmark_base_yaw + 405.0f, 360.0f);
                RunRelocationPipeline(benchmark_x, benchmark_y, benchmark_yaw);
                cout << "[Benchmark] Hist:" << i
                     << " Success:" << last_pipeline_success
                     << " Total:" << last_pipeline_time_ms
                     << "ms GICP:" << last_gicp_time_ms
                     << "ms Error:" << last_position_error_m
                     << "m"
                     << " Source:" << last_gicp_source_points
                     << " Target:" << last_gicp_target_points
                     << " Iter:" << last_gicp_iterations
                     << " Leaf:" << last_gicp_effective_voxel_leaf_size
                     << "m" << endl;
                if (!last_pipeline_success) continue;
                success_count++;
                total_position_error += last_position_error_m;
                if (last_position_error_m <= 0.5) {
                    accurate_count++;
                }
                if (last_pipeline_time_ms > max_pipeline_time) {
                    max_pipeline_time = last_pipeline_time_ms;
                    max_pipeline_index = i;
                }
                if (last_gicp_time_ms > max_gicp_time) {
                    max_gicp_time = last_gicp_time_ms;
                    max_gicp_index = i;
                }
                if (last_position_error_m > max_position_error) {
                    max_position_error = last_position_error_m;
                    max_error_index = i;
                }
            }
            cout << "[BenchmarkSummary] Success:" << success_count
                 << "/" << benchmark_count
                 << " Accurate:" << accurate_count
                 << "/" << benchmark_count
                 << " MeanError:"
                 << (success_count == 0
                         ? 0.0
                         : total_position_error / static_cast<double>(success_count))
                 << "m"
                 << " Mode:"
                 << (ambiguous_benchmark_mode ? "midpoint" : "history")
                 << " MaxTotal:" << max_pipeline_time
                 << "ms@Hist" << max_pipeline_index
                 << " MaxGICP:" << max_gicp_time
                 << "ms@Hist" << max_gicp_index
                 << " MaxError:" << max_position_error
                 << "m@Hist" << max_error_index << endl;
            return success_count == benchmark_count ? 0 : 1;
        }
        const auto& sample = history_db.front();
        const float smoke_yaw = std::fmod(sample.yaw + 45.0f, 360.0f);
        cout << "[Headless] 使用 Hist 0 位置和 +45deg 航向执行完整融合与 GICP 测试。" << endl;
        RunRelocationPipeline(sample.x, sample.y, smoke_yaw);
        return last_pipeline_success ? 0 : 1;
    }

    cv::namedWindow("Interactive Relocation", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Interactive Relocation", onMouse, nullptr);

    current_state = EDIT_MODE;
    redraw_edit_mode();

    while (true) {
        char key = (char)cv::waitKey(30);
        
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }

        if (current_state == EDIT_MODE && (key == 's' || key == 'S')) {
            if (sc_database_dirty) {
                SaveDatabase(db_path, history_db);
            }
            if (!EnsureScanContextDatabase(db_path, history_db)) {
                cout << "[错误] SC++ 数据库生成失败，保留在编辑模式。" << endl;
                continue;
            }

            if (loc_module.Init(db_path, global_map_cloud)) {
                current_state = TEST_MODE;
                redraw_test_mode_base();
            } else {
                cout << "[错误] Location 模块初始化失败！" << endl;
            }
        }

        if (current_state == TEST_MODE && trigger_relocation) {
            trigger_relocation = false;
            redraw_test_mode_base(); 
            try {
                RunRelocationPipeline(test_gt_x, test_gt_y, test_gt_yaw);
            } catch (const cv::Exception& e) {
            } catch (const std::exception& e) {
            }
        }
    }

    return 0;
}
