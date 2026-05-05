#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <exception>

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

constexpr float test_resolution = 0.05f;   
constexpr int test_padding = 150;
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

enum AppState { EDIT_MODE, TEST_MODE };
AppState current_state = EDIT_MODE;

cv::Mat global_map_img_base; 
cv::Mat display_img;         

float min_x, max_x, min_y, max_y;
int global_img_w, global_img_h;

pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr obstacle_kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>());

// GUI 交互时用于维护的本地数据库库
vector<HistNode> history_db; 

// 【重构核心】高内聚定位模块，掌管一切！
relocation::location loc_module;

bool is_dragging = false;
cv::Point drag_start;
bool trigger_relocation = false;
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

void draw_arrow(cv::Mat& img, float x, float y, float yaw_deg, cv::Scalar color, int length = test_arrow_length) {
    cv::Point center = world2pixel(x, y);
    float rad = yaw_deg * M_PI / 180.0f;
    cv::Point end_pt(center.x + length * cos(rad), center.y - length * sin(rad));
    cv::arrowedLine(img, center, end_pt, color, 3, 8, 0, 0.3);
    cv::circle(img, center, 6, color, -1);
}

void redraw_edit_mode() {
    cv::Mat temp = global_map_img_base.clone();
    for (const auto& node : history_db) {
        cv::circle(temp, world2pixel(node.x, node.y), 3, cv::Scalar(255, 0, 0), -1);
    }
    cv::putText(temp, "EDIT MODE: L-Click DELETE | R-Click ADD (with collision check)", cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    cv::putText(temp, "Press 'S' to SAVE changes, build Location Module and test.", cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 200), 2);
    display_img = temp.clone();
    cv::imshow("Interactive Relocation", display_img);
}

void redraw_test_mode_base() {
    cv::Mat temp = global_map_img_base.clone();
    for (const auto& node : history_db) {
        cv::circle(temp, world2pixel(node.x, node.y), 3, cv::Scalar(255, 0, 0), -1);
    }
    cv::putText(temp, "TEST MODE: Left Click & Drag to set Pose.", cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
    display_img = temp.clone();
    cv::imshow("Interactive Relocation", display_img);
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
        auto load_mat = [&](cv::Mat1b& mat) {
            int rows, cols; in >> rows >> cols; mat.create(rows, cols);
            for (int r = 0; r < rows; ++r) { for (int c = 0; c < cols; ++c) { int val; in >> val; mat(r, c) = val; } }
        };
        load_mat(db[i].desc.img); load_mat(db[i].desc.T); load_mat(db[i].desc.M);
    }
    in.close(); return true;
}

void RunRelocationPipeline(float gt_x, float gt_y, float gt_yaw) {
    cout << "\n=================================================" << endl;
    cout << "[GT 真实位姿] X: " << gt_x << ", Y: " << gt_y << ", Yaw: " << gt_yaw << "°" << endl;

    auto query_cloud = ExtractLocalCloud(gt_x, gt_y, gt_yaw);
    if (query_cloud->empty()) return;

    float prior_x = gt_x + test_prior_offset_x; 
    float prior_y = gt_y + test_prior_offset_y;
    loc_module.GetPrePose(prior_x, prior_y, 0.0);

    double coarse_x, coarse_y, coarse_yaw;
    auto t_coarse_start = chrono::steady_clock::now();
    
    bool rough_ok = loc_module.SetRoughPoseWithPrePose(query_cloud, prior_x, prior_y, 0.0, coarse_x, coarse_y, coarse_yaw);
    
    auto t_coarse_end = chrono::steady_clock::now();
    double coarse_time = chrono::duration_cast<chrono::milliseconds>(t_coarse_end - t_coarse_start).count();

    if (!rough_ok) {
        cout << "[失败] 粗定位未能找到有效匹配（可能超出先验范围）。" << endl;
        return;
    }
    cout << "[Stage1: Iris] X: " << coarse_x << ", Y: " << coarse_y << ", Yaw: " << coarse_yaw << "° (" << coarse_time << "ms)" << endl;

    auto t_fine_start = chrono::steady_clock::now();
    
    Eigen::Matrix3d R_icp;
    R_icp = Eigen::AngleAxisd(coarse_yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d T_icp(coarse_x, coarse_y, 0.0);

    bool precise_ok = loc_module.SetPrecisePose(query_cloud, R_icp, T_icp);
    
    auto t_fine_end = chrono::steady_clock::now();
    double fine_time = chrono::duration_cast<chrono::milliseconds>(t_fine_end - t_fine_start).count();

    float fine_x = coarse_x, fine_y = coarse_y, fine_yaw = coarse_yaw;
    if (precise_ok) {
        fine_x = T_icp.x();
        fine_y = T_icp.y();
        fine_yaw = atan2(R_icp(1, 0), R_icp(0, 0)) * 180.0 / M_PI;
        if (fine_yaw < 0) fine_yaw += 360.0f;
    }

    cout << "[Stage2: ICP ] X: " << fine_x << ", Y: " << fine_y << ", Yaw: " << fine_yaw << "° (" << fine_time << "ms)" << endl;
    cout << "[误差分析] 距离误差: " << sqrt(pow(fine_x - gt_x, 2) + pow(fine_y - gt_y, 2)) << "m" << endl;

    cv::Mat temp = display_img.clone();
    draw_arrow(temp, coarse_x, coarse_y, coarse_yaw, cv::Scalar(0, 165, 255)); 
    draw_arrow(temp, fine_x, fine_y, fine_yaw, cv::Scalar(0, 200, 0));         
    draw_arrow(temp, gt_x, gt_y, gt_yaw, cv::Scalar(0, 0, 255));               
    
    string info1 = "Red: GT | Orange: Iris | Green: ICP";
    string info2 = "Time -> Iris: " + to_string((int)coarse_time) + "ms, ICP: " + to_string((int)fine_time) + "ms";
    cv::putText(temp, info1, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(50, 50, 50), 2);
    cv::putText(temp, info2, cv::Point(20, 70), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(50, 50, 50), 2);
    cv::imshow("Interactive Relocation", temp);
}

void onMouse(int event, int x, int y, int /*flags*/, void* /*userdata*/) {
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
            redraw_edit_mode();
        }

    } else if (current_state == TEST_MODE) {
        if (event == cv::EVENT_LBUTTONDOWN) {
            is_dragging = true;
            drag_start = cv::Point(x, y);
        } else if (event == cv::EVENT_MOUSEMOVE && is_dragging) {
            cv::Mat temp = display_img.clone();
            cv::arrowedLine(temp, drag_start, cv::Point(x, y), cv::Scalar(0, 0, 255), 2, 8, 0, 0.2);
            cv::imshow("Interactive Relocation", temp);
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

int main() {
    string pcd_path = "/home/hyl/new_nav/relocation/PCD/1.pcd";
    string db_path = "history_db.txt";

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *global_map_cloud) == -1) return -1;
    
    pcl::PointXYZ pmin, pmax;
    pcl::getMinMax3D(*global_map_cloud, pmin, pmax);
    min_x = pmin.x; max_x = pmax.x; min_y = pmin.y; max_y = pmax.y;

    global_img_w = std::ceil((max_x - min_x) / test_resolution) + test_padding * 2;
    global_img_h = std::ceil((max_y - min_y) / test_resolution) + test_padding * 2;
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
                sampled_count++;
            }
        }
        SaveDatabase(db_path, history_db);
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
            SaveDatabase(db_path, history_db);
            
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