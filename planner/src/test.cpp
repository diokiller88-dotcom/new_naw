#include "planner_2d/replanner.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>
#include <cmath>
#include <memory>
#include <string>
#include <chrono>
#include <algorithm>
#include <spdlog/spdlog.h>

using namespace planner_2d;

// ==============================================================================
// 内嵌的 ESDF Map 算法
// ==============================================================================
namespace esdf_map {
class esdf_map {
public:
    esdf_map() : m_MapLenX(0), m_MapWeightY(0), m_IsSetESDF(false) {}

    bool Init(const std::vector<int>& map_, int x_, int y_) {
        if (map_.empty() || x_ < 1 || y_ < 1 || x_ * y_ != static_cast<int>(map_.size())) return false;
        m_GripMap = map_; m_MapLenX = x_; m_MapWeightY = y_;
        return true;
    }

    void SetSurfMap() {
        m_SurfMap = m_GripMap;
        m_ESDFMap.assign(m_GripMap.size(), 1e9); 
        for (int i = 1; i < m_MapWeightY - 1; i++) {
            for (int j = 1; j < m_MapLenX - 1; j++) {
                int idx = i * m_MapLenX + j;
                if (m_GripMap[idx] == 1) {
                    m_ESDFMap[idx] = 0.0;
                    if (m_GripMap[idx+1] == 1 && m_GripMap[idx-1] == 1 &&
                        m_GripMap[idx+m_MapLenX] == 1 && m_GripMap[idx-m_MapLenX] == 1) {
                        m_SurfMap[idx] = -1;
                    }
                }
            }
        }
    }

    void ComputeEDT() {
        const double INF = 1e9;
        const int W = m_MapLenX; const int H = m_MapWeightY;
        std::vector<double> g(W * H);
        std::vector<int> v(std::max(W, H));
        std::vector<double> z(std::max(W, H) + 1);
        
        for (int y = 0; y < H; ++y) {
            int base = y * W; int k = 0; v[0] = 0; z[0] = -INF; z[1] = INF;
            for (int q = 1; q < W; ++q) {
                auto get_s = [&]() { return ((m_ESDFMap[base+q] + q*q) - (m_ESDFMap[base+v[k]] + v[k]*v[k])) / (2.0*(q-v[k])); };
                double s = get_s();
                while (s <= z[k]) { --k; s = get_s(); }
                v[++k] = q; z[k] = s; z[k+1] = INF;
            }
            for (int q = 0, ki = 0; q < W; ++q) {
                while (z[ki+1] < q) ++ki;
                g[base+q] = std::pow(q - v[ki], 2) + m_ESDFMap[base + v[ki]];
            }
        }
        
        for (int x = 0; x < W; ++x) {
            int k = 0; v[0] = 0; z[0] = -INF; z[1] = INF;
            for (int q = 1; q < H; ++q) {
                auto get_s = [&]() { return ((g[q*W+x] + q*q) - (g[v[k]*W+x] + v[k]*v[k])) / (2.0*(q-v[k])); };
                double s = get_s();
                while (s <= z[k]) { --k; s = get_s(); }
                v[++k] = q; z[k] = s; z[k+1] = INF;
            }
            for (int q = 0, ki = 0; q < H; ++q) {
                while (z[ki+1] < q) ++ki;
                int idx = q * W + x;
                m_ESDFMap[idx] = std::pow(q - v[ki], 2) + g[v[ki]*W+x];
                if (m_SurfMap[idx] == -1) m_ESDFMap[idx] = -m_ESDFMap[idx];
            }
        }
    }

    std::vector<int> m_GripMap;
    std::vector<int> m_SurfMap;
    std::vector<double> m_ESDFMap;
    int m_MapLenX, m_MapWeightY;
    bool m_IsSetESDF; 
};
} // namespace esdf_map
// ==============================================================================

cv::Mat g_base_vis_img; 
cv::Mat g_vis_img;      
std::vector<cv::Point> g_points; 
bool g_initial_planned = false;  

bool g_new_start_click = false; 
bool g_new_goal_click = false; 
cv::Point g_replan_click_pos; 

int g_mode = 0; 
std::vector<int> g_edit_occ; 
bool g_force_replan_from_edit = false; 
bool g_request_render = true; 

int g_map_w = 0, g_map_h = 0;
double g_res_x = 0.05, g_res_y = 0.05, g_origin_x = 0.0, g_origin_y = 0.0;
Eigen::Vector2d g_current_robot_pos(0.0, 0.0);
Eigen::Vector2d g_current_vel(0.0, 0.0);

std::string g_window_name = "Demo: [L/R Click]=Update, 'M'=Toggle Mode, 'J'=Init Plan";

void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    (void)userdata;
    int map_x = x;
    int map_y = g_map_h - 1 - y;

    if (g_mode == 0) {
        if (event == cv::EVENT_LBUTTONDOWN) {
            if (g_points.size() < 2) {
                g_points.push_back(cv::Point(x, y));
                g_request_render = true;
                if (g_points.size() == 2) {
                    std::cout << "Selected points: start (" << g_points[0].x << "," << g_points[0].y << ") goal (" << g_points[1].x << "," << g_points[1].y << ")\n";
                    std::cout << "Press 'J' to run Initial Global Plan.\n";
                }
            } else if (g_initial_planned) {
                g_replan_click_pos = cv::Point(x, y);
                g_new_start_click = true; 
            }
        } else if (event == cv::EVENT_RBUTTONDOWN) {
            if (g_initial_planned) {
                g_replan_click_pos = cv::Point(x, y);
                g_new_goal_click = true; 
            } else {
                g_points.clear();
                g_request_render = true;
                std::cout << "Points cleared! Please re-select start and goal.\n";
            }
        }
    } else if (g_mode == 1 && g_initial_planned) {
        int cx = static_cast<int>((g_current_robot_pos.x() - g_origin_x) / g_res_x);
        int cy = static_cast<int>((g_current_robot_pos.y() - g_origin_y) / g_res_y);
        
        bool is_lbutton = (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON)) || event == cv::EVENT_LBUTTONDOWN;
        bool is_rbutton = (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_RBUTTON)) || event == cv::EVENT_RBUTTONDOWN;

        if (is_lbutton || is_rbutton) {
            int val = is_lbutton ? 1 : 0;
            int brush_radius = 3; 
            for (int dy = -brush_radius; dy <= brush_radius; ++dy) {
                for (int dx = -brush_radius; dx <= brush_radius; ++dx) {
                    int nx = map_x + dx;
                    int ny = map_y + dy;
                    // 滑动窗口修改为 60个栅格 (3.0m)
                    if (std::abs(nx - cx) <= 60 && std::abs(ny - cy) <= 60) {
                        if (nx >= 0 && nx < g_map_w && ny >= 0 && ny < g_map_h) {
                            g_edit_occ[ny * g_map_w + nx] = val;
                        }
                    }
                }
            }
            g_request_render = true;
        }
    }
}

bool LoadOccupancyTxt(const std::string& filename, std::vector<int>& occ, int& w, int& h) { std::ifstream ifs(filename); if (!ifs.is_open()) return false; ifs >> w >> h; if (w <= 0 || h <= 0) return false; occ.resize(w * h); for (int i = 0; i < w * h; ++i) { if (!(ifs >> occ[i])) return false; } return true; }
bool LoadESDFRawTxt(const std::string& filename, std::vector<double>& esdf_raw, int& w, int& h) { std::ifstream ifs(filename); if (!ifs.is_open()) return false; ifs >> w >> h; if (w <= 0 || h <= 0) return false; esdf_raw.resize(w * h); for (int i = 0; i < w * h; ++i) { if (!(ifs >> esdf_raw[i])) return false; } return true; }
bool LoadMapInfo(const std::string& filename, int& w, int& h, double& res_x, double& res_y, double& origin_x, double& origin_y) { std::ifstream ifs(filename); if (!ifs.is_open()) return false; if (ifs >> w >> h >> res_x >> res_y) { if (!(ifs >> origin_x >> origin_y)) { origin_x = 0.0; origin_y = 0.0; } return true; } return false; }

// 主窗口交互渲染逻辑 (移除 JPS 和 Phase1)
void RenderVisualization(replanner& sys_replanner) {
    cv::Mat display = g_base_vis_img.clone();

    if (!g_edit_occ.empty()) {
        for (int y = 0; y < g_map_h; ++y) {
            for (int x = 0; x < g_map_w; ++x) {
                if (g_edit_occ[y * g_map_w + x] == 1) {
                    display.at<cv::Vec3b>(g_map_h - 1 - y, x) = cv::Vec3b(0, 0, 255); 
                }
            }
        }
    }

    if (g_initial_planned) {
        int cx = static_cast<int>((g_current_robot_pos.x() - g_origin_x) / g_res_x);
        int cy = static_cast<int>((g_current_robot_pos.y() - g_origin_y) / g_res_y);
        
        // 绘制 60 栅格大小的滑动窗口
        cv::rectangle(display, 
                      cv::Point(std::max(0, cx - 60), g_map_h - 1 - std::min(g_map_h - 1, cy + 60)),
                      cv::Point(std::min(g_map_w - 1, cx + 60), g_map_h - 1 - std::max(0, cy - 60)),
                      cv::Scalar(255, 0, 0), 2);

        auto traj = sys_replanner.GetTrajectory();
        Eigen::VectorXd times = traj.GetTimes();
        
        if (times.size() > 0) {
            double start_time = sys_replanner.GetTrajStartTime(g_current_robot_pos);
            double front_time = sys_replanner.GetFrontTime();
            if (front_time < start_time) front_time = start_time;

            std::vector<cv::Point> normal_pts;
            std::vector<cv::Point> highlight_pts;
            double current_t = 0.0;
            
            for (int i = 0; i < times.size(); ++i) {
                double T = times(i);
                for (double t = 0; t <= T + 1e-4; t += 0.05) { 
                    double abs_t = current_t + t;
                    if (abs_t >= start_time - 0.05) {
                        Eigen::Vector2d pos = traj.GetPosition(i, t);
                        int gx = static_cast<int>((pos.x() - g_origin_x) / g_res_x);
                        int gy = g_map_h - 1 - static_cast<int>((pos.y() - g_origin_y) / g_res_y);
                        
                        // 投影点 到 前瞻探障点 (红色高亮)
                        if (abs_t <= front_time + 0.05) highlight_pts.emplace_back(gx, gy);
                        // 其余 MINCO 段 (紫色)
                        if (abs_t >= front_time - 0.05) normal_pts.emplace_back(gx, gy);
                    }
                }
                current_t += T;
            }
            
            for (size_t i = 1; i < normal_pts.size(); ++i) 
                cv::line(display, normal_pts[i-1], normal_pts[i], cv::Scalar(255, 0, 255), 2); 
            for (size_t i = 1; i < highlight_pts.size(); ++i) 
                cv::line(display, highlight_pts[i-1], highlight_pts[i], cv::Scalar(0, 0, 255), 4); 
        }

        auto detour_path = sys_replanner.GetDetourPath();
        if (detour_path.size() >= 2) {
            for (size_t i = 1; i < detour_path.size(); ++i) {
                int dx1 = (detour_path[i-1].x() - g_origin_x) / g_res_x;
                int dy1 = g_map_h - 1 - (detour_path[i-1].y() - g_origin_y) / g_res_y;
                int dx2 = (detour_path[i].x() - g_origin_x) / g_res_x;
                int dy2 = g_map_h - 1 - (detour_path[i].y() - g_origin_y) / g_res_y;
                cv::line(display, cv::Point(dx1, dy1), cv::Point(dx2, dy2), cv::Scalar(255, 255, 0), 2); 
            }
        }

        Eigen::Vector2d proj_pt = sys_replanner.GetProjectedPoint();
        Eigen::Vector2d front_pt = sys_replanner.GetFrontPoint();
        int robot_px = (g_current_robot_pos.x() - g_origin_x) / g_res_x;
        int robot_py = g_map_h - 1 - (g_current_robot_pos.y() - g_origin_y) / g_res_y;
        
        if (proj_pt.norm() > 1e-4) {
            int px = (proj_pt.x() - g_origin_x) / g_res_x;
            int py = g_map_h - 1 - (proj_pt.y() - g_origin_y) / g_res_y;
            // 连接当前点到投影点的实线 (纯黄色)
            cv::line(display, cv::Point(robot_px, robot_py), cv::Point(px, py), cv::Scalar(0, 255, 255), 2);
            cv::circle(display, cv::Point(px, py), 5, cv::Scalar(0, 255, 255), -1); // 投影点 黄圈
        }
        
        if (front_pt.norm() > 1e-4) {
            int fx = (front_pt.x() - g_origin_x) / g_res_x;
            int fy = g_map_h - 1 - (front_pt.y() - g_origin_y) / g_res_y;
            cv::circle(display, cv::Point(fx, fy), 5, cv::Scalar(0, 165, 255), -1); // 探障前点 橙圈
        }
        
        cv::circle(display, cv::Point(robot_px, robot_py), 6, cv::Scalar(255, 255, 0), -1); // 机器人 青绿圈

        if (g_points.size() == 2) {
            // 终点颜色为橙色 (BGR: 0, 165, 255)
            cv::circle(display, g_points[1], 6, cv::Scalar(0, 165, 255), -1);
        }
    } else {
        for (size_t i = 0; i < g_points.size(); ++i) {
            cv::circle(display, g_points[i], 6, (i == 0) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255), -1);
        }
    }

    g_vis_img = display;
    cv::imshow(g_window_name, g_vis_img);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("planner_test");
    auto logger = node->get_logger();

    node->declare_parameter<std::string>("occupancy_file", "esdf_occupancy.txt");
    node->declare_parameter<std::string>("esdf_file", "esdf_distance.txt");
    node->declare_parameter<std::string>("map_info_file", "map_info.txt");

    std::string occ_path, esdf_path, info_path;
    node->get_parameter("occupancy_file", occ_path); 
    node->get_parameter("esdf_file", esdf_path); 
    node->get_parameter("map_info_file", info_path);

    if (!LoadMapInfo(info_path, g_map_w, g_map_h, g_res_x, g_res_y, g_origin_x, g_origin_y)) { 
        RCLCPP_WARN(logger, "Failed to load map info"); 
        return -1; 
    }

    std::vector<int> occupancy; LoadOccupancyTxt(occ_path, occupancy, g_map_w, g_map_h);
    std::vector<double> esdf_raw; LoadESDFRawTxt(esdf_path, esdf_raw, g_map_w, g_map_h);
    std::vector<double> esdf_dist(g_map_w * g_map_h);
    for (size_t i = 0; i < esdf_raw.size(); ++i) esdf_dist[i] = (esdf_raw[i] > 0 ? 1.0 : -1.0) * std::sqrt(std::abs(esdf_raw[i])) * g_res_x;

    cv::Mat esdf_gray(g_map_h, g_map_w, CV_8UC1);
    for (int y = 0; y < g_map_h; ++y) {
        for (int x = 0; x < g_map_w; ++x) {
            double d = std::max(0.0, esdf_dist[y * g_map_w + x]);
            int intensity = static_cast<int>(255.0 * (d / 5.0)); 
            esdf_gray.at<uchar>(y, x) = 255 - std::min(intensity, 255);
        }
    }
    cv::applyColorMap(esdf_gray, g_base_vis_img, cv::COLORMAP_JET);
    for (int y = 0; y < g_map_h; ++y) {
        for (int x = 0; x < g_map_w; ++x) {
            if (occupancy[y * g_map_w + x] == 1) g_base_vis_img.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0); 
        }
    }
    cv::flip(g_base_vis_img, g_base_vis_img, 0);
    g_vis_img = g_base_vis_img.clone();

    cv::namedWindow(g_window_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(g_window_name, 1000, 1000); 
    cv::setMouseCallback(g_window_name, mouseCallback);
    cv::imshow(g_window_name, g_vis_img);

    replanner sys_replanner;
    RCLCPP_INFO(logger, "Initializing Map into Replanner...");
    if (!sys_replanner.InitMapWithESDF(occupancy, esdf_dist, g_map_w, g_map_h, g_res_x, g_res_y, g_origin_x, g_origin_y)) {
        RCLCPP_ERROR(logger, "Replanner Map Init Failed!"); return -1;
    }
    
    g_edit_occ = sys_replanner.GetGlobalOccupancy();

    int key = 0;
    
    // --- 【双循环架构：阶段一 (初始化全局规划)】 ---
    while (key != 27 && !g_initial_planned && rclcpp::ok()) {
        key = cv::waitKey(10) & 0xFF;

        if (g_points.size() == 2 && (key == 'j' || key == 'J' || key == 32)) {
            double sx_phys = g_origin_x + g_points[0].x * g_res_x; 
            double sy_phys = g_origin_y + (g_map_h - 1 - g_points[0].y) * g_res_y;
            double gx_phys = g_origin_x + g_points[1].x * g_res_x; 
            double gy_phys = g_origin_y + (g_map_h - 1 - g_points[1].y) * g_res_y;
            
            if (sys_replanner.InitPoint(sx_phys, sy_phys, gx_phys, gy_phys)) {
                g_initial_planned = true;
                g_current_robot_pos << sx_phys, sy_phys;
                
                // === [第一次规划时弹出单独的 Debug 三线图窗口] ===
                cv::Mat debug_img = g_base_vis_img.clone();
                auto jps_path = sys_replanner.GetJpsGridPath();
                for (size_t i = 1; i < jps_path.size(); ++i) {
                    cv::Point pt1(jps_path[i-1].x(), g_map_h - 1 - jps_path[i-1].y());
                    cv::Point pt2(jps_path[i].x(), g_map_h - 1 - jps_path[i].y());
                    cv::line(debug_img, pt1, pt2, cv::Scalar(255, 0, 0), 2); 
                }
                auto phase1_path = sys_replanner.GetPhase1PhysPoints();
                for (size_t i = 1; i < phase1_path.size(); ++i) {
                    cv::Point pt1((phase1_path[i-1].x() - g_origin_x) / g_res_x, g_map_h - 1 - (phase1_path[i-1].y() - g_origin_y) / g_res_y);
                    cv::Point pt2((phase1_path[i].x() - g_origin_x) / g_res_x, g_map_h - 1 - (phase1_path[i].y() - g_origin_y) / g_res_y);
                    cv::line(debug_img, pt1, pt2, cv::Scalar(0, 255, 0), 2); 
                }
                auto traj = sys_replanner.GetTrajectory();
                Eigen::VectorXd times = traj.GetTimes();
                double current_t = 0.0;
                std::vector<cv::Point> debug_pts;
                for (int i = 0; i < times.size(); ++i) {
                    double T = times(i);
                    for (double t = 0; t <= T + 1e-4; t += 0.05) { 
                        Eigen::Vector2d pos = traj.GetPosition(i, t);
                        int gx = static_cast<int>((pos.x() - g_origin_x) / g_res_x);
                        int gy = g_map_h - 1 - static_cast<int>((pos.y() - g_origin_y) / g_res_y);
                        debug_pts.emplace_back(gx, gy);
                    }
                    current_t += T;
                }
                for (size_t i = 1; i < debug_pts.size(); ++i) cv::line(debug_img, debug_pts[i-1], debug_pts[i], cv::Scalar(255, 0, 255), 2); 
                
                cv::circle(debug_img, g_points[0], 6, cv::Scalar(0, 255, 0), -1); 
                cv::circle(debug_img, g_points[1], 6, cv::Scalar(0, 165, 255), -1); 
                
                cv::namedWindow("Optimization Stages (Blue=JPS, Green=Ph1, Purple=Ph2)", cv::WINDOW_NORMAL);
                cv::resizeWindow("Optimization Stages (Blue=JPS, Green=Ph1, Purple=Ph2)", 1000, 1000);
                cv::imshow("Optimization Stages (Blue=JPS, Green=Ph1, Purple=Ph2)", debug_img);

                g_request_render = true; 
            } else {
                RCLCPP_WARN(logger, "Initial global plan failed! Please select new points.");
                g_points.clear();
                g_request_render = true;
            }
        }
        
        if (g_request_render) {
            RenderVisualization(sys_replanner);
            g_request_render = false;
        }
        rclcpp::spin_some(node);
    }

    // --- 【双循环架构：阶段二 (实时交互测试)】 ---
    while (key != 27 && rclcpp::ok()) {
        key = cv::waitKey(10) & 0xFF;

        if (key == 'm' || key == 'M') {
            if (g_mode == 0 && g_initial_planned) {
                g_mode = 1;
                g_edit_occ = sys_replanner.GetGlobalOccupancy();
                std::cout << "[模式切换] -> 进入局部编辑障碍物模式。\n";
            } else if (g_mode == 1) {
                g_mode = 0;
                g_force_replan_from_edit = true;
                std::cout << "[模式切换] -> 返回更新点模式。触发即时局部重规划...\n";
            }
            g_request_render = true;
        }

        if (g_new_start_click || g_new_goal_click) {
            double cur_x_phys = g_origin_x + g_replan_click_pos.x * g_res_x;
            double cur_y_phys = g_origin_y + (g_map_h - 1 - g_replan_click_pos.y) * g_res_y;
            Eigen::Vector2d clicked_pos(cur_x_phys, cur_y_phys);
            
            auto t_replan = std::chrono::high_resolution_clock::now();
            if (g_new_start_click) {
                sys_replanner.Update(clicked_pos, g_current_vel);
                g_current_robot_pos = clicked_pos; 
                g_new_start_click = false;
            } 
            else if (g_new_goal_click) {
                sys_replanner.UpdateGoal(g_current_robot_pos, clicked_pos, g_current_vel);
                g_points[1] = g_replan_click_pos; 
                g_new_goal_click = false;
            }
            double ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_replan).count();
            RCLCPP_INFO(logger, "Interactive Replanning Time: %.2f ms", ms);
            g_request_render = true;
        }

        if (g_force_replan_from_edit && g_initial_planned) {
            int cx = static_cast<int>((g_current_robot_pos.x() - g_origin_x) / g_res_x);
            int cy = static_cast<int>((g_current_robot_pos.y() - g_origin_y) / g_res_y);
            
            int min_x = std::max(0, cx - 60);
            int max_x = std::min(g_map_w - 1, cx + 60);
            int min_y = std::max(0, cy - 60);
            int max_y = std::min(g_map_h - 1, cy + 60);
            
            int lw = max_x - min_x + 1;
            int lh = max_y - min_y + 1;
            
            std::vector<int> local_occ(lw * lh, 0);
            for (int y = 0; y < lh; ++y) {
                for (int x = 0; x < lw; ++x) {
                    local_occ[y * lw + x] = g_edit_occ[(min_y + y) * g_map_w + (min_x + x)];
                }
            }
            
            esdf_map::esdf_map local_esdf_calc;
            local_esdf_calc.Init(local_occ, lw, lh);
            local_esdf_calc.SetSurfMap();
            local_esdf_calc.ComputeEDT();
            
            std::vector<double> local_esdf(lw * lh, 0.0);
            for (size_t i = 0; i < local_esdf.size(); ++i) {
                double raw = local_esdf_calc.m_ESDFMap[i];
                local_esdf[i] = (raw < 0 ? -std::sqrt(-raw) : std::sqrt(raw)) * g_res_x;
            }
            
            double local_ox = g_origin_x + min_x * g_res_x;
            double local_oy = g_origin_y + min_y * g_res_y;

            auto t_start = std::chrono::high_resolution_clock::now();
            
            sys_replanner.Update(g_current_robot_pos, g_current_vel, local_occ, local_esdf, lw, lh, local_ox, local_oy);
            
            double ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            RCLCPP_INFO(logger, "Local Map Injection & Replanning Time: %.2f ms", ms);
            
            g_force_replan_from_edit = false;
            g_request_render = true;
        }

        if (g_request_render) {
            RenderVisualization(sys_replanner);
            g_request_render = false;
        }

        rclcpp::spin_some(node);
    }

    cv::destroyAllWindows(); rclcpp::shutdown(); return 0;
}