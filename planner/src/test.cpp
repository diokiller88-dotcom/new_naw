#include "planner_2d/replanner.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>
#include <cmath>
#include <memory>
#include <string>
#include <chrono>

using namespace planner_2d;

cv::Mat g_base_vis_img; 
cv::Mat g_vis_img;      
std::vector<cv::Point> g_points; 
bool g_initial_planned = false;  

bool g_new_start_click = false; 
bool g_new_goal_click = false; 
cv::Point g_replan_click_pos; 

std::string g_window_name = "Demo: [LClick]=Start/Pose, [RClick]=Goal, 'J' to Run";

void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    (void)flags; (void)userdata;
    if (event == cv::EVENT_LBUTTONDOWN) {
        if (g_points.size() < 2) {
            g_points.push_back(cv::Point(x, y));
            cv::circle(g_vis_img, cv::Point(x, y), 5, (g_points.size() == 1) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), -1);
            cv::imshow(g_window_name, g_vis_img);
            if (g_points.size() == 2) {
                std::cout << "Selected points: start (" << g_points[0].x << "," << g_points[0].y
                          << ") goal (" << g_points[1].x << "," << g_points[1].y << ")\n";
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
            g_vis_img = g_base_vis_img.clone();
            cv::imshow(g_window_name, g_vis_img);
            std::cout << "Points cleared! Please re-select start and goal.\n";
        }
    }
}

bool LoadOccupancyTxt(const std::string& filename, std::vector<int>& occ, int& w, int& h) { std::ifstream ifs(filename); if (!ifs.is_open()) return false; ifs >> w >> h; if (w <= 0 || h <= 0) return false; occ.resize(w * h); for (int i = 0; i < w * h; ++i) { if (!(ifs >> occ[i])) return false; } return true; }
bool LoadESDFRawTxt(const std::string& filename, std::vector<double>& esdf_raw, int& w, int& h) { std::ifstream ifs(filename); if (!ifs.is_open()) return false; ifs >> w >> h; if (w <= 0 || h <= 0) return false; esdf_raw.resize(w * h); for (int i = 0; i < w * h; ++i) { if (!(ifs >> esdf_raw[i])) return false; } return true; }
bool LoadMapInfo(const std::string& filename, int& w, int& h, double& res_x, double& res_y, double& origin_x, double& origin_y) { std::ifstream ifs(filename); if (!ifs.is_open()) return false; if (ifs >> w >> h >> res_x >> res_y) { if (!(ifs >> origin_x >> origin_y)) { origin_x = 0.0; origin_y = 0.0; } return true; } return false; }


void DrawMincoTrajectory(cv::Mat& img, const minco& traj, double res_x, double res_y, double origin_x, double origin_y, int height, double start_time = 0.0) {
    Eigen::VectorXd times = traj.GetTimes();
    if(times.size() == 0) return;

    std::vector<cv::Point> draw_pts;
    double current_t = 0.0;
    for (int i = 0; i < times.size(); ++i) {
        double T = times(i);
        for (double t = 0; t <= T + 1e-4; t += 0.05) { 
            if (current_t + t >= start_time - 0.05) {
                Eigen::Vector2d pos = traj.GetPosition(i, t);
                int gx = static_cast<int>((pos.x() - origin_x) / res_x);
                int gy = height - 1 - static_cast<int>((pos.y() - origin_y) / res_y);
                draw_pts.emplace_back(gx, gy);
            }
        }
        current_t += T;
    }
    for (size_t i = 1; i < draw_pts.size(); ++i) cv::line(img, draw_pts[i-1], draw_pts[i], cv::Scalar(255, 0, 255), 2);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("planner_test");
    auto logger = node->get_logger();

    node->declare_parameter<std::string>("occupancy_file", "esdf_occupancy.txt");
    node->declare_parameter<std::string>("esdf_file", "esdf_distance.txt");
    node->declare_parameter<std::string>("map_info_file", "map_info.txt");

    std::string occ_path, esdf_path, info_path;
    node->get_parameter("occupancy_file", occ_path); node->get_parameter("esdf_file", esdf_path); node->get_parameter("map_info_file", info_path);

    int width = 0, height = 0;
    double res_x = 0.05, res_y = 0.05, origin_x = 0.0, origin_y = 0.0; 
    
    if (!LoadMapInfo(info_path, width, height, res_x, res_y, origin_x, origin_y)) { RCLCPP_WARN(logger, "Failed to load map info"); return -1; }

    std::vector<int> occupancy; LoadOccupancyTxt(occ_path, occupancy, width, height);
    std::vector<double> esdf_raw; LoadESDFRawTxt(esdf_path, esdf_raw, width, height);
    std::vector<double> esdf_dist(width * height);
    for (size_t i = 0; i < esdf_raw.size(); ++i) esdf_dist[i] = (esdf_raw[i] > 0 ? 1.0 : -1.0) * std::sqrt(std::abs(esdf_raw[i])) * res_x;

    cv::Mat esdf_gray(height, width, CV_8UC1);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            double d = std::max(0.0, esdf_dist[y * width + x]);
            int intensity = static_cast<int>(255.0 * (d / 5.0)); 
            esdf_gray.at<uchar>(y, x) = 255 - std::min(intensity, 255);
        }
    }
    cv::Mat esdf_color; cv::applyColorMap(esdf_gray, esdf_color, cv::COLORMAP_JET);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (occupancy[y * width + x] == 1) esdf_color.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0); 
        }
    }
    cv::flip(esdf_color, g_base_vis_img, 0);
    g_vis_img = g_base_vis_img.clone();

    cv::namedWindow(g_window_name, cv::WINDOW_NORMAL);
    cv::setMouseCallback(g_window_name, mouseCallback);
    cv::imshow(g_window_name, g_vis_img);

    replanner sys_replanner;
    RCLCPP_INFO(logger, "Initializing Map into Replanner...");
    if (!sys_replanner.InitMapWithESDF(occupancy, esdf_dist, width, height, res_x, res_y, origin_x, origin_y)) {
        RCLCPP_ERROR(logger, "Replanner Map Init Failed!"); return -1;
    }
    RCLCPP_INFO(logger, "Map Initialization Done.");

    Eigen::Vector2d current_robot_pos;
    int key = 0;

    while (key != 27 && !g_initial_planned) {
        key = cv::waitKey(10) & 0xFF;
        if (g_points.size() == 2 && (key == 'j' || key == 'J' || key == 32)) {
            double sx_phys = origin_x + g_points[0].x * res_x; double sy_phys = origin_y + (height - 1 - g_points[0].y) * res_y;
            double gx_phys = origin_x + g_points[1].x * res_x; double gy_phys = origin_y + (height - 1 - g_points[1].y) * res_y;
            
            if (sys_replanner.InitPoint(sx_phys, sy_phys, gx_phys, gy_phys)) {
                g_initial_planned = true;
                current_robot_pos << sx_phys, sy_phys;
                
                g_vis_img = g_base_vis_img.clone();
                cv::circle(g_vis_img, g_points[1], 6, cv::Scalar(0, 0, 255), -1); 
                cv::circle(g_vis_img, g_points[0], 6, cv::Scalar(0, 255, 255), -1); 
                DrawMincoTrajectory(g_vis_img, sys_replanner.GetTrajectory(), res_x, res_y, origin_x, origin_y, height);
                cv::imshow(g_window_name, g_vis_img);

                cv::Mat debug_img = g_base_vis_img.clone();
                auto jps_path = sys_replanner.GetJpsGridPath();
                for (size_t i = 1; i < jps_path.size(); ++i) {
                    cv::Point pt1(jps_path[i-1].x(), height - 1 - jps_path[i-1].y());
                    cv::Point pt2(jps_path[i].x(), height - 1 - jps_path[i].y());
                    cv::line(debug_img, pt1, pt2, cv::Scalar(255, 0, 0), 2);
                }
                auto phase1_path = sys_replanner.GetPhase1PhysPoints();
                for (size_t i = 1; i < phase1_path.size(); ++i) {
                    cv::Point pt1((phase1_path[i-1].x() - origin_x) / res_x, height - 1 - (phase1_path[i-1].y() - origin_y) / res_y);
                    cv::Point pt2((phase1_path[i].x() - origin_x) / res_x, height - 1 - (phase1_path[i].y() - origin_y) / res_y);
                    cv::line(debug_img, pt1, pt2, cv::Scalar(0, 255, 0), 2);
                }
                DrawMincoTrajectory(debug_img, sys_replanner.GetTrajectory(), res_x, res_y, origin_x, origin_y, height);
                cv::circle(debug_img, g_points[0], 6, cv::Scalar(0, 255, 0), -1); cv::circle(debug_img, g_points[1], 6, cv::Scalar(0, 0, 255), -1); 
                cv::namedWindow("Optimization Stages (Blue=JPS, Green=Ph1, Purple=Ph2)", cv::WINDOW_NORMAL);
                cv::imshow("Optimization Stages (Blue=JPS, Green=Ph1, Purple=Ph2)", debug_img);
            } else {
                RCLCPP_WARN(logger, "Initial global plan failed (Target unreachable or occupied)! Please select new points.");
                g_points.clear();
                g_vis_img = g_base_vis_img.clone();
                cv::imshow(g_window_name, g_vis_img);
            }
        }
        rclcpp::spin_some(node);
    }
    while (key != 27 && rclcpp::ok()) {
        key = cv::waitKey(10) & 0xFF;

        if (g_new_start_click || g_new_goal_click) {
            double cur_x_phys = origin_x + g_replan_click_pos.x * res_x;
            double cur_y_phys = origin_y + (height - 1 - g_replan_click_pos.y) * res_y;
            Eigen::Vector2d clicked_pos(cur_x_phys, cur_y_phys);
            Eigen::Vector2d zero_velocity(0.0, 0.0);
            
            auto t_replan = std::chrono::high_resolution_clock::now();

            if (g_new_start_click) {
                sys_replanner.Update(clicked_pos, zero_velocity);
                current_robot_pos = clicked_pos; 
                g_new_start_click = false;
            } 
            else if (g_new_goal_click) {
                sys_replanner.UpdateGoal(current_robot_pos, clicked_pos, zero_velocity);
                g_points[1] = g_replan_click_pos; 
                g_new_goal_click = false;
            }

            double ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_replan).count();
            RCLCPP_INFO(logger, "Replanning Time: %.2f ms", ms);

            g_vis_img = g_base_vis_img.clone();
            cv::circle(g_vis_img, g_points[1], 6, cv::Scalar(0, 0, 255), -1); 
            int rx = (current_robot_pos.x() - origin_x) / res_x, ry = height - 1 - (current_robot_pos.y() - origin_y) / res_y;
            cv::circle(g_vis_img, cv::Point(rx, ry), 6, cv::Scalar(0, 255, 255), -1); 

            double current_traj_time = sys_replanner.GetTrajStartTime(current_robot_pos);
            DrawMincoTrajectory(g_vis_img, sys_replanner.GetTrajectory(), res_x, res_y, origin_x, origin_y, height, current_traj_time);
            cv::imshow(g_window_name, g_vis_img);
        }
        rclcpp::spin_some(node);
    }

    cv::destroyAllWindows(); rclcpp::shutdown(); return 0;
}