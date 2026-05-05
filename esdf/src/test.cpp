#include "esdf/esdf_map.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <chrono>

constexpr int WIDTH = 800;
constexpr int HEIGHT = 800;
constexpr double MIN_X = -20.0;
constexpr double MAX_X = 20.0;
constexpr double MIN_Y = -20.0;
constexpr double MAX_Y = 20.0;
constexpr double MIN_Z = -1.0;
constexpr double MAX_Z = 5.0;
constexpr double RES_X = (MAX_X - MIN_X) / WIDTH; // 0.05
constexpr double RES_Y = (MAX_Y - MIN_Y) / HEIGHT; // 0.05

constexpr float ARTIFACT_THICKNESS = 0.05f;
constexpr int STAT_K = 20;
constexpr double STAT_STD_MUL = 1.5;

constexpr double SEARCH_RADIUS_METERS = 0.40; 
constexpr int MAX_RADIUS_PIXELS = static_cast<int>(std::ceil(SEARCH_RADIUS_METERS / RES_X));
constexpr double VIS_MAX_DIST_METERS = 5.0;

constexpr double TOTAL_Z_RANGE = MAX_Z - MIN_Z;
constexpr int Z_BITS = 128;
constexpr double leafZ = TOTAL_Z_RANGE / Z_BITS; // 0.046875

constexpr double obsLowerHeight = 0.0;     
constexpr double obsUpperHeight = 0.40;    
constexpr double maxDownStepHeight = 0.30; 
constexpr double maxUpStepHeight   = 0.25; 
constexpr double densityRatioThresh = 0.9;
constexpr double obsHeightThreshMeters = 0.30;


struct EditContext {
    std::vector<int>* occupancy;
    cv::Mat* img;
    int width;
    int height;
    bool updated;
    int brush_size = 2; 
};

void onMouseEditMap(int event, int x, int y, int flags, void* userdata) {
    EditContext* ctx = reinterpret_cast<EditContext*>(userdata);
    if (x < 0 || x >= ctx->width || y < 0 || y >= ctx->height) return;

    bool is_left = (flags & cv::EVENT_FLAG_LBUTTON) || (event == cv::EVENT_LBUTTONDOWN);
    bool is_right = (flags & cv::EVENT_FLAG_RBUTTON) || (event == cv::EVENT_RBUTTONDOWN);

    if (is_left || is_right) {
        int val = is_left ? 0 : 1; 
        cv::Vec3b color = is_left ? cv::Vec3b(0, 255, 0) : cv::Vec3b(0, 0, 255); 

        for (int dy = -ctx->brush_size; dy <= ctx->brush_size; ++dy) {
            for (int dx = -ctx->brush_size; dx <= ctx->brush_size; ++dx) {
                int nx = x + dx;
                int ny = y + dy;
                if (nx >= 0 && nx < ctx->width && ny >= 0 && ny < ctx->height) {
                    int map_y = ctx->height - 1 - ny;
                    int map_x = nx;
                    
                    (*ctx->occupancy)[map_y * ctx->width + map_x] = val;
                    ctx->img->at<cv::Vec3b>(ny, nx) = color;
                }
            }
        }
        ctx->updated = true;
    }
}


int main(int argc, char** argv) {
    std::string pcd_path = "/home/hyl/new_nav/esdf/PCD/1.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
        std::cerr << "Failed to load PCD file!" << std::endl;
        return -1;
    }

    if (!cloud->points.empty()) {
        float current_min_z = std::numeric_limits<float>::max();
        for (const auto& pt : cloud->points) if (pt.z < current_min_z) current_min_z = pt.z;

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& pt : cloud->points) {
            if (pt.z > current_min_z + ARTIFACT_THICKNESS) filtered_cloud->points.push_back(pt);
        }
        filtered_cloud->width = filtered_cloud->points.size();
        filtered_cloud->height = 1;
        filtered_cloud->is_dense = true;
        cloud = filtered_cloud;
    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(STAT_K);
    sor.setStddevMulThresh(STAT_STD_MUL);
    sor.filter(*cloud);

    std::vector<std::array<uint32_t, 4>> cell_z_bits(WIDTH * HEIGHT, {0, 0, 0, 0});
    std::vector<int> state(WIDTH * HEIGHT, 0); // 0: unknown, 1: occupy, 2: free
    std::vector<int> groundz(WIDTH * HEIGHT, -1);

    for (const auto& pt : cloud->points) {
        if (pt.x < MIN_X || pt.x >= MAX_X || pt.y < MIN_Y || pt.y >= MAX_Y) continue;
        if (pt.z < MIN_Z || pt.z > MAX_Z) continue;
        int gx = static_cast<int>((pt.x - MIN_X) / RES_X);
        int gy = static_cast<int>((pt.y - MIN_Y) / RES_Y);
        int z_idx = static_cast<int>((pt.z - MIN_Z) / leafZ);
        if (gx >= 0 && gx < WIDTH && gy >= 0 && gy < HEIGHT && z_idx >= 0 && z_idx < Z_BITS) {
            cell_z_bits[gy * WIDTH + gx][z_idx / 32] |= (1U << (z_idx % 32));
        }
    }

    const int obs_height_bits = static_cast<int>(obsHeightThreshMeters / leafZ);
    const int max_diff_bits = static_cast<int>(maxDownStepHeight / leafZ);

    cv::Mat map_img_s1 = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    for (int i = 0; i < WIDTH * HEIGHT; ++i) {
        int y = i / WIDTH;
        int x = i % WIDTH;
        cv::Vec3b& pixel = map_img_s1.at<cv::Vec3b>(HEIGHT - 1 - y, x);

        if (cell_z_bits[i][0] == 0 && cell_z_bits[i][1] == 0 && cell_z_bits[i][2] == 0 && cell_z_bits[i][3] == 0) {
            pixel = cv::Vec3b(50, 50, 50);
            continue;
        }

        int minz = -1, maxz = -1, bit_count = 0;
        for (int j = 0; j < 4; ++j) {
            if (cell_z_bits[i][j] != 0) {
                if (minz == -1) minz = j * 32 + __builtin_ctz(cell_z_bits[i][j]);
                maxz = j * 32 + 31 - __builtin_clz(cell_z_bits[i][j]);
                bit_count += __builtin_popcount(cell_z_bits[i][j]);
            }
        }

        int height_diff = maxz - minz + 1;
        double ratio = static_cast<double>(bit_count) / height_diff;

        if (ratio >= densityRatioThresh && height_diff >= obs_height_bits) {
            state[i] = 1;
            pixel = cv::Vec3b(0, 0, 255);
        } else {
            state[i] = 2;
            groundz[i] = maxz;
            pixel = cv::Vec3b(0, 255, 0);
        }
    }
    cv::imwrite("s1_initial_occupancy.png", map_img_s1);
    std::cout << "S1 Initial map saved." << std::endl;

    const int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};

    std::vector<int> next_state = state;
    cv::Mat map_img_s2 = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            int i = y * WIDTH + x;
            cv::Vec3b& pixel = map_img_s2.at<cv::Vec3b>(HEIGHT - 1 - y, x);

            if (state[i] == 2) {
                pixel = cv::Vec3b(0, 255, 0);
                for (int k = 0; k < 8; ++k) {
                    int nx = x + dx[k], ny = y + dy[k];
                    if (nx >= 0 && nx < WIDTH && ny >= 0 && ny < HEIGHT) {
                        int ni = ny * WIDTH + nx;
                        if (state[ni] != 0 && groundz[ni] != -1) {
                            if (std::abs(groundz[i] - groundz[ni]) > max_diff_bits) {
                                next_state[i] = 1;
                                pixel = cv::Vec3b(255, 0, 0); 
                                break;
                            }
                        }
                    }
                }
            } else if (state[i] == 1) {
                pixel = cv::Vec3b(0, 0, 255);
            } else {
                pixel = cv::Vec3b(50, 50, 50);
            }
        }
    }
    state = next_state;
    cv::imwrite("s2_gradient_detected.png", map_img_s2);
    std::cout << "S2 Gradient map saved." << std::endl;

    std::vector<int> occupancy(WIDTH * HEIGHT, 0);
    for (int i = 0; i < WIDTH * HEIGHT; ++i) {
        occupancy[i] = (state[i] == 1) ? 1 : 0;
    }

    struct Point2D { int x, y; };
    std::vector<Point2D> current_obs;
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            if (occupancy[y * WIDTH + x] == 1) current_obs.push_back({x, y});
        }
    }

    const double max_dist_sq = MAX_RADIUS_PIXELS * MAX_RADIUS_PIXELS;
    std::vector<std::pair<Point2D, Point2D>> dense_lines;
    
    for (const auto& pt : current_obs) {
        int x = pt.x, y = pt.y;
        for (int mdy = 0; mdy <= MAX_RADIUS_PIXELS; ++mdy) {
            for (int mdx = -MAX_RADIUS_PIXELS; mdx <= MAX_RADIUS_PIXELS; ++mdx) {
                if (mdy == 0 && mdx <= 0) continue; 
                int nx = x + mdx, ny = y + mdy;
                if (nx >= 0 && nx < WIDTH && ny >= 0 && ny < HEIGHT && occupancy[ny * WIDTH + nx] == 1) {
                    if (mdx * mdx + mdy * mdy <= max_dist_sq) dense_lines.push_back({{x, y}, {nx, ny}});
                }
            }
        }
    }

    for (const auto& line : dense_lines) {
        int x0 = line.first.x, y0 = line.first.y;
        int x1 = line.second.x, y1 = line.second.y;
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;
        while (true) {
            if (x0 >= 0 && x0 < WIDTH && y0 >= 0 && y0 < HEIGHT) occupancy[y0 * WIDTH + x0] = 1;
            if (x0 == x1 && y0 == y1) break;
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    // ================== 交互式编辑环节 ==================
    cv::Mat interactive_img = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            if (occupancy[y * WIDTH + x] == 1) {
                interactive_img.at<cv::Vec3b>(HEIGHT - 1 - y, x) = cv::Vec3b(0, 0, 255); 
            } else {
                interactive_img.at<cv::Vec3b>(HEIGHT - 1 - y, x) = cv::Vec3b(0, 255, 0); 
            }
        }
    }

    EditContext edit_ctx;
    edit_ctx.occupancy = &occupancy;
    edit_ctx.img = &interactive_img;
    edit_ctx.width = WIDTH;
    edit_ctx.height = HEIGHT;
    edit_ctx.updated = true;

    std::string win_name = "Interactive Map Editor";
    cv::namedWindow(win_name, cv::WINDOW_NORMAL);
    cv::setMouseCallback(win_name, onMouseEditMap, &edit_ctx);

    std::cout << "\n=======================================================" << std::endl;
    std::cout << ">>> 交互式地图编辑器已启动 <<<" << std::endl;
    std::cout << "[操作说明]:" << std::endl;
    std::cout << "  - 左键涂抹: 清除障碍，设为自由区域 (绿色)" << std::endl;
    std::cout << "  - 右键涂抹: 添加障碍，设为占据区域 (红色)" << std::endl;
    std::cout << "  - 按下 'S' 键: 确认并保存地图，开始计算 ESDF" << std::endl;
    std::cout << "=======================================================\n" << std::endl;

    while (true) {
        if (edit_ctx.updated) {
            cv::imshow(win_name, interactive_img);
            edit_ctx.updated = false;
        }
        
        int key = cv::waitKey(10) & 0xFF;
        if (key == 's' || key == 'S') {
            std::cout << "收到 'S' 指令，正在保存修改并计算 ESDF..." << std::endl;
            break;
        } else if (key == 27) { 
            return 0;
        }
    }
    cv::destroyWindow(win_name);

    auto t_coarse_start = std::chrono::steady_clock::now();
    esdf_map::esdf_map esdf_calc;
    esdf_calc.Init(occupancy, WIDTH, HEIGHT);
    esdf_calc.SetSurfMap();
    esdf_calc.ComputeEDT();
    auto t_coarse_end = std::chrono::steady_clock::now();
    double coarse_time = std::chrono::duration_cast<std::chrono::milliseconds>(t_coarse_end - t_coarse_start).count();
    std::cout << "esdf build time: " << coarse_time << " ms" << std::endl;

    cv::Mat esdf_img = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            int i = y * WIDTH + x;
            if (occupancy[i] == 1) {
                esdf_img.at<cv::Vec3b>(HEIGHT - 1 - y, x) = cv::Vec3b(0, 0, 255);
            } else {
                double dist = esdf_calc.m_ESDFMap[i] * RES_X;
                unsigned char intensity = static_cast<unsigned char>(std::min(255.0, (dist / VIS_MAX_DIST_METERS) * 255.0));
                esdf_img.at<cv::Vec3b>(HEIGHT - 1 - y, x) = cv::Vec3b(intensity, intensity, intensity);
            }
        }
    }
    cv::imwrite("s4_esdf_map_final.png", esdf_img);
    std::cout << "S4 ESDF map saved." << std::endl;
    std::ofstream occ_file("esdf_occupancy.txt");
    if (occ_file.is_open()) {
        occ_file << WIDTH << " " << HEIGHT << std::endl;
        for (int i = 0; i < WIDTH * HEIGHT; ++i) {
            occ_file << occupancy[i] << " ";
            if ((i + 1) % 80 == 0) occ_file << std::endl;
        }
        occ_file.close();
    }

    std::ofstream esdf_file("esdf_distance.txt");
    if (esdf_file.is_open()) {
        esdf_file << WIDTH << " " << HEIGHT << std::endl;
        for (int i = 0; i < WIDTH * HEIGHT; ++i) {
            esdf_file << esdf_calc.m_ESDFMap[i] << " ";
            if ((i + 1) % 10 == 0) esdf_file << std::endl;
        }
        esdf_file.close();
    }

    std::ofstream info_file("map_info.txt");
    if (info_file.is_open()) {
        info_file << WIDTH << " " << HEIGHT << " " 
                  << RES_X << " " << RES_Y << " " 
                  << MIN_X << " " << MIN_Y << std::endl;
        info_file.close();
    }

    std::cout << "All data saved as text files." << std::endl;
    return 0;
}