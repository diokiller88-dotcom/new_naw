#pragma once

#include <vector>
#include <cstdint>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace relocation {

    constexpr float max_distance = 10.0f;
    constexpr float min_distance = 0.0f;
    constexpr float dis_resolution = 1.0f;
    constexpr float max_height = 2.0f;
    constexpr float min_height = 0.0f;
    constexpr float height_resolution = 0.25f; 
    constexpr int iris_rows = 10;              
    constexpr int iris_cols = 360;             
    constexpr int iris_sector_bins = 36;
    constexpr int iris_height_bins = 8;
    constexpr int iris_binary_vec_size = iris_rows + iris_sector_bins + iris_height_bins + iris_rows * iris_sector_bins;

    constexpr int nscale = 4;
    constexpr int min_wavelength = 18;
    constexpr float mult = 1.6f;
    constexpr float sigma_onf = 0.75f;

    class iris {
    public:
        struct FeatureDesc {
            cv::Mat1b img; 
            cv::Mat1b T;   
            cv::Mat1b M;   
        };

        static cv::Mat1b GetIris(const pcl::PointCloud<pcl::PointXYZ> &cloud);
        static FeatureDesc GetFeature(const cv::Mat1b &src);
        static std::vector<uint8_t> IrisToBinaryVec(const cv::Mat1b &src);
        static float Compare(const FeatureDesc &img1, const FeatureDesc &img2, int *bias = nullptr);
        static cv::Mat circShift(const cv::Mat &src, int shift_m_rows, int shift_n_cols);

    private:
        static void LoGFeatureEncode(const cv::Mat1b &src, cv::Mat1b &T, cv::Mat1b &M);
        static std::vector<cv::Mat2f> LogGaborFilter(const cv::Mat1f &src);
        static void GetHammingDistance(const cv::Mat1b &T1, const cv::Mat1b &M1, const cv::Mat1b &T2, const cv::Mat1b &M2, int scale, float &dis, int &bias);
        
        static void Recomb(cv::Mat &src, cv::Mat &dst);
        static void ForwardFFT(cv::Mat &Src, cv::Mat *FImg);
        static void highpass(cv::Size sz, cv::Mat& dst);
        static float logpolar(cv::Mat& src, cv::Mat& dst);
        static cv::RotatedRect FFTMatch(const cv::Mat& im0_in, const cv::Mat& im1_in);
    };

} // namespace relocation
