#include "relocation/iris.hpp"
#include <numeric>
#include <algorithm>
#include <limits>

namespace relocation {

    void iris::Recomb(cv::Mat &src, cv::Mat &dst) {
        int cx = src.cols >> 1; int cy = src.rows >> 1;
        cv::Mat tmp(src.size(), src.type());
        src(cv::Rect(0, 0, cx, cy)).copyTo(tmp(cv::Rect(cx, cy, cx, cy)));
        src(cv::Rect(cx, cy, cx, cy)).copyTo(tmp(cv::Rect(0, 0, cx, cy)));
        src(cv::Rect(cx, 0, cx, cy)).copyTo(tmp(cv::Rect(0, cy, cx, cy)));
        src(cv::Rect(0, cy, cx, cy)).copyTo(tmp(cv::Rect(cx, 0, cx, cy)));
        dst = tmp;
    }

    void iris::ForwardFFT(cv::Mat &Src, cv::Mat *FImg) {
        int M = cv::getOptimalDFTSize(Src.rows), N = cv::getOptimalDFTSize(Src.cols);
        cv::Mat padded;
        cv::copyMakeBorder(Src, padded, 0, M - Src.rows, 0, N - Src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
        cv::Mat planes[] = { cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F) };
        cv::Mat complexImg; cv::merge(planes, 2, complexImg);
        cv::dft(complexImg, complexImg); cv::split(complexImg, planes);
        planes[0] = planes[0](cv::Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
        planes[1] = planes[1](cv::Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));
        Recomb(planes[0], planes[0]); Recomb(planes[1], planes[1]);
        planes[0] /= float(M * N); planes[1] /= float(M * N);
        FImg[0] = planes[0].clone(); FImg[1] = planes[1].clone();
    }

    void iris::highpass(cv::Size sz, cv::Mat& dst) {
        cv::Mat a(sz.height, 1, CV_32F), b(1, sz.width, CV_32F);
        float sy = CV_PI / sz.height, vy = -CV_PI * 0.5f;
        for (int i = 0; i < sz.height; ++i) { a.at<float>(i) = std::cos(vy); vy += sy; }
        float sx = CV_PI / sz.width, vx = -CV_PI * 0.5f;
        for (int i = 0; i < sz.width; ++i) { b.at<float>(i) = std::cos(vx); vx += sx; }
        cv::Mat tmp = a * b;
        dst = (1.0f - tmp).mul(2.0f - tmp);
    }

    float iris::logpolar(cv::Mat& src, cv::Mat& dst) {
        float radii = src.cols, angles = src.rows;
        cv::Point2f center(src.cols / 2.0f, src.rows / 2.0f);
        float d = cv::norm(cv::Vec2f(src.cols - center.x, src.rows - center.y));
        float log_base = std::pow(10.0f, std::log10(d) / radii);
        float d_theta = CV_PI / angles, theta = CV_PI / 2.0f;
        cv::Mat map_x(src.size(), CV_32FC1), map_y(src.size(), CV_32FC1);
        for (int i = 0; i < angles; ++i) {
            for (int j = 0; j < radii; ++j) {
                float r = std::pow(log_base, static_cast<float>(j));
                map_x.at<float>(i, j) = r * std::sin(theta) + center.x;
                map_y.at<float>(i, j) = r * std::cos(theta) + center.y;
            }
            theta += d_theta;
        }
        cv::remap(src, dst, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        return log_base;
    }

    cv::RotatedRect iris::FFTMatch(const cv::Mat& im0_in, const cv::Mat& im1_in) {
        cv::Mat im0, im1;
        im0_in.convertTo(im0, CV_32FC1, 1.0 / 255.0);
        im1_in.convertTo(im1, CV_32FC1, 1.0 / 255.0);

        cv::Mat F0[2], F1[2], f0, f1;
        ForwardFFT(im0, F0); ForwardFFT(im1, F1);
        cv::magnitude(F0[0], F0[1], f0); cv::magnitude(F1[0], F1[1], f1);

        cv::Mat h; highpass(f0.size(), h);
        f0 = f0.mul(h); f1 = f1.mul(h);

        cv::Mat f0lp, f1lp;
        float log_base = logpolar(f0, f0lp);
        logpolar(f1, f1lp);

        cv::Point2d rot_scale = cv::phaseCorrelate(f1lp, f0lp);
        float angle = 180.0 * rot_scale.y / f0lp.rows;
        float scale = std::pow(log_base, static_cast<float>(rot_scale.x));

        if (scale > 1.8f) {
            rot_scale = cv::phaseCorrelate(f1lp, f0lp);
            angle = -180.0 * rot_scale.y / f0lp.rows;
            scale = 1.0f / std::pow(log_base, static_cast<float>(rot_scale.x));
            if (scale > 1.8f) return cv::RotatedRect();
        }

        if (angle < -90.0) angle += 180.0; else if (angle > 90.0) angle -= 180.0;

        cv::Mat rot = cv::getRotationMatrix2D(cv::Point(im1.cols / 2, im1.rows / 2), angle, 1.0 / scale);
        cv::Mat im1_rs; cv::warpAffine(im1, im1_rs, rot, im1.size());
        cv::Point2d tr = cv::phaseCorrelate(im1_rs, im0);

        cv::RotatedRect rr;
        rr.center = tr + cv::Point2d(im0.cols / 2.0, im0.rows / 2.0);
        rr.angle = -angle;
        rr.size.width = im1.cols / scale;
        rr.size.height = im1.rows / scale;
        return rr;
    }

    cv::Mat1b iris::GetIris(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
        cv::Mat1b IrisMap = cv::Mat1b::zeros(iris_rows, iris_cols); 
        for (const auto& p : cloud.points) {
            float dis = std::sqrt(p.x * p.x + p.y * p.y);
            if (dis > max_distance || p.z > max_height || p.z < min_height) continue;
            
            float yaw = (std::atan2(p.y, p.x) * 180.0f / M_PI) + 180.0f;
            int Q_dis = std::min(std::max(static_cast<int>(std::floor(dis / dis_resolution)), 0), iris_rows - 1);
            int Q_yaw = std::min(std::max(static_cast<int>(std::floor(yaw + 0.5f)), 0), iris_cols - 1);
            int Q_h   = std::min(std::max(static_cast<int>(std::floor((p.z - min_height) / height_resolution)), 0), 7);
            IrisMap.at<uint8_t>(Q_dis, Q_yaw) |= (1 << Q_h);
        }
        return IrisMap;
    }

    std::vector<uint8_t> iris::IrisToBinaryVec(const cv::Mat1b &src) {
        std::vector<uint8_t> vec;
        vec.reserve(iris_binary_vec_size);

        for (int r = 0; r < src.rows; ++r) {
            int occupied_cols = 0;
            for (int c = 0; c < src.cols; ++c) {
                if (src.at<uint8_t>(r, c) > 0) occupied_cols++;
            }
            vec.push_back(occupied_cols >= 3 ? 1 : 0);
        }

        const int sector_width = std::max(1, src.cols / iris_sector_bins);
        for (int s = 0; s < iris_sector_bins; ++s) {
            int occupied_cells = 0;
            const int c_begin = s * sector_width;
            const int c_end = (s == iris_sector_bins - 1) ? src.cols : std::min(src.cols, c_begin + sector_width);
            for (int r = 0; r < src.rows; ++r) {
                for (int c = c_begin; c < c_end; ++c) {
                    if (src.at<uint8_t>(r, c) > 0) occupied_cells++;
                }
            }
            vec.push_back(occupied_cells >= 2 ? 1 : 0);
        }

        for (int h = 0; h < iris_height_bins; ++h) {
            int occupied_cells = 0;
            const uint8_t mask = static_cast<uint8_t>(1u << h);
            for (int r = 0; r < src.rows; ++r) {
                for (int c = 0; c < src.cols; ++c) {
                    if ((src.at<uint8_t>(r, c) & mask) != 0) occupied_cells++;
                }
            }
            vec.push_back(occupied_cells >= 3 ? 1 : 0);
        }

        for (int r = 0; r < src.rows; ++r) {
            for (int s = 0; s < iris_sector_bins; ++s) {
                int occupied_cells = 0;
                const int c_begin = s * sector_width;
                const int c_end = (s == iris_sector_bins - 1) ? src.cols : std::min(src.cols, c_begin + sector_width);
                for (int c = c_begin; c < c_end; ++c) {
                    if (src.at<uint8_t>(r, c) > 0) occupied_cells++;
                }
                vec.push_back(occupied_cells >= 1 ? 1 : 0);
            }
        }
        return vec;
    }

    iris::FeatureDesc iris::GetFeature(const cv::Mat1b &src) {
        FeatureDesc desc;
        desc.img = src;
        LoGFeatureEncode(src, desc.T, desc.M);
        return desc;
    }

    float iris::Compare(const iris::FeatureDesc &img1, const iris::FeatureDesc &img2, int *bias) {
        auto firstRect = FFTMatch(img2.img, img1.img);
        int firstShift = firstRect.center.x - img1.img.cols / 2;
        float dis1; int bias1;
        GetHammingDistance(img1.T, img1.M, img2.T, img2.M, firstShift, dis1, bias1);
        
        cv::Mat1b T2x = circShift(img2.T, 0, 180);
        cv::Mat1b M2x = circShift(img2.M, 0, 180);
        cv::Mat1b img2x = circShift(img2.img, 0, 180);
        
        auto secondRect = FFTMatch(img2x, img1.img);
        int secondShift = secondRect.center.x - img1.img.cols / 2;
        float dis2 = 0; int bias2 = 0;
        GetHammingDistance(img1.T, img1.M, T2x, M2x, secondShift, dis2, bias2);
        
        if (std::isnan(dis1) && std::isnan(dis2)) {
            if (bias) *bias = 0;
            return 1.0f; 
        }

        if (std::isnan(dis1)) {
            if (bias) *bias = (bias2 + 180) % 360;
            return dis2;
        }
        if (std::isnan(dis2)) {
            if (bias) *bias = bias1;
            return dis1;
        }

        if (dis1 < dis2) {
            if (bias) *bias = bias1;
            return dis1;
        } else {
            if (bias) *bias = (bias2 + 180) % 360;
            return dis2;
        }
    }

    std::vector<cv::Mat2f> iris::LogGaborFilter(const cv::Mat1f &src) {
        int rows = src.rows, cols = src.cols;
        cv::Mat2f filtersum = cv::Mat2f::zeros(1, cols);
        std::vector<cv::Mat2f> EO(nscale);
        int ndata = cols;
        if (ndata % 2 == 1) ndata--;
        cv::Mat1f logGabor = cv::Mat1f::zeros(1, ndata);
        cv::Mat2f result = cv::Mat2f::zeros(rows, ndata);
        cv::Mat1f radius = cv::Mat1f::zeros(1, ndata / 2 + 1);
        
        radius.at<float>(0, 0) = 1;
        for (int i = 1; i < ndata / 2 + 1; i++) {
            radius.at<float>(0, i) = i / static_cast<float>(ndata);
        }
        
        double wavelength = min_wavelength;
        for (int s = 0; s < nscale; s++) {
            double fo = 1.0 / wavelength;
            cv::Mat1f temp; 
            cv::log(radius / fo, temp);
            cv::pow(temp, 2, temp);
            cv::exp((-temp) / (2 * std::log(sigma_onf) * std::log(sigma_onf)), temp);
            temp.copyTo(logGabor.colRange(0, ndata / 2 + 1));
            logGabor.at<float>(0, 0) = 0;
            
            cv::Mat2f filter;
            cv::Mat1f filterArr[2] = {logGabor, cv::Mat1f::zeros(logGabor.size())};
            cv::merge(filterArr, 2, filter);
            filtersum = filtersum + filter;
            
            for (int r = 0; r < rows; r++) {
                cv::Mat2f src2f;
                cv::Mat1f srcArr[2] = {src.row(r).clone(), cv::Mat1f::zeros(1, src.cols)};
                cv::merge(srcArr, 2, src2f);
                cv::dft(src2f, src2f);
                cv::mulSpectrums(src2f, filter, src2f, 0);
                cv::idft(src2f, src2f);
                src2f.copyTo(result.row(r));
            }
            EO[s] = result.clone();
            wavelength *= mult;
        }
        filtersum = circShift(filtersum, 0, cols / 2);
        return EO;
    }

    void iris::LoGFeatureEncode(const cv::Mat1b &src, cv::Mat1b &T, cv::Mat1b &M) {
        cv::Mat1f srcFloat;
        src.convertTo(srcFloat, CV_32FC1);
        auto list = LogGaborFilter(srcFloat);
        std::vector<cv::Mat1b> Tlist(nscale * 2), Mlist(nscale * 2);
        
        for (size_t i = 0; i < list.size(); i++) {
            cv::Mat1f arr[2];
            cv::split(list[i], arr);
            Tlist[i] = arr[0] > 0;
            Tlist[i + nscale] = arr[1] > 0;
            cv::Mat1f m;
            cv::magnitude(arr[0], arr[1], m);
            Mlist[i] = m < 0.0001;
            Mlist[i + nscale] = m < 0.0001;
        }
        cv::vconcat(Tlist, T);
        cv::vconcat(Mlist, M);
    }

    void iris::GetHammingDistance(const cv::Mat1b &T1, const cv::Mat1b &M1, const cv::Mat1b &T2, const cv::Mat1b &M2, int scale, float &dis, int &bias) {
        dis = NAN;
        bias = -1;
        for (int shift = scale - 2; shift <= scale + 2; shift++) {
            cv::Mat1b T1s = circShift(T1, 0, shift);
            cv::Mat1b M1s = circShift(M1, 0, shift);
            cv::Mat1b mask = M1s | M2;
            int MaskBitsNum = cv::sum(mask / 255)[0];
            int totalBits = T1s.rows * T1s.cols - MaskBitsNum;
            
            cv::Mat1b C = T1s ^ T2;
            C = C & ~mask;
            int bitsDiff = cv::sum(C / 255)[0];
            
            if (totalBits != 0) {
                float currentDis = bitsDiff / static_cast<float>(totalBits);
                if (std::isnan(dis) || currentDis < dis) {
                    dis = currentDis;
                    bias = shift;
                }
            }
        }
    }

    cv::Mat iris::circShift(const cv::Mat &src, int shift_m_rows, int shift_n_cols) {
        cv::Mat res = src.clone();
        if(shift_m_rows != 0) {
            shift_m_rows %= src.rows; if(shift_m_rows < 0) shift_m_rows += src.rows;
            cv::Mat dst(src.size(), src.type());
            src(cv::Range(src.rows - shift_m_rows, src.rows), cv::Range::all()).copyTo(dst(cv::Range(0, shift_m_rows), cv::Range::all()));
            src(cv::Range(0, src.rows - shift_m_rows), cv::Range::all()).copyTo(dst(cv::Range(shift_m_rows, src.rows), cv::Range::all()));
            res = dst;
        }
        if(shift_n_cols != 0) {
            shift_n_cols %= src.cols; if(shift_n_cols < 0) shift_n_cols += src.cols;
            cv::Mat dst(src.size(), src.type());
            res(cv::Range::all(), cv::Range(src.cols - shift_n_cols, src.cols)).copyTo(dst(cv::Range::all(), cv::Range(0, shift_n_cols)));
            res(cv::Range::all(), cv::Range(0, src.cols - shift_n_cols)).copyTo(dst(cv::Range::all(), cv::Range(shift_n_cols, src.cols)));
            res = dst;
        }
        return res;
    }

} // namespace relocation
