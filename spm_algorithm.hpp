#ifndef SPM_ALGORITHM_HPP
#define SPM_ALGORITHM_HPP

#include <iostream>
#include <cmath>
#include <utility>
#include <numeric>
#include <vector>

#include "opencv2/opencv.hpp"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/ply_io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/cloud_viewer.h"


class SpmAlgorithm {
private:
    SpmAlgorithm() = default;

    ~SpmAlgorithm() = default;

public:
    /**
     * @brief 一阶拉平处理
     *
     * @param data The SPM image real data.
     * @return None
     */
    template<typename T>
    static void flattenFirst(T &data) {
        const int num_cols = (int) data[0].size();

        double sum_m_mu = 0.0;
        double x_mean = (num_cols - 1) / 2.0;
        for (int c = 0; c < num_cols; ++c) {
            sum_m_mu += std::pow(c - x_mean, 2);
        }

        for (auto &row : data) {
            double row_mean = std::accumulate(row.begin(), row.end(), 0.0) / num_cols;

            double sum_m_zi = 0.0;
            for (auto it = row.begin(); it != row.end(); ++it) {
                int c = (int) std::distance(row.begin(), it);
                sum_m_zi += (c - x_mean) * (*it - row_mean);
            }

            double m = sum_m_zi / sum_m_mu;
            double b = row_mean - m * x_mean;

            for (auto &element : row) {
                int c = (int) (&element - &row[0]);
                element -= (m * c + b);
            }
        }
    }

    /**
     * @brief 一阶拉平处理
     *
     * @param spm_image The SPM image.
     * @return None
     */
    static void flattenFirst(SpmImage &spm_image) {
        flattenFirst(spm_image.getRealData());
    }

    /**
     * @brief 将二维数组转为 OpenCV Mat 对象
     *
     * @param array 2D-array
     * @return mat object
     */
    template<typename T>
    static cv::Mat array2DToImage(const T &array) {
        int rows = (int) array.size();
        int cols = (int) array[0].size();

        cv::Mat image(rows, cols, CV_64FC1);

        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                image.at<double>(r, c) = array[r][c];
            }
        }

        return image;
    }

    /**
     * @brief 将 SPM Image 的 Real Data 转为 OpenCV Mat 对象
     *
     * @param spm_image The SPM image.
     * @return mat object
     */
    static cv::Mat spmImageToImage(SpmImage &spm_image) {
        return array2DToImage(spm_image.getRealData());
    }

    /**
     * @brief 将二维数组转为 PCL Point Cloud 对象
     *
     * @param array 2D-array
     * @return point cloud shared ptr
     */
    template<typename T>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr array2DToPointCloud(const T &array) {
        int rows = (int) array.size();
        int cols = (int) array[0].size();

        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                auto value = (float) array[r][c];

                pcl::PointXYZ point;
                point.x = static_cast<float>(c);
                point.y = static_cast<float>(rows - r - 1);
                point.z = value;

                point_cloud->push_back(point);
            }
        }

        return point_cloud;
    }

    /**
     * @brief 将 SPM Image 的 Real Data 转为 PCL Point Cloud 对象
     *
     * @param spm_image The SPM image.
     * @return point cloud shared ptr
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr spmImageToPointCloud(SpmImage &spm_image) {
        return array2DToPointCloud(spm_image.getRealData());
    }

    /**
     * @brief 将 SPM Image 的 Real Data 保存为 BMP、JPG、PNG 等图像文件
     *
     * @param spm_image The SPM image.
     * @param file_path The file path to save the SPM image.
     * @return None
     */
    static void saveSpmImageToImage(SpmImage &spm_image, const std::string &file_path) {
        cv::Mat image = array2DToImage(spm_image.getRealData());
        cv::normalize(image, image, 0, 255, cv::NORM_MINMAX);

        cv::imwrite(file_path, image);
    }

    /**
     * @brief 将 SPM Image 的 Real Data 保存为 PLY 点云文件
     *
     * @param spm_image The SPM image.
     * @param file_path The file path to save the SPM image. (end with ".ply")
     * @param type The type of store data. 0 for binary, 1 for ascii.
     * @param zoom Scaling factor in the z-axis of the SPM image. 0 for auto.
     * @return None
     */
    static void
    saveSpmImageToPly(SpmImage &spm_image, const std::string &file_path, int type = 0, double zoom = 1) {
        if (file_path.substr(file_path.find_last_of('.') + 1) != "ply") {
            std::cout << "saveSpmImageToPly() Error: File path must end with '.ply'." << std::endl;
            return;
        }

        auto &real_data = spm_image.getRealData();

        // copy read data to 1D array
        std::vector<double> real_data_array;
        for (const auto &row : real_data) {
            real_data_array.insert(real_data_array.end(), row.begin(), row.end());
        }

        // calc auto zoom
        if (zoom == 0) {
            double max_value = *std::max_element(real_data_array.begin(), real_data_array.end());
            double min_value = *std::min_element(real_data_array.begin(), real_data_array.end());

            zoom = spm_image.getScanSize() / ((max_value - min_value) * 4);  // The units are nm
        }

        auto cloud = arrayToPointCloud(real_data_array, spm_image.getRows(), spm_image.getCols(),
                                       spm_image.getScanSize(), zoom);

        if (type == 0) {
            if (zoom != 1) {
                std::string zoom_file_path =
                        file_path.substr(0, file_path.size() - 4) + "_zoom-" + doubleToDecimalString(zoom, 3) +
                        "_bin.ply";
                pcl::io::savePLYFileBinary(zoom_file_path, *cloud);
            } else {
                pcl::io::savePLYFileBinary(file_path, *cloud);
            }
        } else if (type == 1) {
            if (zoom != 1) {
                std::string zoom_file_path =
                        file_path.substr(0, file_path.size() - 4) + "_zoom-" + doubleToDecimalString(zoom, 3) +
                        "_ascii.ply";
                pcl::io::savePLYFileASCII(zoom_file_path, *cloud);
            } else {
                pcl::io::savePLYFileASCII(file_path, *cloud);
            }
        }
    }

    /**
     * @brief 将 SPM Image 的 Real Data 保存为 PCD 点云文件
     *
     * @param spm_image The SPM image.
     * @param file_path The file path to save the SPM image. (end with ".pcd")
     * @param type The type of store data. 0 for binary, 1 for ascii.
     * @param zoom Scaling factor in the z-axis of the SPM image. 0 for auto.
     * @return None
     */
    static void
    saveSpmImageToPcd(SpmImage &spm_image, const std::string &file_path, int type = 0, double zoom = 1) {
        if (file_path.substr(file_path.find_last_of('.') + 1) != "pcd") {
            std::cout << "saveSpmImageToPcd() Error: File path must end with '.pcd'." << std::endl;
            return;
        }

        auto &real_data = spm_image.getRealData();

        // copy read data to 1D array
        std::vector<double> real_data_array;
        for (const auto &row : real_data) {
            real_data_array.insert(real_data_array.end(), row.begin(), row.end());
        }

        // calc auto zoom
        if (zoom == 0) {
            double max_value = *std::max_element(real_data_array.begin(), real_data_array.end());
            double min_value = *std::min_element(real_data_array.begin(), real_data_array.end());

            zoom = spm_image.getScanSize() / ((max_value - min_value) * 4);  // The units are nm
        }

        auto cloud = arrayToPointCloud(real_data_array, spm_image.getRows(), spm_image.getCols(),
                                       spm_image.getScanSize(), zoom);

        if (type == 0) {
            if (zoom != 1) {
                std::string zoom_file_path =
                        file_path.substr(0, file_path.size() - 4) + "_zoom-" + doubleToDecimalString(zoom, 3) +
                        "_bin.pcd";
                pcl::io::savePCDFileBinary(zoom_file_path, *cloud);
            } else {
                pcl::io::savePCDFileBinary(file_path, *cloud);
            }
        } else if (type == 1) {
            if (zoom != 1) {
                std::string zoom_file_path =
                        file_path.substr(0, file_path.size() - 4) + "_zoom-" + doubleToDecimalString(zoom, 3) +
                        "_ascii.pcd";
                pcl::io::savePCDFileASCII(zoom_file_path, *cloud);
            } else {
                pcl::io::savePCDFileASCII(file_path, *cloud);
            }
        }
    }

private:
    /**
     * @brief 将一维数组转为 PCL Point Cloud 对象
     *
     * @param array The 1D data array.
     * @param rows The number of rows in the array.
     * @param cols The number of columns in the array.
     * @param scan_size Scan size of spm image.
     * @param multi_k The coefficient to be multiplied by.
     * @return point cloud shared ptr
     */
    template<typename T>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr
    arrayToPointCloud(const T &array, int rows, int cols, int scan_size, double multi_k) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        double xy_k = scan_size / rows;
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                auto value = (float) array[r * cols + c];

                pcl::PointXYZ point;
                point.x = static_cast<float>(c * xy_k);
                point.y = static_cast<float>((rows - r - 1) * xy_k);
                point.z = value * multi_k;

                point_cloud->push_back(point);
            }
        }

        return point_cloud;
    }

    static std::string doubleToDecimalString(double value, int decimal_num) {
        double temp = value;
        int digits = 0;
        int sign_bit = value >= 0 ? 1 : -1;

        if (sign_bit == 1) {
            while (temp >= sign_bit) {
                digits += 1;
                temp /= 10;
            }

            if (digits == 0) digits = 1;
        } else {
            digits += 1;

            while (temp <= sign_bit) {
                digits += 1;
                temp /= 10;
            }

            if (digits == 1) digits = 2;
        }

        return std::to_string(value).substr(0, digits + decimal_num + 1);
    }
};


#endif //SPM_ALGORITHM_HPP
