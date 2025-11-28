// public_tools.cpp
#include "public_tools.h"

void FileUtils::GetAllFormatFiles(const std::string& path, 
                                 std::vector<std::string>& files, 
                                 const std::string& format) {
    try {
        for (const auto& entry : fs::recursive_directory_iterator(path)) {
            if (entry.is_regular_file() && entry.path().extension() == format) {
                files.push_back(entry.path().string());
            }
        }
    } catch (const fs::filesystem_error& ex) {
        std::cerr << "File system error: " << ex.what() << std::endl;
    }
}

void FileUtils::GetAllFormatFiles(const std::string& path,
                                 std::vector<std::string>& files_path,
                                 std::vector<std::string>& files_name,
                                 const std::string& format) {
    try {
        for (const auto& entry : fs::recursive_directory_iterator(path)) {
            if (entry.is_regular_file() && entry.path().extension() == format) {
                files_path.push_back(entry.path().string());
                files_name.push_back(entry.path().stem().string());
            }
        }
    } catch (const fs::filesystem_error& ex) {
        std::cerr << "File system error: " << ex.what() << std::endl;
    }
}

int64_t FileUtils::GetFileSize(FILE* file) {
    if (!file) return -1;
    
    #ifdef _WIN32
        return _filelength(_fileno(file));
    #else
        long current = ftell(file);
        fseek(file, 0, SEEK_END);
        long size = ftell(file);
        fseek(file, current, SEEK_SET);
        return size;
    #endif
}

void PointCloudUtils::CubeCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                                pcl::PointCloud<pcl::PointXYZ>& cloud_out,
                                const Eigen::Vector3f& min,
                                const Eigen::Vector3f& max) {
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setMin(Eigen::Vector4f(min(0), min(1), min(2), 1.0));
    crop.setMax(Eigen::Vector4f(max(0), max(1), max(2), 1.0));
    crop.setInputCloud(cloud_in.makeShared());
    crop.setNegative(false);
    crop.filter(cloud_out);
}

void PointCloudUtils::CubeCloudRGB( const pcl::PointCloud<pcl::PointXYZRGB>& cloud_in,
                                    pcl::PointCloud<pcl::PointXYZRGB>& cloud_out,
                                    const Eigen::Vector3f& min,
                                    const Eigen::Vector3f& max) {
    pcl::CropBox<pcl::PointXYZRGB> crop;
    crop.setMin(Eigen::Vector4f(min(0), min(1), min(2), 1.0));
    crop.setMax(Eigen::Vector4f(max(0), max(1), max(2), 1.0));
    crop.setInputCloud(cloud_in.makeShared());
    crop.setNegative(false);
    crop.filter(cloud_out);
}