// public_tools.h
#pragma once
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <memory>
#include <filesystem>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>

namespace fs = std::filesystem;

class FileUtils {
public:
    static int64_t GetFileSize(FILE* file);
    static FILE* CrossPlatformFopen(const std::string& filename, const std::string& mode);
    static void GetAllFormatFiles(  const std::string& path, 
                                    std::vector<std::string>& files, 
                                    const std::string& format);    
    static void GetAllFormatFiles(  const std::string& path,
                                    std::vector<std::string>& files_path,
                                    std::vector<std::string>& files_name,
                                    const std::string& format); 
};

class PointCloudUtils {
public:
    static void CubeCloud(  const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                            pcl::PointCloud<pcl::PointXYZ>& cloud_out,
                            const Eigen::Vector3f& min,
                            const Eigen::Vector3f& max);
    static void CubeCloudRGB(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_in,
                            pcl::PointCloud<pcl::PointXYZRGB>& cloud_out,
                            const Eigen::Vector3f& min,
                            const Eigen::Vector3f& max);
};