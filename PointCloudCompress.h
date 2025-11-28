// PointCloudCompress.h
#pragma once
#include "public_tools.h"
#include "TimeTool.h"
#include <yaml-cpp/yaml.h>

class PointCloudCompressor {
private:
    struct CompressionConfig {
        Eigen::Vector3f min_bound;
        Eigen::Vector3f max_bound;
        uint16_t lenX, lenY, lenZ;
        
        bool LoadFromYAML(const std::string& config_file);
    };
    
    struct BitWriter {
        std::vector<unsigned char> buffer;
        unsigned char current_byte = 0;
        int bit_position = 0;
        
        void WriteBits(uint16_t value, int bits);
        void Flush();
        void Clear();
    };
    
    struct BitReader {
        const unsigned char* buffer;
        size_t buffer_size;
        size_t byte_position = 0;
        int bit_position = 0;
        
        BitReader(const unsigned char* data, size_t size);
        uint16_t ReadBits(int bits);
        bool CanRead(int bits) const;
    };
    
public:
    bool CompressSingleFile(const std::string& input_file, 
                           const std::string& output_file, 
                           bool remove_duplicates);
                           
    bool CompressFolder(const std::string& input_folder,
                       const std::string& output_folder,
                       bool remove_duplicates);
                       
    bool DecompressSingleFile(const std::string& input_file,
                             const std::string& output_file);
                             
    bool DecompressFolder(const std::string& input_folder,
                         const std::string& output_folder);

private:
    bool WriteCompressedHeader(FILE* file, const CompressionConfig& config);
    bool ReadCompressedHeader(FILE* file, CompressionConfig& config);
    int CalculateRequiredBits(uint16_t value);
    void CompressPoint(const pcl::PointXYZ& point, const CompressionConfig& config, 
                      BitWriter& writer, bool remove_duplicates, pcl::PointXYZ& last_point);
    pcl::PointXYZ DecompressPoint(BitReader& reader, const CompressionConfig& config);
};