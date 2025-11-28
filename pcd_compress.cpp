// pcd_compress.cpp
#include "PointCloudCompress.h"
#include <iostream>

int main(int argc, char* argv[]) {
    std::locale::global(std::locale("en_US.UTF-8"));

    if (argc < 4) {
        std::cout << "Usage:" << std::endl;
        std::cout << "Compress folder: " << argv[0] << " /compresspcdall <input_folder> <output_folder> <remove_duplicates(0/1)>" << std::endl;
        std::cout << "Decompress folder: " << argv[0] << " /uncompresspcdall <input_folder> <output_folder>" << std::endl;
        return 1;
    }

    PointCloudCompressor compressor;
    
    if (strcmp(argv[1], "/all") == 0) {
        bool remove_duplicates = (argc > 4 && std::stoi(argv[4]) != 0);
        if (compressor.CompressFolder(argv[2], argv[3], remove_duplicates) == 0) {
            std::cout << "Compression completed successfully" << std::endl;
            return 0;
        } else {
            std::cerr << "Compression failed" << std::endl;
            return 1;
        }
    }
    else if (strcmp(argv[1], "/unall") == 0) {
        if (compressor.DecompressFolder(argv[2], argv[3]) == 0) {
            std::cout << "Decompression completed successfully" << std::endl;
            return 0;
        } else {
            std::cerr << "Decompression failed" << std::endl;
            return 1;
        }
    }
    else {
        std::cerr << "Unknown command: " << argv[1] << std::endl;
        return 1;
    }
}