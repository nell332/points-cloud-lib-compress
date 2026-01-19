#include "PointCloudCompress.h"
#include "TimeTool.h"
#include "yaml-cpp/yaml.h"


bool PointCloudCompressor::CompressSingleFile(	
	const std::string& input_file, 
    const std::string& output_file, 
    bool remove_duplicates
) {
	Timer::Start();

	pcl::PointXYZ points;
	pcl::PointCloud<pcl::PointXYZ> cloud_raw_in;
	pcl::PointCloud<pcl::PointXYZ> cloud_raw_out;
	pcl::PointCloud<pcl::PointXYZ> cloud_raw;
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, cloud_raw_in)) {
		std::cout << "ERROR: Cannot open pcd file " << std::endl;
		return -1;
	}

	YAML::Node config;
	config = YAML::LoadFile("pcd_compress.yaml");
	if (config.IsNull()) {
		std::cout << "ERROR: Cannot open config file " << std::endl;
		return -1;
	}

	FILE* compressFile = FileUtils::CrossPlatformFopen(output_file, "w+b");
	if (compressFile == NULL) {
		std::cout << "ERROR: Cannot open compress file " << std::endl;
		return -1;
	}

	Eigen::Vector3f	central_min;
	Eigen::Vector3f	central_max;
	central_min[0] = config["X_MIN"].as<float>();
	central_min[1] = config["Y_MIN"].as<float>();
	central_min[2] = config["Z_MIN"].as<float>();
	central_max[0] = config["X_MAX"].as<float>();
	central_max[1] = config["Y_MAX"].as<float>();
	central_max[2] = config["Z_MAX"].as<float>();

	PointCloudUtils::CubeCloud(cloud_raw_in, cloud_raw, central_min, central_max);

	uint16_t lenX = abs((central_max[0] - central_min[0]) * 100);
	uint16_t lenY = abs((central_max[1] - central_min[1]) * 100);
	uint16_t lenZ = abs((central_max[2] - central_min[2]) * 100);

	fwrite((char*)(&lenX), sizeof(lenX), 1, compressFile);
	fwrite((char*)(&lenY), sizeof(lenY), 1, compressFile);
	fwrite((char*)(&lenZ), sizeof(lenZ), 1, compressFile);

	uint16_t X		= 0;
	uint16_t L_X	= 0;
	uint16_t Y		= 0;
	uint16_t L_Y	= 0;
	uint16_t Z		= 0;
	uint16_t L_Z	= 0;
	uint16_t T		= 0;
	uint64_t numP	= 0;
	unsigned char W = 0;
	unsigned char N = 0;

	unsigned int  lenToWrite  = 0;
	unsigned char *bitToWrite = new unsigned char[9];

	unsigned char compressTail		= 0;
	unsigned char *bitToWriteTail = new unsigned char[2];
	memset(bitToWriteTail, 0x00, 2);

	for (unsigned int i = 0; i < cloud_raw.size(); i++) {
		memset(bitToWrite, 0x00, 8);
		lenToWrite = 0;

		X = abs((cloud_raw.points[i].x - central_min[0]) * 100);
		Y = abs((cloud_raw.points[i].y - central_min[1]) * 100);
		Z = abs((cloud_raw.points[i].z - central_min[2]) * 100);

		if (i == 0) {
			L_X = X;
			L_Y = Y;
			L_Z = Z;
		}
		else {
			if (L_X == X && L_Y == Y && L_Z == Z && remove_duplicates == true) {
				continue;
			}
			else {
				L_X = X;
				L_Y = Y;
				L_Z = Z;
			}
		}

		points.x = X;
		points.x /= 100;
		points.x -= central_min[0];
		points.y = Y;
		points.y /= 100;
		points.y -= central_min[1];
		points.z = Z;
		points.z /= 100;
		points.z -= central_min[2];
		cloud_raw_out.push_back(points);

		if		(lenX <  256) {
			T							= X;
			bitToWrite[lenToWrite / 8]	= (unsigned char)T;
			lenToWrite				   += 8;
		}
		else if (256  <= lenX && lenX < 512) {
			T							= X;
			T							= T >> 1;
			bitToWrite[lenToWrite / 8]	= (unsigned char)T;
			lenToWrite				   += 8;
			
			T							= X;
			T							= T << 15;
			T							= T >> 8;
			bitToWrite[lenToWrite / 8]	= (unsigned char)T;
			lenToWrite				   += 1;
		}
		else if (512  <= lenX && lenX < 1024) {
			T							= X;
			T							= T >> 2;
			bitToWrite[lenToWrite / 8]	= (unsigned char)T;
			lenToWrite				   += 8;
			
			T							= X;
			T							= T << 14;
			T							= T >> 8;
			bitToWrite[lenToWrite / 8]	= (unsigned char)T;
			lenToWrite				   += 2;
		}
		else if (1024 <= lenX && lenX < 2048) {
			T							= X;
			T							= T >> 3;
			bitToWrite[lenToWrite / 8]	= (unsigned char)T;
			lenToWrite				   += 8;
			
			T							= X;
			T							= T << 13;
			T							= T >> 8;
			bitToWrite[lenToWrite / 8]	= (unsigned char)T;
			lenToWrite				   += 3;
		}
		else if (2048 <= lenX && lenX < 4096) {
			T							= X;
			T							= T >> 4;
			bitToWrite[lenToWrite / 8]	= (unsigned char)T;
			lenToWrite				   += 8;
			
			T							= X;
			T							= T << 12;
			T							= T >> 8;
			bitToWrite[lenToWrite / 8]	= (unsigned char)T;
			lenToWrite				   += 4;
		}
		else if (4096 <= lenX && lenX < 8192) {
			T							= X;
			T							= T >> 5;
			bitToWrite[lenToWrite / 8]	= (unsigned char)T;
			lenToWrite				   += 8;
			
			T							= X;
			T							= T << 11;
			T							= T >> 8;
			bitToWrite[lenToWrite / 8]	= (unsigned char)T;
			lenToWrite				   += 5;
		}
		else if (8192 <= lenX && lenX < 16384) {
			T							= X;
			T							= T >> 6;
			bitToWrite[lenToWrite / 8]	= (unsigned char)T;
			lenToWrite				   += 8;
			
			T							= X;
			T							= T << 10;
			T							= T >> 8;
			bitToWrite[lenToWrite / 8]	= (unsigned char)T;
			lenToWrite				   += 6;
		}
		else if (16384 <= lenX && lenX < 32768) {
			T							= X;
			T							= T >> 7;
			bitToWrite[lenToWrite / 8]	= (unsigned char)T;
			lenToWrite				   += 8;
			
			T							= X;
			T							= T << 9;
			T							= T >> 8;
			bitToWrite[lenToWrite / 8]	= (unsigned char)T;
			lenToWrite				   += 7;
		}
		else if (32768 <= lenX && lenX < 65536) {
			T							= X;
			T							= T >> 8;
			bitToWrite[lenToWrite / 8]	= (unsigned char)T;
			lenToWrite				   += 8;
			
			T							= X;
			T							= T << 8;
			T							= T >> 8;
			bitToWrite[lenToWrite / 8]	= (unsigned char)T;
			lenToWrite				   += 8;
		}

		if		(lenY <  256) {
			switch (lenToWrite % 8) {
			case 0:
				T							= Y;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= W;
				lenToWrite				   += 8;
				break;
			case 1:
				T							= Y;
				T							= T >> 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;

				T							= Y;
				T							= T << 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;
				break;
			case 2:
				T							= Y;
				T							= T >> 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;

				T							= Y;
				T							= T << 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;
				break;
			case 3:
				T							= Y;
				T							= T >> 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Y;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			case 4:
				T							= Y;
				T							= T >> 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;

				T							= Y;
				T							= T << 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;
				break;
			case 5:
				T							= Y;
				T							= T >> 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;

				T							= Y;
				T							= T << 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;
				break;
			case 6:
				T							= Y;
				T							= T >> 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;

				T							= Y;
				T							= T << 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;
				break;
			case 7:
				T							= Y;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;

				T							= Y;
				T							= T << 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;
				break;
			default:
				break;
			}
		}
		else if (256  <= lenY && lenY < 512) {
			switch (lenToWrite % 8) {
			case 0:
				T							= Y;
				T							= T >> 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;
				break;
			case 1:
				T							= Y;
				T							= T >> 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;

				T							= Y;
				T							= T << 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;
				break;
			case 2:
				T							= Y;
				T							= T >> 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;

				T							= Y;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			case 3:
				T							= Y;
				T							= T >> 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Y;
				T							= T << 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;
				break;
			case 4:
				T							= Y;
				T							= T >> 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;

				T							= Y;
				T							= T << 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;
				break;
			case 5:
				T							= Y;
				T							= T >> 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;

				T							= Y;
				T							= T << 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;
				break;
			case 6:
				T							= Y;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;

				T							= Y;
				T							= T << 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;
				break;
			case 7:
				T							= Y;
				T							= T >> 8;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;

				T							= Y;
				T							= T << 0;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;
				break;
			default:
				break;
			}
		}
		else if (512  <= lenY && lenY < 1024) {
			switch (lenToWrite % 8) {
			case 0:
				T							= Y;
				T							= T >> 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;
				break;
			case 1:
				T							= Y;
				T							= T >> 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;

				T							= Y;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			case 2:
				T							= Y;
				T							= T >> 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;

				T							= Y;
				T							= T << 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;
				break;
			case 3:
				T							= Y;
				T							= T >> 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Y;
				T							= T << 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;
				break;
			case 4:
				T							= Y;
				T							= T >> 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;

				T							= Y;
				T							= T << 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;
				break;
			case 5:
				T							= Y;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;

				T							= Y;
				T							= T << 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;
				break;
			case 6:
				T							= Y;
				T							= T >> 8;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;

				T							= Y;
				T							= T << 0;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;
				break;
			case 7:
				T							= Y;
				T							= T >> 9;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;

				T							= Y;
				T							= T >> 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;
				break;
			default:
				break;
			}
		}
		else if (1024 <= lenY && lenY < 2048) {
			switch (lenToWrite % 8) {
			case 0:
				T							= Y;
				T							= T >> 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			case 1:
				T							= Y;
				T							= T >> 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;

				T							= Y;
				T							= T << 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;
				break;
			case 2:
				T							= Y;
				T							= T >> 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;

				T							= Y;
				T							= T << 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;
				break;
			case 3:
				T							= Y;
				T							= T >> 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Y;
				T							= T << 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;
				break;
			case 4:
				T							= Y;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;

				T							= Y;
				T							= T << 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;
				break;
			case 5:
				T							= Y;
				T							= T >> 8;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;

				T							= Y;
				T							= T << 0;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;
				break;
			case 6:
				T							= Y;
				T							= T >> 9;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;

				T							= Y;
				T							= T >> 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;
				break;
			case 7:
				T							= Y;
				T							= T >> 10;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;

				T							= Y;
				T							= T >> 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;
				break;
			default:
				break;
			}
		}
		else if (2048 <= lenY && lenY < 4096) {
			switch (lenToWrite % 8) {
			case 0:
				T							= Y;
				T							= T >> 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;
				break;
			case 1:
				T							= Y;
				T							= T >> 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;

				T							= Y;
				T							= T << 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;
				break;
			case 2:
				T							= Y;
				T							= T >> 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;

				T							= Y;
				T							= T << 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;
				break;
			case 3:
				T							= Y;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Y;
				T							= T << 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;
				break;
			case 4:
				T							= Y;
				T							= T >> 8;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;

				T							= Y;
				T							= T << 0;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;
				break;
			case 5:
				T							= Y;
				T							= T >> 9;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;

				T							= Y;
				T							= T >> 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;
				break;
			case 6:
				T							= Y;
				T							= T >> 10;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;

				T							= Y;
				T							= T >> 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;
				break;
			case 7:
				T							= Y;
				T							= T >> 11;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;

				T							= Y;
				T							= T >> 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			default:
				break;
			}
		}
		else if (4096 <= lenY && lenY < 8192) { // 13
			switch (lenToWrite % 8) {
			case 0:
				T							= Y;
				T							= T >> 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;
				break;
			case 1:
				T							= Y;
				T							= T >> 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;

				T							= Y;
				T							= T << 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;
				break;
			case 2:
				T							= Y;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Y;
				T							= T << 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;
				break;
			case 3:
				T							= Y;
				T							= T >> 8;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Y;
				T							= T << 0;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;
				break;
			case 4:
				T							= Y;
				T							= T >> 9;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;

				T							= Y;
				T							= T >> 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;
				break;
			case 5:
				T							= Y;
				T							= T >> 10;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;

				T							= Y;
				T							= T >> 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;
				break;
			case 6:
				T							= Y;
				T							= T >> 11;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;

				T							= Y;
				T							= T >> 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			case 7:
				T							= Y;
				T							= T >> 12;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;

				T							= Y;
				T							= T >> 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;
				break;
			default:
				break;
			}
		}
		else if (8192 <= lenY && lenY < 16384) { // 14
			switch (lenToWrite % 8) {
			case 0:
				T							= Y;
				T							= T >> 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;
				break;
			case 1:
				T							= Y;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;

				T							= Y;
				T							= T << 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;
				break;
			case 2:
				T							= Y;
				T							= T >> 8;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;

				T							= Y;
				T							= T << 0;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;
				break;
			case 3:
				T							= Y;
				T							= T >> 9;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Y;
				T							= T >> 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;
				break;
			case 4:
				T							= Y;
				T							= T >> 10;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;

				T							= Y;
				T							= T >> 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;
				break;
			case 5:
				T							= Y;
				T							= T >> 11;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;

				T							= Y;
				T							= T >> 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			case 6:
				T							= Y;
				T							= T >> 12;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;

				T							= Y;
				T							= T >> 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;
				break;
			case 7:
				T							= Y;
				T							= T >> 13;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;

				T							= Y;
				T							= T >> 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;
				break;
			default:
				break;
			}
		}
		else if (16384 <= lenY && lenY < 32768) { // 15
			switch (lenToWrite % 8) {
			case 0:
				T							= Y;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;
				break;
			case 1:
				T							= Y;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;

				T							= Y;
				T							= T << 0;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;
				break;
			case 2:
				T							= Y;
				T							= T >> 9;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;

				T							= Y;
				T							= T >> 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;
				break;
			case 3:
				T							= Y;
				T							= T >> 10;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Y;
				T							= T >> 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;
				break;
			case 4:
				T							= Y;
				T							= T >> 11;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;

				T							= Y;
				T							= T >> 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			case 5:
				T							= Y;
				T							= T >> 12;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;

				T							= Y;
				T							= T >> 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;
				break;
			case 6:
				T							= Y;
				T							= T >> 13;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;

				T							= Y;
				T							= T >> 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;
				break;
			case 7:
				T							= Y;
				T							= T >> 14;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;

				T							= Y;
				T							= T >> 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;
				break;
			default:
				break;
			}
		}
		else if (32768 <= lenY && lenY < 65536) { // 16
			switch (lenToWrite % 8) {
			case 0:
				T							= Y;
				T							= T >> 8;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 0;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;
				break;
			case 1:
				T							= Y;
				T							= T >> 8;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;

				T							= Y;
				T							= T >> 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;
				break;
			case 2:
				T							= Y;
				T							= T >> 10;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;

				T							= Y;
				T							= T >> 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;
				break;
			case 3:
				T							= Y;
				T							= T >> 11;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Y;
				T							= T >> 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			case 4:
				T							= Y;
				T							= T >> 12;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;

				T							= Y;
				T							= T >> 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;
				break;
			case 5:
				T							= Y;
				T							= T >> 13;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;

				T							= Y;
				T							= T >> 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;
				break;
			case 6:
				T							= Y;
				T							= T >> 14;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;

				T							= Y;
				T							= T >> 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;
				break;
			case 7:
				T							= Y;
				T							= T >> 15;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;

				T							= Y;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Y;
				T							= T << 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;
				break;
			default:
				break;
			}
		}

		if		(lenZ <  256) {
			switch (lenToWrite % 8) {
			case 0:
				T							= Z;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= W;
				lenToWrite				   += 8;
				break;
			case 1:
				T							= Z;
				T							= T >> 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;

				T							= Z;
				T							= T << 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;
				break;
			case 2:
				T							= Z;
				T							= T >> 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;

				T							= Z;
				T							= T << 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;
				break;
			case 3:
				T							= Z;
				T							= T >> 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Z;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			case 4:
				T							= Z;
				T							= T >> 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;

				T							= Z;
				T							= T << 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;
				break;
			case 5:
				T							= Z;
				T							= T >> 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;

				T							= Z;
				T							= T << 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;
				break;
			case 6:
				T							= Z;
				T							= T >> 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;

				T							= Z;
				T							= T << 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;
				break;
			case 7:
				T							= Z;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;

				T							= Z;
				T							= T << 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;
				break;
			default:
				break;
			}
		}
		else if (256  <= lenZ && lenZ < 512) { // 9
			switch (lenToWrite % 8) {
			case 0:
				T							= Z;
				T							= T >> 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;
				break;
			case 1:
				T							= Z;
				T							= T >> 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;

				T							= Z;
				T							= T << 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;
				break;
			case 2:
				T							= Z;
				T							= T >> 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;

				T							= Z;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			case 3:
				T							= Z;
				T							= T >> 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Z;
				T							= T << 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;
				break;
			case 4:
				T							= Z;
				T							= T >> 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;

				T							= Z;
				T							= T << 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;
				break;
			case 5:
				T							= Z;
				T							= T >> 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;

				T							= Z;
				T							= T << 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;
				break;
			case 6:
				T							= Z;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;

				T							= Z;
				T							= T << 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;
				break;
			case 7:
				T							= Z;
				T							= T >> 8;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;

				T							= Z;
				T							= T << 0;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;
				break;
			default:
				break;
			}
		}
		else if (512  <= lenZ && lenZ < 1024) {
			switch (lenToWrite % 8) {
			case 0:
				T							= Z;
				T							= T >> 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;
				break;
			case 1:
				T							= Z;
				T							= T >> 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;

				T							= Z;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			case 2:
				T							= Z;
				T							= T >> 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;

				T							= Z;
				T							= T << 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;
				break;
			case 3:
				T							= Z;
				T							= T >> 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Z;
				T							= T << 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;
				break;
			case 4:
				T							= Z;
				T							= T >> 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;

				T							= Z;
				T							= T << 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;
				break;
			case 5:
				T							= Z;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;

				T							= Z;
				T							= T << 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;
				break;
			case 6:
				T							= Z;
				T							= T >> 8;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;

				T							= Z;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;
				break;
			case 7:
				T							= Z;
				T							= T >> 9;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;

				T							= Z;
				T							= T >> 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;
				break;
			default:
				break;
			}
		}
		else if (1024 <= lenZ && lenZ < 2048) { // 11
			switch (lenToWrite % 8) {
			case 0:
				T							= Z;
				T							= T >> 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			case 1:
				T							= Z;
				T							= T >> 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;

				T							= Z;
				T							= T << 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;
				break;
			case 2:
				T							= Z;
				T							= T >> 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;

				T							= Z;
				T							= T << 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;
				break;
			case 3:
				T							= Z;
				T							= T >> 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Z;
				T							= T << 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;
				break;
			case 4:
				T							= Z;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;

				T							= Z;
				T							= T << 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;
				break;
			case 5:
				T							= Z;
				T							= T >> 8;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;

				T							= Z;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;
				break;
			case 6:
				T							= Z;
				T							= T >> 9;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;

				T							= Z;
				T							= T >> 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;
				break;
			case 7:
				T							= Z;
				T							= T >> 10;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;

				T							= Z;
				T							= T >> 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;
				break;
			default:
				break;
			}
		}
		else if (2048 <= lenZ && lenZ < 4096) { // 12
			switch (lenToWrite % 8) {
			case 0:
				T							= Z;
				T							= T >> 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;
				break;
			case 1:
				T							= Z;
				T							= T >> 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;

				T							= Z;
				T							= T << 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;
				break;
			case 2:
				T							= Z;
				T							= T >> 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;

				T							= Z;
				T							= T << 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;
				break;
			case 3:
				T							= Z;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Z;
				T							= T << 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;
				break;
			case 4:
				T							= Z;
				T							= T >> 8;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;

				T							= Z;
				T							= T << 0;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;
				break;
			case 5:
				T							= Z;
				T							= T >> 9;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;

				T							= Z;
				T							= T >> 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;
				break;
			case 6:
				T							= Z;
				T							= T >> 10;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;

				T							= Z;
				T							= T >> 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;
				break;
			case 7:
				T							= Z;
				T							= T >> 11;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;

				T							= Z;
				T							= T >> 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			default:
				break;
			}
		}
		else if (4096 <= lenZ && lenZ < 8192) { // 13
			switch (lenToWrite % 8) {
			case 0:
				T							= Z;
				T							= T >> 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;
				break;
			case 1:
				T							= Z;
				T							= T >> 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;

				T							= Z;
				T							= T << 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;
				break;
			case 2:
				T							= Z;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;

				T							= Z;
				T							= T << 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;
				break;
			case 3:
				T							= Z;
				T							= T >> 8;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Z;
				T							= T << 0;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;
				break;
			case 4:
				T							= Z;
				T							= T >> 9;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;

				T							= Z;
				T							= T >> 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;
				break;
			case 5:
				T							= Z;
				T							= T >> 10;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;

				T							= Z;
				T							= T >> 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;
				break;
			case 6:
				T							= Z;
				T							= T >> 11;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;

				T							= Z;
				T							= T >> 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			case 7:
				T							= Z;
				T							= T >> 12;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;

				T							= Z;
				T							= T >> 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;
				break;
			default:
				break;
			}
		}
		else if (8192 <= lenZ && lenZ < 16384) { // 14
			switch (lenToWrite % 8) {
			case 0:
				T							= Z;
				T							= T >> 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;
				break;
			case 1:
				T							= Z;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;

				T							= Z;
				T							= T << 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;
				break;
			case 2:
				T							= Z;
				T							= T >> 8;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;

				T							= Z;
				T							= T << 0;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;
				break;
			case 3:
				T							= Z;
				T							= T >> 9;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Z;
				T							= T >> 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;
				break;
			case 4:
				T							= Z;
				T							= T >> 10;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;

				T							= Z;
				T							= T >> 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;
				break;
			case 5:
				T							= Z;
				T							= T >> 11;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;

				T							= Z;
				T							= T >> 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			case 6:
				T							= Z;
				T							= T >> 12;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;

				T							= Z;
				T							= T >> 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;
				break;
			case 7:
				T							= Z;
				T							= T >> 13;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;

				T							= Z;
				T							= T >> 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;
				break;
			default:
				break;
			}
		}
		else if (16384 <= lenZ && lenZ < 32768) { // 15
			switch (lenToWrite % 8) {
			case 0:
				T							= Z;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;
				break;
			case 1:
				T							= Z;
				T							= T >> 8;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;

				T							= Z;
				T							= T << 0;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;
				break;
			case 2:
				T							= Z;
				T							= T >> 9;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;

				T							= Z;
				T							= T >> 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;
				break;
			case 3:
				T							= Z;
				T							= T >> 10;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Z;
				T							= T >> 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;
				break;
			case 4:
				T							= Z;
				T							= T >> 11;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;

				T							= Z;
				T							= T >> 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			case 5:
				T							= Z;
				T							= T >> 12;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;

				T							= Z;
				T							= T >> 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;
				break;
			case 6:
				T							= Z;
				T							= T >> 13;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;

				T							= Z;
				T							= T >> 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;
				break;
			case 7:
				T							= Z;
				T							= T >> 14;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;

				T							= Z;
				T							= T >> 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;
				break;
			default:
				break;
			}
		}
		else if (32768 <= lenZ && lenZ < 65536) { // 16
			switch (lenToWrite % 8) {
			case 0:
				T							= Z;
				T							= T >> 8;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 0;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;
				break;
			case 1:
				T							= Z;
				T							= T >> 9;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;

				T							= Z;
				T							= T >> 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;
				break;
			case 2:
				T							= Z;
				T							= T >> 10;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;

				T							= Z;
				T							= T >> 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;
				break;
			case 3:
				T							= Z;
				T							= T >> 11;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 5;

				T							= Z;
				T							= T >> 3;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			case 4:
				T							= Z;
				T							= T >> 12;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;

				T							= Z;
				T							= T >> 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 4;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 4;
				break;
			case 5:
				T							= Z;
				T							= T >> 13;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;

				T							= Z;
				T							= T >> 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 5;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 3;
				break;
			case 6:
				T							= Z;
				T							= T >> 14;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 2;

				T							= Z;
				T							= T >> 6;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 2;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 6;
				break;
			case 7:
				T							= Z;
				T							= T >> 15;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 1;

				T							= Z;
				T							= T >> 7;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 8;

				T							= Z;
				T							= T << 1;
				W							= (unsigned char)T;
				bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
				lenToWrite				   += 7;
				break;
			default:
				break;
			}
		}

		if (compressTail == 0) {
			fwrite(bitToWrite, sizeof(unsigned char), lenToWrite / 8, compressFile);

			compressTail	  = lenToWrite % 8;
			bitToWriteTail[0] = bitToWrite[lenToWrite / 8];
		}
		else {
			for (int x = 0; x < (lenToWrite + compressTail) / 8 + 1; x++) {
				T   = bitToWrite[x];
				T >>= compressTail;
				T   = bitToWriteTail[0] | T;

				N					= bitToWrite[x];
				N				  <<= 8 - compressTail;
				bitToWriteTail[0]	= N;

				bitToWrite[x] = (unsigned char)T;
			}

			fwrite(bitToWrite, sizeof(unsigned char), (lenToWrite + compressTail) / 8, compressFile);

			T					= bitToWrite[(lenToWrite + compressTail) / 8];
			compressTail		= (lenToWrite + compressTail) % 8;
			bitToWriteTail[0]	= (unsigned char)T;
		}
	}

	if (compressTail > 0) {
		fwrite(bitToWriteTail, sizeof(unsigned char), 1, compressFile);
	}

	delete[] bitToWrite;
	delete[] bitToWriteTail;
	cloud_raw.clear();
	cloud_raw.resize(0);
	cloud_raw_in.clear();
	cloud_raw_in.resize(0);
	cloud_raw_out.clear();
	cloud_raw_out.resize(0);

	fclose(compressFile);

	double timeInterval = Timer::Stop();
	printf("time past:%fs\n", timeInterval);

	return 0;
}

bool PointCloudCompressor::CompressFolder(
	const std::string& input_folder,
    const std::string& output_folder,
    bool remove_duplicates
) {
	Timer::Start();

	YAML::Node config;
	config = YAML::LoadFile("pcd_compress.yaml");
	if (config.IsNull()) {
		std::cout << "ERROR: Cannot open config file " << std::endl;
		return -1;
	}

	Eigen::Vector3f	central_min;
	Eigen::Vector3f	central_max;
	central_min[0] = config["X_MIN"].as<float>();
	central_min[1] = config["Y_MIN"].as<float>();
	central_min[2] = config["Z_MIN"].as<float>();
	central_max[0] = config["X_MAX"].as<float>();
	central_max[1] = config["Y_MAX"].as<float>();
	central_max[2] = config["Z_MAX"].as<float>();

	pcl::PointXYZ points;
	pcl::PointCloud<pcl::PointXYZ> cloud_raw_in;
	pcl::PointCloud<pcl::PointXYZ> cloud_raw_out;
	pcl::PointCloud<pcl::PointXYZ> cloud_raw;

	vector<string> pcdList;
	string format = ".pcd";
	FileUtils::GetAllFormatFiles(input_folder, pcdList, format);

	int size = pcdList.size();
	for (int i = 0; i < size; i++) {
		ifstream in(pcdList[i]);

		if (in) { // 有该文件  
			if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdList[i], cloud_raw_in)) {
				std::cout << "ERROR: Cannot open file " << std::endl;
				return -1;
			}

			string::size_type iPos = (	pcdList[i].find_last_of('\\') + 1) == 0 ? 
										pcdList[i].find_last_of('/') + 1 : pcdList[i].find_last_of('\\') + 1;
			fs::path pcdSavePath = output_folder;
			pcdSavePath /= pcdList[i].substr(iPos, pcdList[i].length() - iPos);
			std::string pcdSaveName = pcdSavePath.string();
			string pcdNameNTag = pcdSaveName.substr(0, pcdSaveName.rfind(".")) + ".mtx";

			FILE* compressFile = FileUtils::CrossPlatformFopen(pcdNameNTag, "w+b");
			if (compressFile == NULL) {
				std::cout << "ERROR: Cannot open compress file " << std::endl;
				return -1;
			}

			PointCloudUtils::CubeCloud(cloud_raw_in, cloud_raw, central_min, central_max);

			uint16_t lenX = abs((central_max[0] - central_min[0]) * 100);
			uint16_t lenY = abs((central_max[1] - central_min[1]) * 100);
			uint16_t lenZ = abs((central_max[2] - central_min[2]) * 100);

			fwrite((char*)(&lenX), sizeof(lenX), 1, compressFile);
			fwrite((char*)(&lenY), sizeof(lenY), 1, compressFile);
			fwrite((char*)(&lenZ), sizeof(lenZ), 1, compressFile);

			uint16_t X		= 0;
			uint16_t L_X	= 0;
			uint16_t Y		= 0;
			uint16_t L_Y	= 0;
			uint16_t Z		= 0;
			uint16_t L_Z	= 0;
			uint16_t T		= 0;
			uint64_t numP	= 0;
			unsigned char W = 0;
			unsigned char N = 0;

			unsigned int  lenToWrite  = 0;
			unsigned char *bitToWrite = new unsigned char[9];

			unsigned char compressTail	  = 0;
			unsigned char *bitToWriteTail = new unsigned char[2];
			memset(bitToWriteTail, 0x00, 2);

			for (unsigned int i = 0; i < cloud_raw.size(); i++) {
				memset(bitToWrite, 0x00, 8);
				lenToWrite = 0;

				X = abs((cloud_raw.points[i].x - central_min[0]) * 100);
				Y = abs((cloud_raw.points[i].y - central_min[1]) * 100);
				Z = abs((cloud_raw.points[i].z - central_min[2]) * 100);

				if (i == 0) {
					L_X = X;
					L_Y = Y;
					L_Z = Z;
				}
				else {
					if (L_X == X && L_Y == Y && L_Z == Z && remove_duplicates == true) {
						continue;
					}
					else {
						L_X = X;
						L_Y = Y;
						L_Z = Z;
					}
				}

				points.x = X;
				points.x /= 100;
				points.x -= central_min[0];
				points.y = Y;
				points.y /= 100;
				points.y -= central_min[1];
				points.z = Z;
				points.z /= 100;
				points.z -= central_min[2];
				cloud_raw_out.push_back(points);

				if		(lenX <  256) {
					T							= X;
					bitToWrite[lenToWrite / 8]	= (unsigned char)T;
					lenToWrite				   += 8;
				}
				else if (256  <= lenX && lenX < 512) {
					T							= X;
					T							= T >> 1;
					bitToWrite[lenToWrite / 8]	= (unsigned char)T;
					lenToWrite				   += 8;
			
					T							= X;
					T							= T << 15;
					T							= T >> 8;
					bitToWrite[lenToWrite / 8]	= (unsigned char)T;
					lenToWrite				   += 1;
				}
				else if (512  <= lenX && lenX < 1024) {
					T							= X;
					T							= T >> 2;
					bitToWrite[lenToWrite / 8]	= (unsigned char)T;
					lenToWrite				   += 8;
			
					T							= X;
					T							= T << 14;
					T							= T >> 8;
					bitToWrite[lenToWrite / 8]	= (unsigned char)T;
					lenToWrite				   += 2;
				}
				else if (1024 <= lenX && lenX < 2048) {
					T							= X;
					T							= T >> 3;
					bitToWrite[lenToWrite / 8]	= (unsigned char)T;
					lenToWrite				   += 8;
			
					T							= X;
					T							= T << 13;
					T							= T >> 8;
					bitToWrite[lenToWrite / 8]	= (unsigned char)T;
					lenToWrite				   += 3;
				}
				else if (2048 <= lenX && lenX < 4096) {
					T							= X;
					T							= T >> 4;
					bitToWrite[lenToWrite / 8]	= (unsigned char)T;
					lenToWrite				   += 8;
			
					T							= X;
					T							= T << 12;
					T							= T >> 8;
					bitToWrite[lenToWrite / 8]	= (unsigned char)T;
					lenToWrite				   += 4;
				}
				else if (4096 <= lenX && lenX < 8192) {
					T							= X;
					T							= T >> 5;
					bitToWrite[lenToWrite / 8]	= (unsigned char)T;
					lenToWrite				   += 8;
			
					T							= X;
					T							= T << 11;
					T							= T >> 8;
					bitToWrite[lenToWrite / 8]	= (unsigned char)T;
					lenToWrite				   += 5;
				}
				else if (8192 <= lenX && lenX < 16384) {
					T							= X;
					T							= T >> 6;
					bitToWrite[lenToWrite / 8]	= (unsigned char)T;
					lenToWrite				   += 8;
			
					T							= X;
					T							= T << 10;
					T							= T >> 8;
					bitToWrite[lenToWrite / 8]	= (unsigned char)T;
					lenToWrite				   += 6;
				}
				else if (16384 <= lenX && lenX < 32768) {
					T							= X;
					T							= T >> 7;
					bitToWrite[lenToWrite / 8]	= (unsigned char)T;
					lenToWrite				   += 8;
			
					T							= X;
					T							= T << 9;
					T							= T >> 8;
					bitToWrite[lenToWrite / 8]	= (unsigned char)T;
					lenToWrite				   += 7;
				}
				else if (32768 <= lenX && lenX < 65536) {
					T							= X;
					T							= T >> 8;
					bitToWrite[lenToWrite / 8]	= (unsigned char)T;
					lenToWrite				   += 8;
			
					T							= X;
					T							= T << 8;
					T							= T >> 8;
					bitToWrite[lenToWrite / 8]	= (unsigned char)T;
					lenToWrite				   += 8;
				}

				if		(lenY <  256) {
					switch (lenToWrite % 8) {
					case 0:
						T							= Y;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= W;
						lenToWrite				   += 8;
						break;
					case 1:
						T							= Y;
						T							= T >> 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;

						T							= Y;
						T							= T << 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;
						break;
					case 2:
						T							= Y;
						T							= T >> 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;

						T							= Y;
						T							= T << 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;
						break;
					case 3:
						T							= Y;
						T							= T >> 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Y;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					case 4:
						T							= Y;
						T							= T >> 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;

						T							= Y;
						T							= T << 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;
						break;
					case 5:
						T							= Y;
						T							= T >> 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;

						T							= Y;
						T							= T << 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;
						break;
					case 6:
						T							= Y;
						T							= T >> 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;

						T							= Y;
						T							= T << 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;
						break;
					case 7:
						T							= Y;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;

						T							= Y;
						T							= T << 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;
						break;
					default:
						break;
					}
				}
				else if (256  <= lenY && lenY < 512) {
					switch (lenToWrite % 8) {
					case 0:
						T							= Y;
						T							= T >> 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;
						break;
					case 1:
						T							= Y;
						T							= T >> 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;

						T							= Y;
						T							= T << 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;
						break;
					case 2:
						T							= Y;
						T							= T >> 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;

						T							= Y;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					case 3:
						T							= Y;
						T							= T >> 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Y;
						T							= T << 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;
						break;
					case 4:
						T							= Y;
						T							= T >> 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;

						T							= Y;
						T							= T << 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;
						break;
					case 5:
						T							= Y;
						T							= T >> 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;

						T							= Y;
						T							= T << 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;
						break;
					case 6:
						T							= Y;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;

						T							= Y;
						T							= T << 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;
						break;
					case 7:
						T							= Y;
						T							= T >> 8;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;

						T							= Y;
						T							= T << 0;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;
						break;
					default:
						break;
					}
				}
				else if (512  <= lenY && lenY < 1024) {
					switch (lenToWrite % 8) {
					case 0:
						T							= Y;
						T							= T >> 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;
						break;
					case 1:
						T							= Y;
						T							= T >> 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;

						T							= Y;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					case 2:
						T							= Y;
						T							= T >> 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;

						T							= Y;
						T							= T << 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;
						break;
					case 3:
						T							= Y;
						T							= T >> 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Y;
						T							= T << 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;
						break;
					case 4:
						T							= Y;
						T							= T >> 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;

						T							= Y;
						T							= T << 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;
						break;
					case 5:
						T							= Y;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;

						T							= Y;
						T							= T << 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;
						break;
					case 6:
						T							= Y;
						T							= T >> 8;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;

						T							= Y;
						T							= T << 0;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;
						break;
					case 7:
						T							= Y;
						T							= T >> 9;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;

						T							= Y;
						T							= T >> 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;
						break;
					default:
						break;
					}
				}
				else if (1024 <= lenY && lenY < 2048) {
					switch (lenToWrite % 8) {
					case 0:
						T							= Y;
						T							= T >> 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					case 1:
						T							= Y;
						T							= T >> 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;

						T							= Y;
						T							= T << 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;
						break;
					case 2:
						T							= Y;
						T							= T >> 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;

						T							= Y;
						T							= T << 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;
						break;
					case 3:
						T							= Y;
						T							= T >> 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Y;
						T							= T << 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;
						break;
					case 4:
						T							= Y;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;

						T							= Y;
						T							= T << 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;
						break;
					case 5:
						T							= Y;
						T							= T >> 8;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;

						T							= Y;
						T							= T << 0;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;
						break;
					case 6:
						T							= Y;
						T							= T >> 9;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;

						T							= Y;
						T							= T >> 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;
						break;
					case 7:
						T							= Y;
						T							= T >> 10;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;

						T							= Y;
						T							= T >> 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;
						break;
					default:
						break;
					}
				}
				else if (2048 <= lenY && lenY < 4096) {
					switch (lenToWrite % 8) {
					case 0:
						T							= Y;
						T							= T >> 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;
						break;
					case 1:
						T							= Y;
						T							= T >> 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;

						T							= Y;
						T							= T << 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;
						break;
					case 2:
						T							= Y;
						T							= T >> 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;

						T							= Y;
						T							= T << 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;
						break;
					case 3:
						T							= Y;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Y;
						T							= T << 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;
						break;
					case 4:
						T							= Y;
						T							= T >> 8;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;

						T							= Y;
						T							= T << 0;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;
						break;
					case 5:
						T							= Y;
						T							= T >> 9;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;

						T							= Y;
						T							= T >> 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;
						break;
					case 6:
						T							= Y;
						T							= T >> 10;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;

						T							= Y;
						T							= T >> 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;
						break;
					case 7:
						T							= Y;
						T							= T >> 11;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;

						T							= Y;
						T							= T >> 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					default:
						break;
					}
				}
				else if (4096 <= lenY && lenY < 8192) { // 13
					switch (lenToWrite % 8) {
					case 0:
						T							= Y;
						T							= T >> 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;
						break;
					case 1:
						T							= Y;
						T							= T >> 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;

						T							= Y;
						T							= T << 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;
						break;
					case 2:
						T							= Y;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Y;
						T							= T << 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;
						break;
					case 3:
						T							= Y;
						T							= T >> 8;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Y;
						T							= T << 0;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;
						break;
					case 4:
						T							= Y;
						T							= T >> 9;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;

						T							= Y;
						T							= T >> 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;
						break;
					case 5:
						T							= Y;
						T							= T >> 10;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;

						T							= Y;
						T							= T >> 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;
						break;
					case 6:
						T							= Y;
						T							= T >> 11;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;

						T							= Y;
						T							= T >> 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					case 7:
						T							= Y;
						T							= T >> 12;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;

						T							= Y;
						T							= T >> 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;
						break;
					default:
						break;
					}
				}
				else if (8192 <= lenY && lenY < 16384) { // 14
					switch (lenToWrite % 8) {
					case 0:
						T							= Y;
						T							= T >> 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;
						break;
					case 1:
						T							= Y;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;

						T							= Y;
						T							= T << 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;
						break;
					case 2:
						T							= Y;
						T							= T >> 8;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;

						T							= Y;
						T							= T << 0;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;
						break;
					case 3:
						T							= Y;
						T							= T >> 9;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Y;
						T							= T >> 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;
						break;
					case 4:
						T							= Y;
						T							= T >> 10;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;

						T							= Y;
						T							= T >> 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;
						break;
					case 5:
						T							= Y;
						T							= T >> 11;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;

						T							= Y;
						T							= T >> 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					case 6:
						T							= Y;
						T							= T >> 12;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;

						T							= Y;
						T							= T >> 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;
						break;
					case 7:
						T							= Y;
						T							= T >> 13;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;

						T							= Y;
						T							= T >> 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;
						break;
					default:
						break;
					}
				}
				else if (16384 <= lenY && lenY < 32768) { // 15
					switch (lenToWrite % 8) {
					case 0:
						T							= Y;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;
						break;
					case 1:
						T							= Y;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;

						T							= Y;
						T							= T << 0;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;
						break;
					case 2:
						T							= Y;
						T							= T >> 9;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;

						T							= Y;
						T							= T >> 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;
						break;
					case 3:
						T							= Y;
						T							= T >> 10;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Y;
						T							= T >> 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;
						break;
					case 4:
						T							= Y;
						T							= T >> 11;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;

						T							= Y;
						T							= T >> 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					case 5:
						T							= Y;
						T							= T >> 12;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;

						T							= Y;
						T							= T >> 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;
						break;
					case 6:
						T							= Y;
						T							= T >> 13;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;

						T							= Y;
						T							= T >> 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;
						break;
					case 7:
						T							= Y;
						T							= T >> 14;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;

						T							= Y;
						T							= T >> 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;
						break;
					default:
						break;
					}
				}
				else if (32768 <= lenY && lenY < 65536) { // 16
					switch (lenToWrite % 8) {
					case 0:
						T							= Y;
						T							= T >> 8;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 0;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;
						break;
					case 1:
						T							= Y;
						T							= T >> 8;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;

						T							= Y;
						T							= T >> 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;
						break;
					case 2:
						T							= Y;
						T							= T >> 10;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;

						T							= Y;
						T							= T >> 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;
						break;
					case 3:
						T							= Y;
						T							= T >> 11;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Y;
						T							= T >> 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					case 4:
						T							= Y;
						T							= T >> 12;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;

						T							= Y;
						T							= T >> 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;
						break;
					case 5:
						T							= Y;
						T							= T >> 13;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;

						T							= Y;
						T							= T >> 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;
						break;
					case 6:
						T							= Y;
						T							= T >> 14;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;

						T							= Y;
						T							= T >> 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;
						break;
					case 7:
						T							= Y;
						T							= T >> 15;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;

						T							= Y;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Y;
						T							= T << 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;
						break;
					default:
						break;
					}
				}

				if		(lenZ <  256) {
					switch (lenToWrite % 8) {
					case 0:
						T							= Z;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= W;
						lenToWrite				   += 8;
						break;
					case 1:
						T							= Z;
						T							= T >> 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;

						T							= Z;
						T							= T << 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;
						break;
					case 2:
						T							= Z;
						T							= T >> 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;

						T							= Z;
						T							= T << 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;
						break;
					case 3:
						T							= Z;
						T							= T >> 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Z;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					case 4:
						T							= Z;
						T							= T >> 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;

						T							= Z;
						T							= T << 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;
						break;
					case 5:
						T							= Z;
						T							= T >> 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;

						T							= Z;
						T							= T << 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;
						break;
					case 6:
						T							= Z;
						T							= T >> 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;

						T							= Z;
						T							= T << 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;
						break;
					case 7:
						T							= Z;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;

						T							= Z;
						T							= T << 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;
						break;
					default:
						break;
					}
				}
				else if (256  <= lenZ && lenZ < 512) { // 9
					switch (lenToWrite % 8) {
					case 0:
						T							= Z;
						T							= T >> 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;
						break;
					case 1:
						T							= Z;
						T							= T >> 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;

						T							= Z;
						T							= T << 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;
						break;
					case 2:
						T							= Z;
						T							= T >> 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;

						T							= Z;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					case 3:
						T							= Z;
						T							= T >> 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Z;
						T							= T << 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;
						break;
					case 4:
						T							= Z;
						T							= T >> 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;

						T							= Z;
						T							= T << 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;
						break;
					case 5:
						T							= Z;
						T							= T >> 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;

						T							= Z;
						T							= T << 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;
						break;
					case 6:
						T							= Z;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;

						T							= Z;
						T							= T << 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;
						break;
					case 7:
						T							= Z;
						T							= T >> 8;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;

						T							= Z;
						T							= T << 0;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;
						break;
					default:
						break;
					}
				}
				else if (512  <= lenZ && lenZ < 1024) {
					switch (lenToWrite % 8) {
					case 0:
						T							= Z;
						T							= T >> 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;
						break;
					case 1:
						T							= Z;
						T							= T >> 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;

						T							= Z;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					case 2:
						T							= Z;
						T							= T >> 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;

						T							= Z;
						T							= T << 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;
						break;
					case 3:
						T							= Z;
						T							= T >> 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Z;
						T							= T << 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;
						break;
					case 4:
						T							= Z;
						T							= T >> 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;

						T							= Z;
						T							= T << 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;
						break;
					case 5:
						T							= Z;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;

						T							= Z;
						T							= T << 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;
						break;
					case 6:
						T							= Z;
						T							= T >> 8;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;

						T							= Z;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;
						break;
					case 7:
						T							= Z;
						T							= T >> 9;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;

						T							= Z;
						T							= T >> 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;
						break;
					default:
						break;
					}
				}
				else if (1024 <= lenZ && lenZ < 2048) { // 11
					switch (lenToWrite % 8) {
					case 0:
						T							= Z;
						T							= T >> 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					case 1:
						T							= Z;
						T							= T >> 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;

						T							= Z;
						T							= T << 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;
						break;
					case 2:
						T							= Z;
						T							= T >> 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;

						T							= Z;
						T							= T << 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;
						break;
					case 3:
						T							= Z;
						T							= T >> 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Z;
						T							= T << 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;
						break;
					case 4:
						T							= Z;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;

						T							= Z;
						T							= T << 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;
						break;
					case 5:
						T							= Z;
						T							= T >> 8;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;

						T							= Z;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;
						break;
					case 6:
						T							= Z;
						T							= T >> 9;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;

						T							= Z;
						T							= T >> 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;
						break;
					case 7:
						T							= Z;
						T							= T >> 10;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;

						T							= Z;
						T							= T >> 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;
						break;
					default:
						break;
					}
				}
				else if (2048 <= lenZ && lenZ < 4096) { // 12
					switch (lenToWrite % 8) {
					case 0:
						T							= Z;
						T							= T >> 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;
						break;
					case 1:
						T							= Z;
						T							= T >> 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;

						T							= Z;
						T							= T << 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;
						break;
					case 2:
						T							= Z;
						T							= T >> 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;

						T							= Z;
						T							= T << 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;
						break;
					case 3:
						T							= Z;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Z;
						T							= T << 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;
						break;
					case 4:
						T							= Z;
						T							= T >> 8;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;

						T							= Z;
						T							= T << 0;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;
						break;
					case 5:
						T							= Z;
						T							= T >> 9;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;

						T							= Z;
						T							= T >> 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;
						break;
					case 6:
						T							= Z;
						T							= T >> 10;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;

						T							= Z;
						T							= T >> 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;
						break;
					case 7:
						T							= Z;
						T							= T >> 11;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;

						T							= Z;
						T							= T >> 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					default:
						break;
					}
				}
				else if (4096 <= lenZ && lenZ < 8192) { // 13
					switch (lenToWrite % 8) {
					case 0:
						T							= Z;
						T							= T >> 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;
						break;
					case 1:
						T							= Z;
						T							= T >> 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;

						T							= Z;
						T							= T << 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;
						break;
					case 2:
						T							= Z;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;

						T							= Z;
						T							= T << 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;
						break;
					case 3:
						T							= Z;
						T							= T >> 8;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Z;
						T							= T << 0;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;
						break;
					case 4:
						T							= Z;
						T							= T >> 9;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;

						T							= Z;
						T							= T >> 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;
						break;
					case 5:
						T							= Z;
						T							= T >> 10;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;

						T							= Z;
						T							= T >> 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;
						break;
					case 6:
						T							= Z;
						T							= T >> 11;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;

						T							= Z;
						T							= T >> 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					case 7:
						T							= Z;
						T							= T >> 12;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;

						T							= Z;
						T							= T >> 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;
						break;
					default:
						break;
					}
				}
				else if (8192 <= lenZ && lenZ < 16384) { // 14
					switch (lenToWrite % 8) {
					case 0:
						T							= Z;
						T							= T >> 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;
						break;
					case 1:
						T							= Z;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;

						T							= Z;
						T							= T << 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;
						break;
					case 2:
						T							= Z;
						T							= T >> 8;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;

						T							= Z;
						T							= T << 0;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;
						break;
					case 3:
						T							= Z;
						T							= T >> 9;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Z;
						T							= T >> 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;
						break;
					case 4:
						T							= Z;
						T							= T >> 10;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;

						T							= Z;
						T							= T >> 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;
						break;
					case 5:
						T							= Z;
						T							= T >> 11;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;

						T							= Z;
						T							= T >> 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					case 6:
						T							= Z;
						T							= T >> 12;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;

						T							= Z;
						T							= T >> 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;
						break;
					case 7:
						T							= Z;
						T							= T >> 13;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;

						T							= Z;
						T							= T >> 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;
						break;
					default:
						break;
					}
				}
				else if (16384 <= lenZ && lenZ < 32768) { // 15
					switch (lenToWrite % 8) {
					case 0:
						T							= Z;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;
						break;
					case 1:
						T							= Z;
						T							= T >> 8;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;

						T							= Z;
						T							= T << 0;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;
						break;
					case 2:
						T							= Z;
						T							= T >> 9;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;

						T							= Z;
						T							= T >> 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;
						break;
					case 3:
						T							= Z;
						T							= T >> 10;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Z;
						T							= T >> 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;
						break;
					case 4:
						T							= Z;
						T							= T >> 11;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;

						T							= Z;
						T							= T >> 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					case 5:
						T							= Z;
						T							= T >> 12;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;

						T							= Z;
						T							= T >> 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;
						break;
					case 6:
						T							= Z;
						T							= T >> 13;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;

						T							= Z;
						T							= T >> 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;
						break;
					case 7:
						T							= Z;
						T							= T >> 14;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;

						T							= Z;
						T							= T >> 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;
						break;
					default:
						break;
					}
				}
				else if (32768 <= lenZ && lenZ < 65536) { // 16
					switch (lenToWrite % 8) {
					case 0:
						T							= Z;
						T							= T >> 8;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 0;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;
						break;
					case 1:
						T							= Z;
						T							= T >> 9;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;

						T							= Z;
						T							= T >> 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;
						break;
					case 2:
						T							= Z;
						T							= T >> 10;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;

						T							= Z;
						T							= T >> 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;
						break;
					case 3:
						T							= Z;
						T							= T >> 11;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 5;

						T							= Z;
						T							= T >> 3;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					case 4:
						T							= Z;
						T							= T >> 12;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;

						T							= Z;
						T							= T >> 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 4;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 4;
						break;
					case 5:
						T							= Z;
						T							= T >> 13;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;

						T							= Z;
						T							= T >> 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 5;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 3;
						break;
					case 6:
						T							= Z;
						T							= T >> 14;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 2;

						T							= Z;
						T							= T >> 6;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 2;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 6;
						break;
					case 7:
						T							= Z;
						T							= T >> 15;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 1;

						T							= Z;
						T							= T >> 7;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 8;

						T							= Z;
						T							= T << 1;
						W							= (unsigned char)T;
						bitToWrite[lenToWrite / 8]	= bitToWrite[lenToWrite / 8] | W;
						lenToWrite				   += 7;
						break;
					default:
						break;
					}
				}

				if (compressTail == 0) {
					fwrite(bitToWrite, sizeof(unsigned char), lenToWrite / 8, compressFile);

					compressTail	  = lenToWrite % 8;
					bitToWriteTail[0] = bitToWrite[lenToWrite / 8];
				}
				else {
					for (int x = 0; x < (lenToWrite + compressTail) / 8 + 1; x++) {
						T   = bitToWrite[x];
						T >>= compressTail;
						T   = bitToWriteTail[0] | T;

						N					= bitToWrite[x];
						N				  <<= 8 - compressTail;
						bitToWriteTail[0]	= N;

						bitToWrite[x] = (unsigned char)T;
					}

					fwrite(bitToWrite, sizeof(unsigned char), (lenToWrite + compressTail) / 8, compressFile);

					T					= bitToWrite[(lenToWrite + compressTail) / 8];
					compressTail		= (lenToWrite + compressTail) % 8;
					bitToWriteTail[0]	= (unsigned char)T;
				}
			}

			if (compressTail > 0) {
				fwrite(bitToWriteTail, sizeof(unsigned char), 1, compressFile);
			}

			delete[] bitToWrite;
			delete[] bitToWriteTail;
			cloud_raw.clear();
			cloud_raw.resize(0);
			cloud_raw_in.clear();
			cloud_raw_in.resize(0);
			cloud_raw_out.clear();
			cloud_raw_out.resize(0);
			
			fclose(compressFile);
		}
	}	

	double timeInterval = Timer::Stop();
	printf("time past:%fs\n", timeInterval);

	return 0;
}

bool PointCloudCompressor::DecompressSingleFile(
	const std::string& input_file,
    const std::string& output_file
) {
	Timer::Start();

	pcl::PointCloud<pcl::PointXYZ> cloud_raw;
	pcl::PointXYZ point;

	YAML::Node config;
	config = YAML::LoadFile("pcd_compress.yaml");
	if (config.IsNull()) {
		std::cout << "ERROR: Cannot open config file " << std::endl;
		return -1;
	}

	FILE* compressFile = FileUtils::CrossPlatformFopen(input_file, "rb");
	if (compressFile == NULL) {
		std::cout << "ERROR: Cannot open compress file " << std::endl;
		return -1;
	}
	uint64_t fileLength = FileUtils::GetFileSize(compressFile);

	Eigen::Vector3f	central_min;
	Eigen::Vector3f	central_max;
	central_min[0] = config["X_MIN"].as<float>();
	central_min[1] = config["Y_MIN"].as<float>();
	central_min[2] = config["Z_MIN"].as<float>();
	central_max[0] = config["X_MAX"].as<float>();
	central_max[1] = config["Y_MAX"].as<float>();
	central_max[2] = config["Z_MAX"].as<float>();

	uint16_t lenX = 0;
	uint16_t lenY = 0;
	uint16_t lenZ = 0;

	fread((char*)(&lenX), sizeof(lenX), 1, compressFile);
	fread((char*)(&lenY), sizeof(lenY), 1, compressFile);
	fread((char*)(&lenZ), sizeof(lenZ), 1, compressFile);

	unsigned char LEN_READ_ONCE = 0;
	unsigned char FIX_READ_ONCE = 0;
	
	if		(lenX <  256) {
		LEN_READ_ONCE += 8;
		lenX = 8;
	}
	else if (256  <= lenX && lenX < 512) {
		LEN_READ_ONCE += 9;
		lenX = 9;
	}
	else if (512  <= lenX && lenX < 1024) {
		LEN_READ_ONCE += 10;
		lenX = 10;
	}
	else if (1024 <= lenX && lenX < 2048) {
		LEN_READ_ONCE += 11;
		lenX = 11;
	}
	else if (2048 <= lenX && lenX < 4096) {
		LEN_READ_ONCE += 12;
		lenX = 12;
	}
	else if (4096 <= lenX && lenX < 8192) {
		LEN_READ_ONCE += 13;
		lenX = 13;
	}
	else if (8192 <= lenX && lenX < 16384) {
		LEN_READ_ONCE += 14;
		lenX = 14;
	}
	else if (16384 <= lenX && lenX < 32768) {
		LEN_READ_ONCE += 15;
		lenX = 15;
	}
	else if (32768 <= lenX && lenX < 65536) {
		LEN_READ_ONCE += 16;
		lenX = 16;
	}

	if		(lenY <  256) {
		LEN_READ_ONCE += 8;
		lenY = 8;
	}
	else if (256  <= lenY && lenY < 512) {
		LEN_READ_ONCE += 9;
		lenY = 9;
	}
	else if (512  <= lenY && lenY < 1024) {
		LEN_READ_ONCE += 10;
		lenY = 10;
	}
	else if (1024 <= lenY && lenY < 2048) {
		LEN_READ_ONCE += 11;
		lenY = 11;
	}
	else if (2048 <= lenY && lenY < 4096) {
		LEN_READ_ONCE += 12;
		lenY = 12;
	}
	else if (4096 <= lenY && lenY < 8192) {
		LEN_READ_ONCE += 13;
		lenY = 13;
	}
	else if (8192 <= lenY && lenY < 16384) {
		LEN_READ_ONCE += 14;
		lenY = 14;
	}
	else if (16384 <= lenY && lenY < 32768) {
		LEN_READ_ONCE += 15;
		lenY = 15;
	}
	else if (32768 <= lenY && lenY < 65536) {
		LEN_READ_ONCE += 16;
		lenY = 16;
	}

	if		(lenZ <  256) {
		LEN_READ_ONCE += 8;
		lenZ = 8;
	}
	else if (256  <= lenZ && lenZ < 512) {
		LEN_READ_ONCE += 9;
		lenZ = 9;
	}
	else if (512  <= lenZ && lenZ < 1024) {
		LEN_READ_ONCE += 10;
		lenY = 10;
	}
	else if (1024 <= lenZ && lenZ < 2048) {
		LEN_READ_ONCE += 11;
		lenY = 11;
	}
	else if (2048 <= lenZ && lenZ < 4096) {
		LEN_READ_ONCE += 12;
		lenZ = 12;
	}
	else if (4096 <= lenZ && lenZ < 8192) {
		LEN_READ_ONCE += 13;
		lenZ = 13;
	}
	else if (8192 <= lenZ && lenZ < 16384) {
		LEN_READ_ONCE += 14;
		lenZ = 14;
	}
	else if (16384 <= lenZ && lenZ < 32768) {
		LEN_READ_ONCE += 15;
		lenZ = 15;
	}
	else if (32768 <= lenZ && lenZ < 65536) {
		LEN_READ_ONCE += 16;
		lenZ = 16;
	}

	FIX_READ_ONCE = LEN_READ_ONCE;
	LEN_READ_ONCE = (LEN_READ_ONCE / 8) + 1;

	bool AdjustRead	= false;
	float pX		= 0;
	float pY		= 0;
	float pZ		= 0;
	uint16_t X		= 0;
	uint16_t Y		= 0;
	uint16_t Z		= 0;
	uint16_t T		= 0;
	uint16_t N		= 0;
	uint64_t numPo	= 0;
	unsigned char W = 0;

	unsigned char compressTail		= 0;
	unsigned char bitToWriteTail[2] = { 0 };
	memset(bitToWriteTail, 0x00, 2);

	for (uint64_t lenFile = 0; lenFile < fileLength;) {
		unsigned int  lenToWrite	 = 0;
		unsigned char bitToWrite[10] = { 0 };
		memset(bitToWrite, 0x00, 10);

		unsigned char lenRead = 0;
		if (AdjustRead == false) {
			fread(bitToWrite, sizeof(unsigned char), LEN_READ_ONCE, compressFile);
			lenRead  = LEN_READ_ONCE;
			lenFile += lenRead;
		}
		else {
			fread(bitToWrite, sizeof(unsigned char), LEN_READ_ONCE - 1, compressFile);
			lenRead  = LEN_READ_ONCE - 1;
			lenFile += lenRead;
		}		

		if (compressTail > 0) {
			for (int i = 0; i <= lenRead; i++) {
				T   = bitToWrite[i];
				T >>= compressTail;
				T   = bitToWriteTail[0] | T;

				N					= bitToWrite[i];
				N				  <<= 8 - compressTail;
				bitToWriteTail[0]	= (unsigned char)N;
				bitToWrite[i]		= (unsigned char)T;
			}
			lenRead++;
		}

		switch (lenX) {
		case 8:
			X			 = bitToWrite[lenToWrite / 8];
			pX			 = X;
			pX			 = pX / 100;
			point.x		 = pX;
			lenToWrite	+= 8;			
			break;
		case 9:
			X			 = bitToWrite[lenToWrite / 8];
			T			 = bitToWrite[lenToWrite / 8 + 1];
			T		   >>= 7;
			X		   <<= 1;
			X			 = X | T;
			pX			 = X;
			pX			 = pX / 100;
			point.x		 = pX;
			lenToWrite	+= 9;
			break;
		case 10:
			X			 = bitToWrite[lenToWrite / 8];
			T			 = bitToWrite[lenToWrite / 8 + 1];
			T		   >>= 6;
			X		   <<= 2;
			X			 = X | T;
			pX			 = X;
			pX			 = pX / 100;
			point.x		 = pX;
			lenToWrite	+= 10;
			break;
		case 11:
			X			 = bitToWrite[lenToWrite / 8];
			T			 = bitToWrite[lenToWrite / 8 + 1];
			T		   >>= 5;
			X		   <<= 3;
			X			 = X | T;
			pX			 = X;
			pX			 = pX / 100;
			point.x		 = pX;
			lenToWrite	+= 11;
			break;
		case 12:
			X			 = bitToWrite[lenToWrite / 8];
			T			 = bitToWrite[lenToWrite / 8 + 1];
			T		   >>= 4;
			X		   <<= 4;
			X			 = X | T;
			pX			 = X;
			pX			 = pX / 100;
			point.x		 = pX;
			lenToWrite	+= 12;
			break;
		case 13:
			X			 = bitToWrite[lenToWrite / 8];
			T			 = bitToWrite[lenToWrite / 8 + 1];
			T		   >>= 3;
			X		   <<= 5;
			X			 = X | T;
			pX			 = X;
			pX			 = pX / 100;
			point.x		 = pX;
			lenToWrite	+= 13;
			break;
		case 14:
			X			 = bitToWrite[lenToWrite / 8];
			T			 = bitToWrite[lenToWrite / 8 + 1];
			T		   >>= 2;
			X		   <<= 6;
			X			 = X | T;
			pX			 = X;
			pX			 = pX / 100;
			point.x		 = pX;
			lenToWrite	+= 14;
			break;
		case 15:
			X			 = bitToWrite[lenToWrite / 8];
			T			 = bitToWrite[lenToWrite / 8 + 1];
			T		   >>= 1;
			X		   <<= 7;
			X			 = X | T;
			pX			 = X;
			pX			 = pX / 100;
			point.x		 = pX;
			lenToWrite	+= 15;
			break;
		case 16:
			X			 = bitToWrite[lenToWrite / 8];
			T			 = bitToWrite[lenToWrite / 8 + 1];
			T		   >>= 0;
			X		   <<= 8;
			X			 = X | T;
			pX			 = X;
			pX			 = pX / 100;
			point.x		 = pX;
			lenToWrite	+= 16;
			break;
		default:
			break;
		}

		if		(lenToWrite % 8 == 0) {
			switch (lenY) {
			case 8:
				Y			 = bitToWrite[lenToWrite / 8];
				pY			 = Y;
				pY			 = pY / 100;
				point.y		 = pY;
				lenToWrite	+= 8;			
				break;
			case 9:
				Y			 = bitToWrite[lenToWrite / 8];
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 7;
				Y		   <<= 1;
				Y			 = Y | T;
				pY			 = Y;
				pY			 = pY / 100;
				point.y		 = pY;
				lenToWrite	+= 9;
				break;
			case 10:
				Y			 = bitToWrite[lenToWrite / 8];
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 6;
				Y		   <<= 2;
				Y			 = Y | T;
				pY			 = Y;
				pY			 = pY / 100;
				point.y		 = pY;
				lenToWrite	+= 10;
				break;
			case 11:
				Y			 = bitToWrite[lenToWrite / 8];
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 5;
				Y		   <<= 3;
				Y			 = Y | T;
				pY			 = Y;
				pY			 = pY / 100;
				point.y		 = pY;
				lenToWrite	+= 11;
				break;
			default:
				break;
			}
		}
		else if (lenToWrite % 8 == 1) {
			switch (lenY) {
			case 8:
				Y			 = bitToWrite[lenToWrite / 8];
				Y		   <<= 9;
				Y		   >>= 8;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 7;
				Y			 = Y | T;
				pY			 = Y;
				pY			 = pY / 100;
				point.y		 = pY;
				lenToWrite	+= 8;			
				break;
			case 9:
				Y			 = bitToWrite[lenToWrite / 8];
				Y		   <<= 9;
				Y		   >>= 7;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 7;
				Y			 = Y | T;
				pY			 = Y;
				pY			 = pY / 100;
				point.y		 = pY;
				lenToWrite	+= 9;
				break;
			case 10:
				Y			 = bitToWrite[lenToWrite / 8];
				Y		   <<= 9;
				Y		   >>= 6;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 6;
				Y			 = Y | T;
				pY			 = Y;
				pY			 = pY / 100;
				point.y		 = pY;
				lenToWrite	+= 10;
				break;
			case 11:
				Y			 = bitToWrite[lenToWrite / 8];
				Y		   <<= 9;
				Y		   >>= 5;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 5;
				Y			 = Y | T;
				pY			 = Y;
				pY			 = pY / 100;
				point.y		 = pY;
				lenToWrite	+= 11;
				break;
			default:
				break;
			}
		}
		else if (lenToWrite % 8 == 2) {
			switch (lenY) {
			case 8:
				Y			 = bitToWrite[lenToWrite / 8];
				Y		   <<= 10;
				Y		   >>= 8;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 6;
				Y			 = Y | T;
				pY			 = Y;
				pY			 = pY / 100;
				point.y		 = pY;
				lenToWrite	+= 8;			
				break;
			case 9:
				Y			 = bitToWrite[lenToWrite / 8];
				Y		   <<= 10;
				Y		   >>= 7;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 5;
				Y			 = Y | T;
				pY			 = Y;
				pY			 = pY / 100;
				point.y		 = pY;
				lenToWrite	+= 9;
				break;
			case 10:
				Y			 = bitToWrite[lenToWrite / 8];
				Y		   <<= 10;
				Y		   >>= 6;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 4;
				Y			 = Y | T;
				pY			 = Y;
				pY			 = pY / 100;
				point.y		 = pY;
				lenToWrite	+= 10;
				break;
			case 11:
				Y			 = bitToWrite[lenToWrite / 8];
				Y		   <<= 10;
				Y		   >>= 5;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 3;
				Y			 = Y | T;
				pY			 = Y;
				pY			 = pY / 100;
				point.y		 = pY;
				lenToWrite	+= 11;
				break;
			default:
				break;
			}
		}
		else if (lenToWrite % 8 == 3) {
			switch (lenY) {
			case 8:
				Y			 = bitToWrite[lenToWrite / 8];
				Y		   <<= 11;
				Y		   >>= 8;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 5;
				Y			 = Y | T;
				pY			 = Y;
				pY			 = pY / 100;
				point.y		 = pY;
				lenToWrite	+= 8;			
				break;
			case 9:
				Y			 = bitToWrite[lenToWrite / 8];
				Y		   <<= 11;
				Y		   >>= 7;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 4;
				Y			 = Y | T;
				pY			 = Y;
				pY			 = pY / 100;
				point.y		 = pY;
				lenToWrite	+= 9;
				break;
			case 10:
				Y			 = bitToWrite[lenToWrite / 8];
				Y		   <<= 11;
				Y		   >>= 6;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 3;
				Y			 = Y | T;
				pY			 = Y;
				pY			 = pY / 100;
				point.y		 = pY;
				lenToWrite	+= 10;
				break;
			case 11:
				Y			 = bitToWrite[lenToWrite / 8];
				Y		   <<= 11;
				Y		   >>= 5;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 2;
				Y			 = Y | T;
				pY			 = Y;
				pY			 = pY / 100;
				point.y		 = pY;
				lenToWrite	+= 11;
				break;
			default:
				break;
			}
		}

		if		(lenToWrite % 8 == 0) {
			switch (lenZ) {
			case 8:
				Z			 = bitToWrite[lenToWrite / 8];
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 8;			
				break;
			case 9:
				Z			 = bitToWrite[lenToWrite / 8];
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 7;
				Z		   <<= 1;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 9;
				break;
			case 10:
				Z			 = bitToWrite[lenToWrite / 8];
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 6;
				Z		   <<= 2;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 10;
				break;
			case 11:
				Z			 = bitToWrite[lenToWrite / 8];
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 5;
				Z		   <<= 3;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 11;
				break;
			default:
				break;
			}
		}
		else if (lenToWrite % 8 == 1) {
			switch (lenZ) {
			case 8:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 9;
				Z		   >>= 8;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 7;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 8;			
				break;
			case 9:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 9;
				Z		   >>= 7;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 7;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 9;
				break;
			case 10:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 9;
				Z		   >>= 6;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 6;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 10;
				break;
			case 11:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 9;
				Z		   >>= 5;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 5;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 11;
				break;
			default:
				break;
			}
		}
		else if (lenToWrite % 8 == 2) {
			switch (lenZ) {
			case 8:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 10;
				Z		   >>= 8;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 6;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 8;			
				break;
			case 9:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 10;
				Z		   >>= 7;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 5;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 9;
				break;
			case 10:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 10;
				Z		   >>= 6;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 4;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 10;
				break;
			case 11:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 10;
				Z		   >>= 5;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 3;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 11;
				break;
			default:
				break;
			}
		}
		else if (lenToWrite % 8 == 3) {
			switch (lenZ) {
			case 8:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 11;
				Z		   >>= 8;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 5;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 8;			
				break;
			case 9:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 11;
				Z		   >>= 7;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 4;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 9;
				break;
			case 10:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 11;
				Z		   >>= 6;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 3;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 10;
				break;
			case 11:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 11;
				Z		   >>= 5;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 2;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 11;
				break;
			default:
				break;
			}
		}
		else if (lenToWrite % 8 == 4) {
			switch (lenZ) {
			case 8:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 12;
				Z		   >>= 8;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 4;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 8;			
				break;
			case 9:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 12;
				Z		   >>= 7;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 3;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 9;
				break;
			case 10:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 12;
				Z		   >>= 6;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 2;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 10;
				break;
			case 11:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 12;
				Z		   >>= 5;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 1;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 11;
				break;
			default:
				break;
			}
		}
		else if (lenToWrite % 8 == 5) {
			switch (lenZ) {
			case 8:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 13;
				Z		   >>= 8;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 3;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 8;			
				break;
			case 9:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 13;
				Z		   >>= 7;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 2;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 9;
				break;
			case 10:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 13;
				Z		   >>= 6;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 1;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 10;
				break;
			case 11:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 13;
				Z		   >>= 5;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 11;
				break;
			default:
				break;
			}
		}
		else if (lenToWrite % 8 == 6) {
			switch (lenZ) {
			case 8:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 14;
				Z		   >>= 8;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 2;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 8;			
				break;
			case 9:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 14;
				Z		   >>= 7;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   >>= 1;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 9;
				break;
			case 10:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 14;
				Z		   >>= 6;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 10;
				break;
			case 11:
				Z			 = bitToWrite[lenToWrite / 8];
				Z		   <<= 14;
				Z		   >>= 5;
				T			 = bitToWrite[lenToWrite / 8 + 1];
				T		   <<= 1;
				Z			 = Z | T;
				T			 = bitToWrite[lenToWrite / 8 + 2];
				T		   >>= 7;
				Z			 = Z | T;
				pZ			 = Z;
				pZ			 = pZ / 100;
				point.z		 = pZ;
				lenToWrite	+= 11;
				break;
			default:
				break;
			}
		}

		point.x = point.x + central_min[0];
		point.y = point.y + central_min[1];
		point.z = point.z + central_min[2];

		cloud_raw.push_back(point);

		unsigned int fixTailLength = 0;
		if (compressTail == 0) {
			compressTail		= LEN_READ_ONCE * 8 - FIX_READ_ONCE;
			T					= bitToWrite[lenRead - 1];
			T				  <<= 8 - compressTail;
			bitToWriteTail[0]	= (unsigned char)T;			
		}
		else if (AdjustRead == false) {
			fixTailLength = LEN_READ_ONCE * 8 - FIX_READ_ONCE;

			T					= bitToWrite[lenRead - 2];
			T				  <<= 8 - fixTailLength;
			bitToWriteTail[0]	= (unsigned char)T;

			W					= bitToWrite[lenRead - 1];
			W				  >>= fixTailLength;
			bitToWriteTail[0]	= bitToWriteTail[0] | W;
			compressTail		= compressTail + fixTailLength;
		}
		else {
			fixTailLength = compressTail + (LEN_READ_ONCE - 1) * 8 - FIX_READ_ONCE;

			W					= bitToWrite[lenRead - 1];
			W				  <<= compressTail - fixTailLength;
			bitToWriteTail[0]	= W;
			compressTail		= fixTailLength;
		}

		if (compressTail + (LEN_READ_ONCE - 1) * 8 >= FIX_READ_ONCE) {
			AdjustRead = true;
		}
		else {
			AdjustRead = false;
		}
	}

	if (pcl::io::savePCDFile(output_file, cloud_raw, true)) {
		std::cout << "ERROR: Cannot open pcd file " << std::endl;
		return -1;
	}
	fclose(compressFile);

	double timeInterval = Timer::Stop();
	printf("time past:%fs\n", timeInterval);

	return 0;
}

bool PointCloudCompressor::DecompressFolder(
	const std::string& input_folder,
    const std::string& output_folder
) {
	Timer::Start();

	pcl::PointCloud<pcl::PointXYZ> cloud_raw;
	pcl::PointXYZ point;

	YAML::Node config;
	config = YAML::LoadFile("pcd_compress.yaml");
	if (config.IsNull()) {
		std::cout << "ERROR: Cannot open config file " << std::endl;
		return -1;
	}

	Eigen::Vector3f	central_min;
	Eigen::Vector3f	central_max;
	central_min[0] = config["X_MIN"].as<float>();
	central_min[1] = config["Y_MIN"].as<float>();
	central_min[2] = config["Z_MIN"].as<float>();
	central_max[0] = config["X_MAX"].as<float>();
	central_max[1] = config["Y_MAX"].as<float>();
	central_max[2] = config["Z_MAX"].as<float>();

	uint16_t lenX = 0;
	uint16_t lenY = 0;
	uint16_t lenZ = 0;

	vector<string> pcdList;
	string format = ".mtx";
	FileUtils::GetAllFormatFiles(input_folder, pcdList, format);

	int size = pcdList.size();
	for (int i = 0; i < size; i++) {
		ifstream in(pcdList[i]);

		if (in) { // 有该文件
			FILE* compressFile = FileUtils::CrossPlatformFopen(pcdList[i], "rb");
			if (compressFile == NULL) {
				std::cout << "ERROR: Cannot open compress file " << std::endl;
				return -1;
			}
			uint64_t fileLength = FileUtils::GetFileSize(compressFile);		

			fread((char*)(&lenX), sizeof(lenX), 1, compressFile);
			fread((char*)(&lenY), sizeof(lenY), 1, compressFile);
			fread((char*)(&lenZ), sizeof(lenZ), 1, compressFile);

			unsigned char LEN_READ_ONCE = 0;
			unsigned char FIX_READ_ONCE = 0;
			if		(lenX <  256) {
				LEN_READ_ONCE  += 8;
				lenX			= 8;
			}
			else if (256  <= lenX && lenX < 512) {
				LEN_READ_ONCE  += 9;
				lenX			= 9;
			}
			else if (512  <= lenX && lenX < 1024) {
				LEN_READ_ONCE  += 10;
				lenX			= 10;
			}
			else if (1024 <= lenX && lenX < 2048) {
				LEN_READ_ONCE  += 11;
				lenX			= 11;
			}

			if		(lenY <  256) {
				LEN_READ_ONCE  += 8;
				lenY			= 8;
			}
			else if (256  <= lenY && lenY < 512) {
				LEN_READ_ONCE  += 9;
				lenY			= 9;
			}
			else if (512  <= lenY && lenY < 1024) {
				LEN_READ_ONCE  += 10;
				lenY			= 10;
			}
			else if (1024 <= lenY && lenY < 2048) {
				LEN_READ_ONCE  += 11;
				lenY			= 11;
			}

			if		(lenZ <  256) {
				LEN_READ_ONCE  += 8;
				lenZ			= 8;
			}
			else if (256  <= lenZ && lenZ < 512) {
				LEN_READ_ONCE  += 9;
				lenZ			= 9;
			}
			else if (512  <= lenZ && lenZ < 1024) {
				LEN_READ_ONCE  += 10;
				lenY			= 10;
			}
			else if (1024 <= lenZ && lenZ < 2048) {
				LEN_READ_ONCE  += 11;
				lenY			= 11;
			}

			FIX_READ_ONCE = LEN_READ_ONCE;
			LEN_READ_ONCE = (LEN_READ_ONCE / 8) + 1;

			bool AdjustRead	= false;
			float pX		= 0;
			float pY		= 0;
			float pZ		= 0;
			uint16_t X		= 0;
			uint16_t Y		= 0;
			uint16_t Z		= 0;
			uint16_t T		= 0;
			uint16_t N		= 0;
			uint64_t numPo	= 0;
			unsigned char W = 0;

			unsigned char compressTail		= 0;
			unsigned char bitToWriteTail[2] = { 0 };
			memset(bitToWriteTail, 0x00, 2);

			for (uint64_t lenFile = 0; lenFile < fileLength;) {
				unsigned int  lenToWrite	 = 0;
				unsigned char bitToWrite[10] = { 0 };
				memset(bitToWrite, 0x00, 10);

				unsigned char lenRead = 0;
				if (AdjustRead == false) {
					fread(bitToWrite, sizeof(unsigned char), LEN_READ_ONCE, compressFile);
					lenRead  = LEN_READ_ONCE;
					lenFile += lenRead;
				}
				else {
					fread(bitToWrite, sizeof(unsigned char), LEN_READ_ONCE - 1, compressFile);
					lenRead  = LEN_READ_ONCE - 1;
					lenFile += lenRead;
				}		

				if (compressTail > 0) {
					for (int i = 0; i <= lenRead; i++) {
						T   = bitToWrite[i];
						T >>= compressTail;
						T   = bitToWriteTail[0] | T;

						N					= bitToWrite[i];
						N				  <<= 8 - compressTail;
						bitToWriteTail[0]	= (unsigned char)N;
						bitToWrite[i]		= (unsigned char)T;
					}
					lenRead++;
				}

				switch (lenX) {
				case 8:
					X			 = bitToWrite[lenToWrite / 8];
					pX			 = X;
					pX			 = pX / 100;
					point.x		 = pX;
					lenToWrite	+= 8;			
					break;
				case 9:
					X			 = bitToWrite[lenToWrite / 8];
					T			 = bitToWrite[lenToWrite / 8 + 1];
					T		   >>= 7;
					X		   <<= 1;
					X			 = X | T;
					pX			 = X;
					pX			 = pX / 100;
					point.x		 = pX;
					lenToWrite	+= 9;
					break;
				case 10:
					X			 = bitToWrite[lenToWrite / 8];
					T			 = bitToWrite[lenToWrite / 8 + 1];
					T		   >>= 6;
					X		   <<= 2;
					X			 = X | T;
					pX			 = X;
					pX			 = pX / 100;
					point.x		 = pX;
					lenToWrite	+= 10;
					break;
				case 11:
					X			 = bitToWrite[lenToWrite / 8];
					T			 = bitToWrite[lenToWrite / 8 + 1];
					T		   >>= 5;
					X		   <<= 3;
					X			 = X | T;
					pX			 = X;
					pX			 = pX / 100;
					point.x		 = pX;
					lenToWrite	+= 11;
					break;
				default:
					break;
				}

				if		(lenToWrite % 8 == 0) {
					switch (lenY) {
					case 8:
						Y			 = bitToWrite[lenToWrite / 8];
						pY			 = Y;
						pY			 = pY / 100;
						point.y		 = pY;
						lenToWrite	+= 8;			
						break;
					case 9:
						Y			 = bitToWrite[lenToWrite / 8];
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 7;
						Y		   <<= 1;
						Y			 = Y | T;
						pY			 = Y;
						pY			 = pY / 100;
						point.y		 = pY;
						lenToWrite	+= 9;
						break;
					case 10:
						Y			 = bitToWrite[lenToWrite / 8];
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 6;
						Y		   <<= 2;
						Y			 = Y | T;
						pY			 = Y;
						pY			 = pY / 100;
						point.y		 = pY;
						lenToWrite	+= 10;
						break;
					case 11:
						Y			 = bitToWrite[lenToWrite / 8];
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 5;
						Y		   <<= 3;
						Y			 = Y | T;
						pY			 = Y;
						pY			 = pY / 100;
						point.y		 = pY;
						lenToWrite	+= 11;
						break;
					default:
						break;
					}
				}
				else if (lenToWrite % 8 == 1) {
					switch (lenY) {
					case 8:
						Y			 = bitToWrite[lenToWrite / 8];
						Y		   <<= 9;
						Y		   >>= 8;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 7;
						Y			 = Y | T;
						pY			 = Y;
						pY			 = pY / 100;
						point.y		 = pY;
						lenToWrite	+= 8;			
						break;
					case 9:
						Y			 = bitToWrite[lenToWrite / 8];
						Y		   <<= 9;
						Y		   >>= 7;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 7;
						Y			 = Y | T;
						pY			 = Y;
						pY			 = pY / 100;
						point.y		 = pY;
						lenToWrite	+= 9;
						break;
					case 10:
						Y			 = bitToWrite[lenToWrite / 8];
						Y		   <<= 9;
						Y		   >>= 6;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 6;
						Y			 = Y | T;
						pY			 = Y;
						pY			 = pY / 100;
						point.y		 = pY;
						lenToWrite	+= 10;
						break;
					case 11:
						Y			 = bitToWrite[lenToWrite / 8];
						Y		   <<= 9;
						Y		   >>= 5;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 5;
						Y			 = Y | T;
						pY			 = Y;
						pY			 = pY / 100;
						point.y		 = pY;
						lenToWrite	+= 11;
						break;
					default:
						break;
					}
				}
				else if (lenToWrite % 8 == 2) {
					switch (lenY) {
					case 8:
						Y			 = bitToWrite[lenToWrite / 8];
						Y		   <<= 10;
						Y		   >>= 8;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 6;
						Y			 = Y | T;
						pY			 = Y;
						pY			 = pY / 100;
						point.y		 = pY;
						lenToWrite	+= 8;			
						break;
					case 9:
						Y			 = bitToWrite[lenToWrite / 8];
						Y		   <<= 10;
						Y		   >>= 7;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 5;
						Y			 = Y | T;
						pY			 = Y;
						pY			 = pY / 100;
						point.y		 = pY;
						lenToWrite	+= 9;
						break;
					case 10:
						Y			 = bitToWrite[lenToWrite / 8];
						Y		   <<= 10;
						Y		   >>= 6;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 4;
						Y			 = Y | T;
						pY			 = Y;
						pY			 = pY / 100;
						point.y		 = pY;
						lenToWrite	+= 10;
						break;
					case 11:
						Y			 = bitToWrite[lenToWrite / 8];
						Y		   <<= 10;
						Y		   >>= 5;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 3;
						Y			 = Y | T;
						pY			 = Y;
						pY			 = pY / 100;
						point.y		 = pY;
						lenToWrite	+= 11;
						break;
					default:
						break;
					}
				}
				else if (lenToWrite % 8 == 3) {
					switch (lenY) {
					case 8:
						Y			 = bitToWrite[lenToWrite / 8];
						Y		   <<= 11;
						Y		   >>= 8;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 5;
						Y			 = Y | T;
						pY			 = Y;
						pY			 = pY / 100;
						point.y		 = pY;
						lenToWrite	+= 8;			
						break;
					case 9:
						Y			 = bitToWrite[lenToWrite / 8];
						Y		   <<= 11;
						Y		   >>= 7;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 4;
						Y			 = Y | T;
						pY			 = Y;
						pY			 = pY / 100;
						point.y		 = pY;
						lenToWrite	+= 9;
						break;
					case 10:
						Y			 = bitToWrite[lenToWrite / 8];
						Y		   <<= 11;
						Y		   >>= 6;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 3;
						Y			 = Y | T;
						pY			 = Y;
						pY			 = pY / 100;
						point.y		 = pY;
						lenToWrite	+= 10;
						break;
					case 11:
						Y			 = bitToWrite[lenToWrite / 8];
						Y		   <<= 11;
						Y		   >>= 5;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 2;
						Y			 = Y | T;
						pY			 = Y;
						pY			 = pY / 100;
						point.y		 = pY;
						lenToWrite	+= 11;
						break;
					default:
						break;
					}
				}

				if		(lenToWrite % 8 == 0) {
					switch (lenZ) {
					case 8:
						Z			 = bitToWrite[lenToWrite / 8];
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 8;			
						break;
					case 9:
						Z			 = bitToWrite[lenToWrite / 8];
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 7;
						Z		   <<= 1;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 9;
						break;
					case 10:
						Z			 = bitToWrite[lenToWrite / 8];
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 6;
						Z		   <<= 2;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 10;
						break;
					case 11:
						Z			 = bitToWrite[lenToWrite / 8];
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 5;
						Z		   <<= 3;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 11;
						break;
					default:
						break;
					}
				}
				else if (lenToWrite % 8 == 1) {
					switch (lenZ) {
					case 8:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 9;
						Z		   >>= 8;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 7;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 8;			
						break;
					case 9:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 9;
						Z		   >>= 7;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 7;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 9;
						break;
					case 10:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 9;
						Z		   >>= 6;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 6;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 10;
						break;
					case 11:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 9;
						Z		   >>= 5;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 5;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 11;
						break;
					default:
						break;
					}
				}
				else if (lenToWrite % 8 == 2) {
					switch (lenZ) {
					case 8:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 10;
						Z		   >>= 8;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 6;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 8;			
						break;
					case 9:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 10;
						Z		   >>= 7;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 5;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 9;
						break;
					case 10:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 10;
						Z		   >>= 6;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 4;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 10;
						break;
					case 11:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 10;
						Z		   >>= 5;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 3;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 11;
						break;
					default:
						break;
					}
				}
				else if (lenToWrite % 8 == 3) {
					switch (lenZ) {
					case 8:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 11;
						Z		   >>= 8;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 5;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 8;			
						break;
					case 9:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 11;
						Z		   >>= 7;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 4;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 9;
						break;
					case 10:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 11;
						Z		   >>= 6;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 3;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 10;
						break;
					case 11:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 11;
						Z		   >>= 5;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 2;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 11;
						break;
					default:
						break;
					}
				}
				else if (lenToWrite % 8 == 4) {
					switch (lenZ) {
					case 8:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 12;
						Z		   >>= 8;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 4;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 8;			
						break;
					case 9:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 12;
						Z		   >>= 7;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 3;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 9;
						break;
					case 10:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 12;
						Z		   >>= 6;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 2;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 10;
						break;
					case 11:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 12;
						Z		   >>= 5;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 1;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 11;
						break;
					default:
						break;
					}
				}
				else if (lenToWrite % 8 == 5) {
					switch (lenZ) {
					case 8:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 13;
						Z		   >>= 8;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 3;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 8;			
						break;
					case 9:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 13;
						Z		   >>= 7;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 2;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 9;
						break;
					case 10:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 13;
						Z		   >>= 6;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 1;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 10;
						break;
					case 11:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 13;
						Z		   >>= 5;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 11;
						break;
					default:
						break;
					}
				}
				else if (lenToWrite % 8 == 6) {
					switch (lenZ) {
					case 8:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 14;
						Z		   >>= 8;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 2;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 8;			
						break;
					case 9:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 14;
						Z		   >>= 7;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   >>= 1;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 9;
						break;
					case 10:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 14;
						Z		   >>= 6;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 10;
						break;
					case 11:
						Z			 = bitToWrite[lenToWrite / 8];
						Z		   <<= 14;
						Z		   >>= 5;
						T			 = bitToWrite[lenToWrite / 8 + 1];
						T		   <<= 1;
						Z			 = Z | T;
						T			 = bitToWrite[lenToWrite / 8 + 2];
						T		   >>= 7;
						Z			 = Z | T;
						pZ			 = Z;
						pZ			 = pZ / 100;
						point.z		 = pZ;
						lenToWrite	+= 11;
						break;
					default:
						break;
					}
				}

				point.x = point.x + central_min[0];
				point.y = point.y + central_min[1];
				point.z = point.z + central_min[2];

				cloud_raw.push_back(point);

				unsigned int fixTailLength = 0;
				if (compressTail == 0) {
					compressTail		= LEN_READ_ONCE * 8 - FIX_READ_ONCE;
					T					= bitToWrite[lenRead - 1];
					T				  <<= 8 - compressTail;
					bitToWriteTail[0]	= (unsigned char)T;			
				}
				else if (AdjustRead == false) {
					fixTailLength = LEN_READ_ONCE * 8 - FIX_READ_ONCE;

					T					= bitToWrite[lenRead - 2];
					T				  <<= 8 - fixTailLength;
					bitToWriteTail[0]	= (unsigned char)T;

					W					= bitToWrite[lenRead - 1];
					W				  >>= fixTailLength;
					bitToWriteTail[0]	= bitToWriteTail[0] | W;
					compressTail		= compressTail + fixTailLength;
				}
				else {
					fixTailLength = compressTail + (LEN_READ_ONCE - 1) * 8 - FIX_READ_ONCE;

					W					= bitToWrite[lenRead - 1];
					W				  <<= compressTail - fixTailLength;
					bitToWriteTail[0]	= W;
					compressTail		= fixTailLength;
				}

				if (compressTail + (LEN_READ_ONCE - 1) * 8 >= FIX_READ_ONCE) {
					AdjustRead = true;
				}
				else {
					AdjustRead = false;
				}
			}

			pcdList[i].erase(pcdList[i].end() - 3, pcdList[i].end());
			pcdList[i] += "pcd";

			string::size_type iPos = (	pcdList[i].find_last_of('\\') + 1) == 0 ? 
										pcdList[i].find_last_of('/') + 1 : pcdList[i].find_last_of('\\') + 1;
			fs::path pcdSavePath = output_folder;
			pcdSavePath /= pcdList[i].substr(iPos, pcdList[i].length() - iPos);
			string pcdSaveName = pcdSavePath.string();

			if (pcl::io::savePCDFileBinaryCompressed(pcdSaveName.c_str(), cloud_raw)) {
				std::cout << "ERROR: Cannot open pcd file " << std::endl;
				return -1;
			}
			fclose(compressFile);
			cloud_raw.clear();
		}
	}

	double timeInterval = Timer::Stop();
	printf("time past:%fs\n", timeInterval);

	return 0;
}