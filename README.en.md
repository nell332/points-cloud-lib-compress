# points-cloud-lib-compress

#### Introduction
LiDAR Point Cloud Compression

#### Usage Instructions
1. Compression command: pcd_compress.exe /z pcdfolder zipfolder 1 1
    1. pcdfolder: Folder containing point cloud data to be compressed
    2. zipfolder: Destination folder for compressed data
    3. remove_duplicates: Whether to remove duplicate points, 0 = no, 1 = yes
    4. crop: Whether to crop the point cloud according to custom ranges defined in the configuration file, 0 = no, 1 = yes
2. Decompression command: pcd_compress.exe /u zipfolder pcdfolder

#### Testing
1. Test data link: https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles
2. Original data size: 368 MB (with binary compression enabled)
3. Compressed size: 223 MB
4. Compression ratio: 223/368 = 60%
5. The smaller the cropping range set for the point cloud, the higher the compression ratio. 