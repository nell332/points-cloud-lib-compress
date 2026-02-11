# points-cloud-lib-compress

#### Introduction
LiDAR Point Cloud Compression

#### Usage Instructions
1. Compression command: pcd_compress.exe /z pcdfolder zipfolder 1 1
2. Decompression command: pcd_compress.exe /u zipfolder pcdfolder

#### Testing
1. Test data link: https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles
2. Original data size: 368 MB (with binary compression enabled)
3. Compressed size: 223 MB
4. Compression ratio: 223/368 = 60%