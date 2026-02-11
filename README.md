# points-cloud-lib-compress

#### 介绍
雷达点云压缩

#### 使用说明
1.  压缩命令：pcd_compress.exe /z pcdfolder zipfolder 1 1
2.  解压命令：pcd_compress.exe /u zipfolder pcdfolder

#### 测试
1. 测试数据地址：https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles
2. 原数据大小：368MB（已启用二进制压缩）
3. 压缩后大小：223MB
4. 压缩率：223/368=60%