# points-cloud-lib-compress

#### 介绍
雷达点云压缩

#### 使用说明
1.  压缩命令：pcd_compress.exe /z pcdfolder zipfolder remove_duplicates(0/1) crop(0/1)
	1. pcdfolder：待压缩的点云数据文件夹
	2. zipfolder: 压缩后的文件夹
	3. remove_duplicates: 是否去重，0表示不去重，1表示去重
	4. crop: 是否根据配置文件中的自定义范围对点云进行裁剪，0表示不裁剪，1表示裁剪 
2.  解压命令：pcd_compress.exe /u zipfolder pcdfolder

#### 测试
1. 测试数据地址：https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles
2. 原数据大小：368MB（已启用二进制压缩）
3. 压缩后大小：223MB
4. 压缩率：223/368=60%
5. 设定的点云裁剪范围越小，则压缩率越高。 