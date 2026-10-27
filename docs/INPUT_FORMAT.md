# 坐标、时间戳和文件格式

## 符号

`T_A_B` 将 B 坐标的点变换到 A 坐标。W 为里程计世界坐标，I 为 IMU/body，L 为雷达，C 为相机。
所有平移和点坐标以米为单位，时间戳以秒为单位。不存在自动尺度拟合或时间偏移估计。

## 雷达点云

`all_pcd_body/{timestamp:.6f}.pcd` 包含 XYZI 字段。点应已逐点运动补偿，并表达在 **该扫描时间戳的 IMU/body 坐标系**。
这里的 body 不能仅凭目录名猜成原始雷达坐标。
`lidar_poses.txt` 实际对应 **T_W_I**，不是 T_W_L。

扫描内每点补偿需要外部提供高频轨迹；本程序不会重新读取原始点时间戳再去畸变。
若输入来自 PDA 深度，必须先明确深度是相机 Z 还是射线距离，再按正确内参与外参转换到所需 body 坐标。

## 图像及里程计文件

图像文件名为六位小数时间戳加 `.png`，例如 `1661398632.022152.png`。
当前 `getImagePath()` 使用 `std::to_string(timestamp)`，不要直接改成图像编号或其他小数位数。
图像已去畸变时，配置中的畸变项应为零，内参必须对应最终图像。

两个 TUM 格式位姿文件均为：

```text
timestamp tx ty tz qx qy qz qw
```

- `all_pcd_body/lidar_poses.txt`：按扫描时间顺序的 T_W_I。
- `all_image/image_poses.txt`：按图像时间顺序的 T_W_I。**虽然文件名叫 image_poses，输入仍是 IMU 位姿。**

代码读取这些文件时主要依赖排序后逐行对应关系，并不会完整核验每行时间戳。准备数据时务必保证点云、图像与各自位姿的数量和顺序一致。

## 外参方向

```text
extrin_calib/extrinsic_R, extrinsic_T = T_I_L
extrin_calib/Rcl, Pcl               = T_C_L
T_I_C                              = T_I_L * inverse(T_C_L)
T_W_C                              = T_W_I * T_I_C
```

不要重复应用 IMU↔LiDAR 外参，也不要把 world-to-camera 和 camera-to-world 混用。

## sdk_cameras.txt

这是两阶段优化使用的相机初值，必须与采样后图像列表严格同序：

```text
timestamp qw qx qy qz cx cy cz
```

这里四元数是 **R_W_C**，`cx cy cz` 是世界坐标中的相机中心，即 **T_W_C**。
它与前两个 TUM 文件的四元数顺序和坐标含义均不同。
该文件名沿用历史命名，优化器不会为初值额外添加 SDK 运动先验。
新数据可按上述外参链从 IMU 位姿生成。复现历史实验应使用冻结的原始相机初值文件，不能默认任何旧的 image_poses 导出都能逐值重建它。

## COLMAP

数据库图像名须匹配上述 PNG basename，图像数量须与采样列表一致。
只使用 `two_view_geometries` 中几何验证后的内点，`matches` 表本身不够。
若 OpenCV 图像内参是 `(fx,fy,cx,cy)`，COLMAP 关键点半像素约定对应配置主点为 `(cx+0.5,cy+0.5)`。
更换去畸变模型、输出 K 或分辨率后必须重新提取/匹配特征。

## 输出相机位姿

三个 camera_poses 文件格式为：

```text
timestamp qw qx qy qz tx ty tz
```

这里是 **T_C_W**，不是输入的 T_W_C。还原相机中心用 `C_W=-R_C_W.transpose()*t_C_W`。

## 当前实现的固定参数

联合阶段最多 35 次迭代，固定结构精修最多 20 次迭代；Ceres 的可用解不自动等同于完全收敛，需检查报告中的终止原因。
点云平面初始化使用 0.4m 体素、5cm 距离门槛和至少 3 扫描支持。
每扫描约 300 个抽样点，点到平面残差 sigma=3cm；取整步长可能产生略多于 300 个点，不能视为严格硬上限。
视觉重投影使用 Huber 2px，第二阶段使用 Huber 1.5px。
弱观测相机保留联合阶段结果，不插入里程计相对运动先验。
这些参数目前写在源码中。第二阶段的覆盖格网按原实验 1280×1088 尺寸写死，改变分辨率前应相应修改覆盖计算。
