# Joint LVBA

在 Global-LVBA 基础上的实验性改进：**先联合优化雷达平面、视觉三维点与相机轨迹，再固定可靠结构精修视觉位姿**。用于离线改善已完成里程计的轨迹与地图一致性。

初始位姿可来自 Odin SDK；优化目标中没有 SDK 绝对位姿或相对运动先验。首帧位姿固定以确定坐标基准，雷达约束提供米制尺度。

## 相比上游的主要改动

**核心变化：从“用固定雷达地图辅助视觉 BA”，扩展为“雷达测量与视觉共同调整共享轨迹和结构”。**

原版 Global-LVBA 本来就有联合约束：其视觉 BA 同时优化相机位姿和视觉三维点，目标包含视觉重投影与视觉点到雷达平面的残差。
区别在于，原版这一阶段的雷达平面由已有点云地图估计后保持固定，且只让有有效平面关联的视觉点参与优化。
因此，本改进并不是简单地“给原版加上视觉和雷达两种 cost”。

| 对比项 | 原版视觉 BA 阶段 | 本版第一阶段 |
|---|---|---|
| 优化变量 | 相机位姿、视觉三维点；雷达平面固定 | 共享相机轨迹、视觉三维点、雷达平面法向与偏移 |
| 视觉约束 | 视觉三维点的重投影误差 | 视觉三维点的重投影误差 |
| 雷达对轨迹的作用 | 视觉点贴近固定雷达平面，通过视觉点间接约束位姿 | 去畸变后的雷达测量点通过共享轨迹变换，再计算点到平面残差，直接约束轨迹 |
| 视觉点与平面的联系 | 有有效雷达平面关联才参与这一阶段 | 有可靠平面关联时增加视觉点到平面约束；通过视觉一致性检查的非平面视觉点也可参与 |
| 雷达结构能否随视觉反馈调整 | 这一阶段不调整雷达平面参数 | 平面参数与轨迹、视觉点在同一个 Ceres 问题中一起调整 |

### 两阶段优化

1. **雷达视觉联合 BA。** 视觉重投影、雷达测量点到平面、具有可靠关联的视觉点到平面残差进入同一个问题。
   扫描时刻的位姿由邻近相机轨迹节点插值得到，雷达与视觉共同约束这些轨迹变量。
   这使雷达几何与视觉观测可以共同修正轨迹和结构。
2. **固定可靠结构的视觉位姿精修。** 按观测数量、重投影误差和视差筛选可靠三维点，固定这些点后只优化相机位姿。
   观测数量或图像覆盖不足的相机保留第一阶段结果，降低弱约束区域被错误更新的风险。

第二阶段只有视觉重投影残差，**最终位姿没有继续受到雷达残差的共同约束**，可能偏离第一阶段的雷达几何最优解。
因此准确描述是“联合 BA 后进行固定结构视觉精修”，并不表示最终阶段仍是雷达视觉联合优化，也不保证最终地图的雷达几何一致性自动提高。
该版本不联合估计扫描内运动、去畸变、外参、时间偏移或 IMU 状态。

### 工程适配

- 读取外部 COLMAP 数据库中的特征和几何验证内点，不依赖 SiftGPU/OpenGL。
- 统一 WXYZ 四元数及对应流形，接入鲁棒核和稀疏求解器；离线运行后退出。

联合求解与可靠结构筛选见 [`src/lvba_system.cpp`](src/lvba_system.cpp) 中的 `optimizeCameraPoses()`；
三种残差的实现见 [`include/joint_costs.hpp`](include/joint_costs.hpp)。
算法改动与工程适配的效果尚未通过逐项消融实验分离，后文实验数值是整套方案的结果。

这是一份已有实验版本的源码整理。当前 PDA 点云对照实验尚未纳入本仓库，也没有宣称彻底解决弱纹理、动态物体或局部退化。

## 构建

完整程序依赖 Linux、C++17、CMake ≥3.16、ROS 1/catkin、PCL、OpenCV、Eigen3、Ceres ≥2.1、SQLite3、fmt 和 Sophus。
已验证环境为 Ubuntu 24.04 上独立的 ROS 1 兼容运行时；这不表示 Ubuntu 24.04 原生 apt 仓库提供 ROS Noetic。

当前源码使用 `sophus_lib/se3.hpp`，与此前 Coco-LIC 运行时中的头文件一致。设置 `SOPHUS_INCLUDE_DIR` 为 **`sophus_lib` 的父目录**；仓库不打包第三方 Sophus 头文件。

在已激活 ROS/Ceres/PCL 环境的终端中：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DSOPHUS_INCLUDE_DIR=/path/to/directory/containing/sophus_lib
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

部分 Conda 编译器需要显式指定自身运行库，可追加
`-DJOINT_LVBA_LIBGCC=/path/to/env/lib/libgcc_s.so.1`。原生系统编译器通常无需设置。

只检查残差雅可比与坐标变换不变性时，不需要 ROS、PCL、OpenCV 或 Sophus：

```bash
cmake -S . -B build-costs -DJOINT_LVBA_BUILD_PIPELINE=OFF
cmake --build build-costs -j2
ctest --test-dir build-costs --output-on-failure
```

测试与实际优化共用 `include/joint_costs.hpp` 中的残差实现。

## 输入数据

完整坐标及位姿约定见 [docs/INPUT_FORMAT.md](docs/INPUT_FORMAT.md)。

```text
sequence/
├── all_image/
│   ├── 1661398632.022152.png
│   └── image_poses.txt
├── all_pcd_body/
│   ├── 1661398632.022152.pcd
│   └── lidar_poses.txt
├── Colmap/match.db
├── sdk_cameras.txt
├── depth/
├── reproj/
└── result/
```

图像须已正确去畸变，雷达点须预先逐点运动补偿。内部不联合估计扫描内运动、相机内参、外参或时间偏移。
COLMAP 数据库必须与输入图像、分辨率和内参一致，包含 `keypoints`、`images` 和 `two_view_geometries`。

`config/odin_legacy_406.yaml` 记录原实验的 **K≈406.8** 去畸变图像配置，不能直接用于后来 **K=550** 的图像。
复制配置并按当前数据修改内参、外参、数据库路径以及 `data_config/data_path`。绝对路径受支持，相对路径相对于仓库根目录。
COLMAP 像素中心相对 OpenCV 整数坐标有 +0.5 像素差异，配置必须使用与特征坐标一致的内参。

## 运行

启动一个专用于本次运行的 ROS master，在另一终端设置相同的 `ROS_MASTER_URI`：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11349
roscore -p 11349
```

```bash
export ROS_MASTER_URI=http://127.0.0.1:11349
rosparam load /absolute/path/to/your_config.yaml /joint_lvba
OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=1 \
  ./build/lidar_visual_ba __ns:=/joint_lvba
```

不要把两个不同实验的配置加载到同一个 ROS 参数命名空间。

主要输出位于数据目录：

- `joint_initial_camera_poses.txt`、`joint_camera_poses.txt`、`optimized_camera_poses.txt`：初始、联合优化及精修后的 world-to-camera 位姿。
- `joint_solver.txt`、`refine_solver.txt`：Ceres 完整求解报告。
- `reliable_structure.txt`、`refine_counts.txt`：可靠结构与每相机的观测覆盖统计。

## 实验记录与限制

原始 Odin 对照使用相同 1437 张图像、271105 个 3DGS 初始点、30k 训练步和 180 个留出图像视角。
SDK / 原 LVBA / 本两阶段方案的 PSNR 分别为 **26.467 / 27.975 / 28.874 dB**。
留出视角参与过轨迹与地图优化，因此不是完全独立的测试序列；这组数值不代表跨数据集结论。
更完整的协议和局限见 [docs/EXPERIMENT.md](docs/EXPERIMENT.md)。

优化准备总流程约 6.1 分钟，联合 Ceres 求解约 144.25 秒，视觉精修约 1.42 秒；不含更早的特征匹配和逐点去畸变。
该版本是离线后端，不能据此声称实时。

## 来源与许可

基于 [xuankuzcr/Global-LVBA](https://github.com/xuankuzcr/Global-LVBA)，上游提交
`10493a2b0f695e32db83dde49dad843a4826e090`，保留其 MIT 许可证及版权说明。
归属和依赖说明见 [NOTICE.md](NOTICE.md)。本仓库不是上游官方发布版本。
