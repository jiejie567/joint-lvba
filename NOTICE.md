# Attribution and scope

This experimental derivative is based on [Global-LVBA](https://github.com/xuankuzcr/Global-LVBA),
upstream commit `10493a2b0f695e32db83dde49dad843a4826e090`.
The upstream MIT license and copyright notice are preserved in `LICENSE`.
`docs/UPSTREAM_README.md` is the original upstream README, not the build instructions for this derivative.

The derivative adds a shared-trajectory LiDAR/visual optimization problem and a
fixed-structure visual pose refinement stage, plus runtime adaptations for the
validated Odin experiment. It is not an official Global-LVBA release.

Eigen, Ceres, ROS, PCL, OpenCV, SQLite, fmt and Sophus are external dependencies,
subject to their own licenses. Sophus headers are not redistributed here.

The private repository contains source and configuration only. Dataset frames,
trajectories, maps, model weights and experiment videos are not included.
