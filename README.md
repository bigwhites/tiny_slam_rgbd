# SLAM 三维重建项目

这是一个使用 C++ 实现的 SLAM (Simultaneous Localization and Mapping) 三维重建项目，支持 RGB-D 数据输入。

## 特点

- 实时三维点云处理和特征提取
- 利用 ORB 特征进行快速匹配
- PnP (Perspective-n-Point) 算法进行位姿估计
- 利用 OpenGL 进行三维可视化

## 依赖

- C++11 或更高版本
- [Eigen](http://eigen.tuxfamily.org) - 线性代数库
- [OpenCV](https://opencv.org/) - 计算机视觉库
- [Pangolin](https://github.com/stevenlovegrove/Pangolin) - 3D 视觉和计算机图形库

## 安装

1. 克隆项目到本地机器