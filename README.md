# MY_SLAM 项目

## 项目概况
本项目是WTU计科实训三大作业，是一个基于 C++ 的三维重建和 SLAM（Simultaneous Localization and Mapping）项目。它利用 RGB-D 数据进行实时的三维空间建模和相机定位。项目采用了现代化的 C++ 标准，结合了 OpenCV、Pangolin 和 Eigen 等流行库来实现高效的计算和可视化。

## 特点

- **实时性**：项目设计考虑了实时处理的需求。
- **模块化**：代码结构清晰，易于扩展和维护。
- **可视化**：通过 Pangolin 库提供直观的三维可视化。
- **高精度**：采用先进的视觉算法确保高精度的重建和定位。

## 依赖

- C++23 编译器（g++ 14.1.0）
- [Eigen3](http://eigen.tuxfamily.org)：用于线性代数运算。
- [OpenCV](https://opencv.org/)：用于图像处理和计算机视觉。
- [Pangolin](https://github.com/stevenlovegrove/Pangolin)：用于三维视觉和图形渲染。
- 编译工具 CMake。

## 安装和编译

1. 克隆项目到本地机器：
   ```sh
   git clone https://github.com/yourusername/MY_SLAM.git
   cd MY_SLAM
2. 运行编译生成的可执行文件：
   ```sh
    ./out/bin/slam_app

## 结构
    - src/：包含项目源代码。
    - include/：包含项目头文件。
    - Main/：包含主程序入口和相关逻辑。


## 运行效果
 - 本项目使用TUM数据集，需要使用其内部的associate.txt，若没有可使用数据集官方的脚本生成，数据集的路径可以在config.yaml中配置，还有像素大小，是否清除历史点等参数可以配置。confifg.yaml的地址可通过命令行参数给出
![Alt text](<img/2024-06-13 14-40-45屏幕截图.png>)

![Alt text](<img/2024-06-13 14-40-58屏幕截图.png>)

![Alt text](<img/2024-06-13 14-41-28屏幕截图.png>)

