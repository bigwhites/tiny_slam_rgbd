#pragma once

#include <common_include.h>
#include <DS.h>
#include <config.hpp>
class Vo
{
public:
    ~Vo() = default;
    Vo() = default;
    unique_ptr<Config> cfgPtr;
    Frame preFrame;           // 前一帧
    bool hasPreFrame = false; // 是否有前一帧

    vector<MapPoint> mapPoints; // 三维点
    decltype(mapPoints)::iterator currMapPointsIter;
    Eigen::Isometry3d cumulaPos = Eigen::Isometry3d::Identity(); // 累计的位姿
    vector<Eigen::Isometry3d> cameraPoses;                       // 用于保存相机位姿的容器
    cv::Mat K;                                                   // 相机内参

    // 保证线程同步
    atomic<int> atmVersion;
    condition_variable conVar;
    mutex mtx;
    atomic<bool> isRun;
    bool followCamera = true;
    bool showPoints = true;
    bool showKeyframes = true;
    bool onlyshowCurframes = true;
    unique_ptr<pangolin::Var<bool>> guiFollowCameraPtr;
    unique_ptr<pangolin::Var<bool>> guiShowKeyPointsPtr;
    unique_ptr<pangolin::Var<bool>> guiShowKeyFramesPtr;
    unique_ptr<pangolin::Var<bool>> guiOnlyShowCurFramesPtr;
    Files rgbData, depthData;

    unique_ptr<pangolin::OpenGlRenderState> sCamera;
    unique_ptr<pangolin::View> dCamera;

    // 初始化
    void init(const string &configFileList);
    // 入口
    void applictionStart();
    // 计算线程
    void processData();
    // 可视化线程（主线程）
    void runPangoLin();

    // 提取一帧的特征点
    void extractFeatures(Frame &frame);
    // 前后匹配特征点
    vector<cv::DMatch> matchFeatures(const Frame &fPrev, const Frame &fCur);

    // 估计相机的位姿
    Eigen::Isometry3d estimatePose(
        const Frame &framePrev, const Frame &frame2,
        const vector<cv::DMatch> &matches);

    // 加入地图点
    void addFrame(const Frame &frame);

    // 画出相机位置
    void drawKeyFrame(const Eigen::Vector3d &pos, const array<double, 3> &colors, double size = 0.1);
};
