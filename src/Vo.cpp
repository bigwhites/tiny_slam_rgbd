#include <common_include.h>
#include <Vo.h>
#include <utils.h>
void Vo::init(const string &configFileList)
{
    this->atmVersion = 0;
    this->isRun = true;
    this->cfgPtr = make_unique<Config>(configFileList);
    K = (cv::Mat_<double>(3, 3) << 525.0, 0, 319.5, 0, 525.0, 239.5, 0, 0, 1);
    auto [rgb, depth] = readAssociationFile(format("{}/associate.txt", cfgPtr->DATASET_ROOT));
    rgbData = move(rgb);
    depthData = move(depth);
    pangolin::CreateWindowAndBind("SLAM-RGBD Rebuilt", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // 设置窗口背景为白色
    glClearColor(0.6f, 0.6f, 0.6f, 0.9f);
    // 创建一个面板用于添加GUI控件
    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
    // 添加复选框控件
    guiFollowCameraPtr = make_unique<pangolin::Var<bool>>("ui.Follow Camera", followCamera, true);
    guiShowKeyPointsPtr = make_unique<pangolin::Var<bool>>("ui.Show Points", showPoints, true);
    guiShowKeyFramesPtr = make_unique<pangolin::Var<bool>>("ui.Show KeyFrames", showKeyframes, true);
    guiOnlyShowCurFramesPtr = make_unique<pangolin::Var<bool>>("ui.Only CurFrames", showKeyframes, true);
    sCamera = make_unique<pangolin::OpenGlRenderState>(
        pangolin::ProjectionMatrix(640, 480, 500, 500, 320, 240, 0.1, 1000),
        pangolin::ModelViewLookAt(0, 1, 1, 0, 0, -1, pangolin::AxisY)); // 将相机位姿设置为窗口中心
    auto p = &pangolin::CreateDisplay()
                  .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -640.0f / 480.0f)
                  .SetHandler(new pangolin::Handler3D(ref(*sCamera)));
    dCamera.reset(p);
}

void Vo::processData()
{

    cv::namedWindow(WIN_NAME);
    println("开始读取数据");
    for (int i = 0; i < rgbData.size(); ++i)
    {
        Frame frame;
        unique_lock<mutex> lock(mtx);
        println("========================当前:{:.{}}%========================", (i * 1.0 / rgbData.size()) * 100, 4);
        conVar.wait(lock, [&]()
                    { return atmVersion < 1; }); // pangolin消费后

        frame.rgbImgCv = cv::imread(format("{}/{}", cfgPtr->DATASET_ROOT, rgbData[i].fileName), cv::IMREAD_UNCHANGED);
        frame.depthImgCv = cv::imread(format("{}/{}", cfgPtr->DATASET_ROOT, depthData[i].fileName), cv::IMREAD_UNCHANGED);
        if (frame.rgbImgCv.empty() || frame.depthImgCv.empty())
        {
            continue;
        }
        vector<cv::DMatch> matchesList;
        extractFeatures(frame);
        if (hasPreFrame)
        {
            matchesList = matchFeatures(preFrame, frame);
            println("匹配的数量为:{}", matchesList.size());
            auto prehepsPose = estimatePose(preFrame, frame, matchesList);
            this->cumulaPos = this->cumulaPos * (prehepsPose.inverse());
            cameraPoses.push_back(cumulaPos); // 加入累计pos
            this->addFrame(frame);
        }
        preFrame = frame;
        hasPreFrame = true;
        cv::Mat rgbWKPoints; // 带有key points
        cv::drawKeypoints(frame.rgbImgCv, frame.keypoints, rgbWKPoints);

        // 标记匹配的特征点为红色
        for (auto &match : matchesList)
        {
            cv::circle(rgbWKPoints, frame.keypoints[match.trainIdx].pt, 3,
                       cv::Scalar(0, 0, 255), -1);
        }
        // 标记未匹配的特征点为蓝色
        for (const auto &kp : frame.keypoints)
        {
            bool isMatched = false;
            for (auto &match : matchesList)
            {
                if (match.trainIdx == &kp - &frame.keypoints[0])
                {
                    isMatched = true;
                    break;
                }
            }
            if (!isMatched)
            {
                cv::circle(rgbWKPoints, kp.pt, 3, cv::Scalar(235, 205, 0), -1);
            }
        }

        cv::imshow(WIN_NAME, rgbWKPoints);
        atmVersion++;
        conVar.notify_one();
        cv::waitKey(cfgPtr->WK_TIME);
        println("*****Loop {} finish******", i);
    }
    isRun = false;
}

void Vo::runPangoLin()
{
    unique_lock<mutex> lock(mtx);
    conVar.wait(lock, [&]()
                { return atmVersion > 0 || !isRun; });
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    dCamera->Activate(ref(*sCamera));
    // 获取相机位姿参数
    if (hasPreFrame) // 有前面一帧的数据时候
    {
        if (*guiFollowCameraPtr)
        {
            Eigen::Matrix4d _Twc = cumulaPos.matrix();
            cout << "twc estimate:";
            cout << _Twc << endl;

            Eigen::Matrix4d Twc = cumulaPos.matrix();
            pangolin::OpenGlMatrix M;
            for (int i = 0; i < 4; i++)
            {
                M.m[4 * i] = Twc(0, i);
                M.m[4 * i + 1] = Twc(1, i);
                M.m[4 * i + 2] = Twc(2, i);
                M.m[4 * i + 3] = Twc(3, i);
            }
            sCamera->Follow(M);
        }

        if (int i = 0; *guiShowKeyFramesPtr)
        {
            if (*guiOnlyShowCurFramesPtr && cameraPoses.size() > 0)
            {
                auto v = *prev(cameraPoses.end());
                drawKeyFrame(v.translation(), {0.0, 0.0, 1.0}, 0.2); // 最后的
            }
            else
            {
                for (auto pose : cameraPoses)
                {
                    // 平移部分
                    Eigen::Vector3d posEiV3d = pose.translation();
                    if (i < cameraPoses.size() - 1) // 不是只绘制当前
                    {
                        drawKeyFrame(posEiV3d, {0.5, 0.45, 0.0});
                    }
                    else
                    {
                        drawKeyFrame(posEiV3d, {0.0, 0.0, 1.0}, 0.2); // 最后的
                    }
                    i++;
                }
            }
        }

        if (*guiShowKeyPointsPtr)
        {
            for (auto it = mapPoints.begin(); it != mapPoints.end(); ++it)
            {
                Eigen::Vector3d world_point = it->position;

                if (it < currMapPointsIter) // 历史地图点
                {
                    glColor3f(0.0, 1.0, 0.0); // 绘制地图点颜色
                    glPointSize(cfgPtr->pointSizeHis);
                }
                else // 当前
                {
                    glColor3f(1.0, 0.0, 0.0); // 绘制地图点颜色
                    glPointSize(cfgPtr->pointSizeCur);
                }
                // Negate the coordinates of the map point
                glBegin(GL_POINTS);
                glVertex3d(world_point[0], -world_point[1], -world_point[2]);
            }
            glEnd();
        }
    }
    atmVersion--;
    conVar.notify_one();
    pangolin::FinishFrame();
}
void Vo::applictionStart()
{

    println("\n开始运行!");

    thread th1([&]()
               { this->processData(); });

    while (!pangolin::ShouldQuit() && isRun)
    {
        runPangoLin();
    }
    th1.join();
    println("运行结束");
    this->sCamera.release();
    this->dCamera.release();
}

// 特征提取函数
void Vo::extractFeatures(Frame &frame)
{
    cv::Ptr<cv::ORB> orb = cv::ORB::create(2600);
    orb->detectAndCompute(frame.rgbImgCv, cv::noArray(), frame.keypoints, frame.descriptors);
}
// 特征匹配函数
vector<cv::DMatch> Vo::matchFeatures(const Frame &fPrev, const Frame &fCur)
{
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::DMatch> matches;
    matcher.match(fPrev.descriptors, fCur.descriptors, matches);
    return matches;
}

// 位姿估计算法
Eigen::Isometry3d Vo::estimatePose(const Frame &framePrev, const Frame &frame2, const std::vector<cv::DMatch> &matches)
{

    // 3d->2dpnp
    std::vector<cv::Point3f> pts3d;
    std::vector<cv::Point2f> pts2d;

    for (const auto &match : matches)
    {
        ushort d = framePrev.depthImgCv.at<ushort>(framePrev.keypoints[match.queryIdx].pt.y, framePrev.keypoints[match.queryIdx].pt.x);
        if (d == 0)
            continue; // 跳过无效深度值

        float dd = d / 5000.0;
        cv::Point2f p = framePrev.keypoints[match.queryIdx].pt;
        pts3d.push_back(cv::Point3f((p.x - K.at<double>(0, 2)) * dd / K.at<double>(0, 0), (p.y - K.at<double>(1, 2)) * dd / K.at<double>(1, 1), dd));
        pts2d.push_back(frame2.keypoints[match.trainIdx].pt);
    }

    cv::Mat rvec, tvec;
    cv::solvePnPRansac(pts3d, pts2d, K, cv::Mat(), rvec, tvec);

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d rotation;
    cv::cv2eigen(R, rotation);
    Eigen::Vector3d translation;
    cv::cv2eigen(tvec, translation);

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.rotate(rotation);
    pose.pretranslate(translation);

    return pose;
}

// 更新地图点
void Vo::addFrame(const Frame &frame)
{
    static bool b = true;
    if (!cfgPtr->isDense && b)
    {
        removeIndexElements(mapPoints);
    }
    b = !b;
    currMapPointsIter = mapPoints.end();
    for (const auto &kp : frame.keypoints)
    {
        ushort d = frame.depthImgCv.at<ushort>(kp.pt.y, kp.pt.x);
        if (d == 0)
            continue; // 防止ub
        float dd = d / 5000.0;
        Eigen::Vector3d point((kp.pt.x - K.at<double>(0, 2)) * dd / K.at<double>(0, 0),
                              (kp.pt.y - K.at<double>(1, 2)) * dd / K.at<double>(1, 1), dd);
        point = frame.pose * point; // 转换到世界坐标系
        if (&kp - &frame.keypoints[0] < frame.descriptors.rows)
        {
            mapPoints.emplace_back(MapPoint{point, frame.descriptors.row(&kp - &frame.keypoints[0])});
        }
    }
}

void Vo::drawKeyFrame(const Eigen::Vector3d &pos, const array<double, 3> &colors, double size)
{
    glColor3f(colors[0], colors[1], colors[2]);
    if (!abs(size - 0.1) < 1e-6) // 当前帧
    {
        // 开始绘制矩形的四条边
        glBegin(GL_LINE_LOOP);
        glVertex3d(pos[0] - size, -pos[1] - size, -pos[2] - size); // 左下角
        glVertex3d(pos[0] + size, -pos[1] - size, -pos[2] - size); // 右下角
        glVertex3d(pos[0] + size, -pos[1] + size, -pos[2] - size); // 右上角
        glVertex3d(pos[0] - size, -pos[1] + size, -pos[2] - size); // 左上角
        glEnd();                                                   // 结束绘制四条边
        // 开始绘制对角线
        glBegin(GL_LINES);
        glVertex3d(pos[0] - size, -pos[1] - size, -pos[2] - size); // 左下角
        glVertex3d(pos[0] + size, -pos[1] + size, -pos[2] - size); // 右上角
        glVertex3d(pos[0] + size, -pos[1] - size, -pos[2] - size); // 右下角
        glVertex3d(pos[0] - size, -pos[1] + size, -pos[2] - size); // 左上角
    }

    else
    {
        glBegin(GL_LINE_LOOP);
        glVertex3d(pos[0] - size, -pos[1] - size, -pos[2] - size); // 左下角
        glVertex3d(pos[0] + size, -pos[1] - size, -pos[2] - size); // 右下角
        glVertex3d(pos[0] + size, -pos[1] + size, -pos[2] - size); // 右上角
        glVertex3d(pos[0] - size, -pos[1] + size, -pos[2] - size); // 左上角
    }
    glEnd();
}
