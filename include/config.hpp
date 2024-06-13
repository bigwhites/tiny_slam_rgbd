#pragma once
#include <common_include.h>

class Config
{
public:
    // 创建 FileStorage 对象并打开 YAML 文件
    cv::FileStorage fs;
    double pointSizeCur;
    double pointSizeHis;
    string DATASET_ROOT;
    int WK_TIME;
    bool isDense;
    Config(const string &fileName) : fs(fileName, cv::FileStorage::READ)
    {
        fs["dataset_dir"] >> DATASET_ROOT;
        fs["waitTime"] >> WK_TIME;
        fs["pointSizeCur"] >> pointSizeCur;
        fs["pointSizeHis"] >> pointSizeHis;
        fs["dense"] >> isDense;
        println("read data:{},dense={}", DATASET_ROOT, isDense);
    }
    ~Config()
    {
        fs.release();
    }
};