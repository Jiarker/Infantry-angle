#ifndef _FIT_HPP_
#define _FIT_HPP_

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "time.h"
#include <filesystem>
#include <ceres/ceres.h>
#include <Eigen/Dense>

using namespace std;
using namespace cv;

typedef struct SpeedTime
{
    double angle_speed; // y
    double time;      // x

    SpeedTime(double speed = 0.0, double t = 0.0)
    {
        angle_speed = speed;
        time = t;
    }
} SpeedTime;

// 代价函数
// 构建代价函数的计算模型
// struct CURVE_FITTING_COST{
//     CURVE_FITTING_COST(Eigen::Vector3d x, double y) : _x(x), _y(y) {}

//     // 重载()，仿函数
//     template<typename T>
//     bool operator()(
//             const T *const abcd, // 模型参数，有4维
//             T *residual) const {
//         residual[0] = pow((T(_y) - abcd[0]*_x(0) - abcd[1]*_x(1) - abcd[2]*_x(2) - abcd[3]),2);
//         return true;
//     }

//     const Eigen::Vector3d _x;
//     const double _y;
// };


class newFitting
{
public:
    newFitting(){};
    ~newFitting()= default;

    bool run(vector<SpeedTime>& Fittingdata, double& w, Eigen::Vector4d& result);
    bool runCeres(vector<SpeedTime>& Fittingdata, double& w, Eigen::Vector4d& result);
private:
    double w_min = 1.884;
    double w_max = 2.000; 
    double dw = 0.001;
    bool fit(vector<SpeedTime> Fittingdata, double w, Eigen::Vector4d &output, double& error);

};

class DataWrite
{
public:
    DataWrite(){};
    ~DataWrite() = default;

    void countFilesInDirectory();

    void writeParam(double w, Eigen::Vector4d param);

    void writeToFile(SpeedTime data, int label);

private:
    string path = "../txt/";
    string filename;
    int fileCount = 0;
    ofstream txt;

};

#endif