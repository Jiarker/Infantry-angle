/**
 * spd = (a * sin(w * (x_ + t)) + 2.090 - a)
 * 转化:spd = P1*sin(wt) + P2*cos(wt) + P3
 * 
 */

#ifndef FITTOOL_H
#define FITTOOL_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include "../../Base/rune_armor.hpp"
#include "../../Base/const.hpp"
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <filesystem>

namespace detector{

using namespace std;

typedef struct AngleTime
{
    double angle; // y
    double time;      // x

    AngleTime(double a = 0.0, double t = 0)
    {
        angle = a;
        time = t;
    }
} AngleTime;

class Judgement{
public:
    Judgement(){};
    ~Judgement() = default;

    /**
     * @brief 判断数据是否为坏值
     * 
    */
    bool Judge(double angle);

    /**
     * @brief 判断器重置
    */
    void resetJudge();

private:
    vector<double> angleJudge;          // 判断器数据集
    double n;
    double mean;
    double variance;
    double standard_deviation;
    int judge_clear_num = 0;            // 坏值累加器,若大于3,则重置判断器

    /**
     * @brief 得到判断器内数据集个数
    */
    void getN();

    /**
     * @brief 计算数据集的平均值
    */
    void getMean();

    /**
     * @brief 计算方差
    */
    void getVariance();

    /**
     * @brief 处理坏值
    */
    void solveBadData();
};

class Fit
{
public:
    Fit();
    ~Fit()= default;

    double delay_time;
    int save_txt;
    int print_result;

    double run(vector<AngleTime> Fittingdata, double N);

    void clear();

private:
    double w_min = 5.00;
    double w_max = 9.000; 
    double dw = 0.001;
    
    double w = 1.9;
    double P1 = 0.9;
    double P2 = 0.0;
    double P3 = 1.19;
    double P4 = 0;

    double start_time = -1.0;          // 角度变化的初值,不为拟合AngleTime的初始时间;成功拟合的情况下才改变

    string path = "./src/Algorithm/configure/Detector/Fitting/buff_state/";
    string filename;
    bool have_file_count = false;
    bool have_first_time =false;
    double first_time;
    int fileCount = 0;
    ofstream txt;

    bool fitting(vector<AngleTime> Fittingdata, double w, Eigen::Vector4d &output, double& error);

    double getRotateAngle(double t);

    AngleTime predictAngle(double t);

    void drawData(AngleTime predict, AngleTime now);

    void countFilesInDirectory();    
};

}

#endif