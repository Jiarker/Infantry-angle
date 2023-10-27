/**
 * (a * sin(w * (x_ + t)) + 2.090 - a)
 */

#ifndef FITTING_H
#define FITTING_H

# include "../Fitting/Fittool.h"

namespace detector{

using namespace std;
using namespace cv;
using namespace base;

class AngleLinear
{
public:
    bool have_first_angle = false;
    double last_angle = 0;      
    int T = 0;

    double base_delta_angle = 2 * M_PI / 5;
    double d_angle =  5 / 180 * M_PI;

    AngleLinear(){};
    ~AngleLinear() = default;

    double run(double origin_angle);

    void clear();
};

class Fitting
{
public:
    vector<RuneArmor> armor_buffer;     // 存储一段时间内的未激活扇叶集合,用来计算速度
    vector<AngleTime> fitting_data;     // 拟合数据集

    double DELAY_TIME = 0.50;           // 预测时间，单位：秒
    int save_txt;
    int print_result;

private:

    AngleLinear linear;
    Judgement judge;
    Fit fit;

    // double DT = 0.01;                // 采样时间间隔，单位：秒
    double N = 80;                      // 角速度采样数
    double N_min = 20;                  // 开始确定旋转方向与拟合曲线的角速度最小采样数

    int DN = 1;                         // 逐差法测速度间距

    double start_time;                  // 拟合数据集中的第一个时间戳
    bool is_Inited = false;             // 大符拟合是否初始化
    bool is_direction_inited = false;   // 能量机关旋转方向初始化
    bool is_clockwise;                  // 顺时针

    double DT_clear = 0.8;              // 时间阈值，超过该阈值重置参数
    int DN_direction;                   // 逐差法确定旋转方向 

public:
    /**
     * @brief 初始化参数
    */
    Fitting();
    
    ~Fitting() = default;

    /**
     *  @brief  封装API
     */
    bool run(RuneArmor armor_1, vector<cv::Point2f> &nextPosition, TrackState armor_state, Mode rune_mode=Mode::RUNE);

protected:
    /**
     *  @brief  清空数据
     */
    void clearData();

    /**
     *  @brief  击打大符模式
     */
    bool runRune(RuneArmor armor_1, vector<cv::Point2f> &nextPosition, TrackState armor_state);

    /**
     *  @brief  击打小符模式
     */ 
    bool runNormalRune(RuneArmor armor_1, vector<cv::Point2f> &nextPosition, TrackState armor_state);

    /**
     *  @brief  根据旋转角度和半径计算下一点(装甲板四个角点)的像素位置
     *  @param  point   动点
     *  @param  org     原点
     *  @param  angle   旋转角度
     */
    cv::Point2f calNextPosition(cv::Point2f point, cv::Point2f org, float angle);

    /**
     *  @brief  根据状态处理数据
     *  @param  armor_1 处理完的装甲板
     *  @param  timestamp   原图像时间戳
     */
    bool processDataState(RuneArmor armor_1, TrackState armor_state);

    /**
     *  @brief  数据作插值处理
     */
    void pushFittingData(AngleTime new_data);

    /**
     *  @brief  判断能量机关旋转方向
     */
    void initDirection();

    /**
     * @brief 计算两点距离
    */
    float calDistance(cv::Point2f pt1, cv::Point2f pt2)
    {
        cv::Point2f dis = pt1 - pt2;
        return sqrt(pow(dis.x,2)+pow(dis.y,2));
    };
};

}

#endif