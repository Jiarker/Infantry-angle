#include "Rune/Fitting.h"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "time.h"

using namespace std;
using namespace cv;

//spd = a * sin[w * (t + t0)] + (2.090 - a)
//积分后：angle = -(a / w) * cons[w * (t + t0)] + (2.090 - a) * t + (a / w) * cos(w * t)
//w:[1.884,2.000]
//T:[3.1415,3.3350]
//b = 2.090 - a
//a:[0.780,1.045]

//曲线拟合所用时间
//4s = 400 * 10ms

class newFitting
{
public:
    newFitting(){};
    ~newFitting()= default;

    bool run(vector<SpeedTime>& Fittingdata, double& w, double& P1, double& P2, double& P3);

private:
    double w_min = 1.884;
    double w_max = 2.000; 
    double dw = 0.001;
    bool fit(vector<SpeedTime> Fittingdata, double w, Eigen::Vector3d &output, double& error);

};

bool newFitting::fit(vector<SpeedTime> Fittingdata, double w, Eigen::Vector3d &output, double &error)
{
    if(Fittingdata.empty())
        return false;

    // X,y 赋值
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    Eigen::Vector3d N = Eigen::Vector3d::Zero();
    for(int i = 0; i < Fittingdata.size(); i++)
    {
        double x1i = sin(w * (double(Fittingdata[i].time)/1000 - double(Fittingdata[0].time)/1000));
        double x2i = cos(w * (double(Fittingdata[i].time)/1000 - double(Fittingdata[0].time)/1000));
        Eigen::Vector3d Xi(x1i, x2i, 1);
        M += Xi * Xi.transpose();
        double yi = Fittingdata[i].angle_speed;
        N += Xi * yi;
    }
    // 判断M是否有逆矩阵;为0,矩阵不可逆
    if(M.determinant() == 0)
        return 0;
    output = M.inverse() * N;
    // cout<<"output:"<<output<<endl;

    // 计算误差
    double temp_error = 0.0;
    for(int i = 0; i < Fittingdata.size(); i++)
    {
        double x1i = sin(w * (double(Fittingdata[i].time)/1000 - double(Fittingdata[0].time)/1000));
        double x2i = cos(w * (double(Fittingdata[i].time)/1000 - double(Fittingdata[0].time)/1000));           
        double yi = Fittingdata[i].angle_speed;
        double temp = x1i*output(0) + x2i*output(1) + output(2) - yi;
        temp_error += pow(temp,2);
    }
    error = temp_error;
    return true;
}


bool newFitting::run(vector<SpeedTime>& Fittingdata, double& w, double& P1, double& P2, double& P3)
{
    bool change_data = false;   // 拟合是否成功标志位
    double w_temp = this->w_min; 
    double w_best = w_min;
    double min_error = DBL_MAX;
    Eigen::Vector3d result;
    while(w_temp <= this->w_max)
    {
        Eigen::Vector3d output;
        double error;
        if(fit(Fittingdata, w_temp, output, error))
        {
            // cout<<"error:"<<error<<endl;
            if(min_error > error)
            {
                change_data = true;
                min_error = error;
                result = output;;
                w_best = w_temp;
            }
        }
        w_temp += this->dw;
    }

    if(!change_data)
    {
        cout<<"fit error"<<endl;
        return false;
    }

    cout<<"error:"<<min_error<<endl;
    w = w_best;
    cout<<"result:"<<result<<endl;
    P1 = result(0);
    P2 = result(1);
    P3 = result(2);
    return true;
}


int main()
{
    cv::RNG rng;
    double data_angle;
    uint32_t data_time = 0;
    uint32_t time_to_angle = 0;

    double a = 0.9;
    double b = 2.090 - a;
    double w = 1.884;
    double t0 = 1.8;

    double correct_angle = 0;

    //产生随机数
    srand(cv::getTickCount());
    a = (rand() %(1045 - 780 + 1) + 780) * 1.0 / 1000.0;
    b = 2.090 - a;
    w = (rand() % (2000 - 1884 + 1) + 1884) * 1.0 / 1000.0;
    t0 = (rand() % (3000 - 0 + 1) + 0) * 1.0 / 1000.0;

    double sum = 400;//fitting_data_w.size()超过400才能进行拟合

    // 产生数据
    vector<SpeedTime> fittingdata;
    uint32_t timestamp = 0; // 时间从0开始产生
    for(int i = 0; i < sum; i++)
    {
        double speed = a * sin(w*(double(timestamp)/1000+t0)) + b + (rand() % (3 - 0) + 1) / 10.0; // 生成0-0.5随机数
        fittingdata.push_back(SpeedTime(speed, timestamp));
        // cout<<"timestamp:"<<timestamp<<"\tspeed:"<<speed<<endl;
        timestamp += 10;
    }

    newFitting fitting;
    double w_fit=0,p1=0,p2=0,p3=0;
    double t1 = clock();    // clock()函数单位为微秒
    fitting.run(fittingdata, w_fit, p1, p2, p3);
    double t2 = clock();

    double p1_true = a * cos(w*t0);
    double p2_true = a * sin(w*t0);
    double p3_true = b;

    cout << "waste time:"<< t2 - t1 << endl;
    // cout<<"w:"<<w<<"\ta:"<<a<<"\tb:"<<b<<"\tt0:"<<t0<<endl;
    cout<<"w:"<<w<<"\tp1_true:"<<p1_true<<"\tp2_true:"<<p2_true<<"\tp3_true:"<<p3_true<<endl;
    cout<<"w_fit:"<<w_fit<<"\tp1:"<<p1<<"\tp2:"<<p2<<"\tp3:"<<p3<<endl;
    return 0;
}