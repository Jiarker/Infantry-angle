#include "../include/fit.hpp"

//spd = a * sin[w * (t + t0)] + (2.090 - a)
//积分后：angle = -(a / w) * cons[w * (t + t0)] + (2.090 - a) * t + (a / w) * cos(w * t)
//w:[1.884,2.000]
//T:[3.1415,3.3350]
//b = 2.090 - a
//a:[0.780,1.045]

//曲线拟合所用时间
//4s = 400 * 10ms

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

    int sum = 30;//fitting_data_w.size()超过400才能进行拟合
    int sum_after = 25;
    int rand_num = 2;
    double dt = 0.025;

    // 产生数据
    vector<SpeedTime> fittingdata;
    double timestamp = 0; // 时间从0开始产生
    for(int i = 0; i < sum; i++)
    {
        double t = timestamp;
        double angle = -(a/w)*cos(w*(t+t0)) + (2.090-a)*t + (a/w)*cos(w*t0) + (rand() % (rand_num - 0) + 1) / 10.0;
        fittingdata.push_back(SpeedTime(angle, timestamp));
        // cout<<"timestamp:"<<timestamp<<"\tspeed:"<<speed<<endl;
        timestamp += dt;
    }

    newFitting fitting;
    double w_fit=0;
    Eigen::Vector4d result;
    double t1 = clock();    // clock()函数单位为微秒
    fitting.run(fittingdata, w_fit, result);
    double t2 = clock();

    double p1_true = (a/w)*sin(w*t0);
    double p2_true = -(a/w)*cos(w*t0);
    double p3_true = b;
    double p4_true = (a/w)*cos(w*t0);

    cout << "waste time:"<< t2 - t1 << endl;
    // cout<<"w:"<<w<<"\ta:"<<a<<"\tb:"<<b<<"\tt0:"<<t0<<endl;
    cout<<"w:"<<w<<"\tp1_true:"<<p1_true<<"\tp2_true:"<<p2_true<<"\tp3_true:"<<p3_true<<"\tp4_true:"<<p4_true<<endl;
    cout<<"w_fit:"<<w_fit<<"\tp1:"<<result(0)<<"\tp2:"<<result(1)<<"\tp3:"<<result(2)<<"\tp4:"<<result(3)<<endl;
    
    // 绘图
    DataWrite txt;
    txt.countFilesInDirectory();
    // 写入参数
    txt.writeParam(w_fit, result);
    for(int i = 0; i < fittingdata.size(); i++)
        txt.writeToFile(fittingdata[i], 0);

    // 写入未参与拟合的参数
    for(int i = 0; i < sum_after; i++)
    {
        double t = timestamp;
        double angle = -(a/w)*cos(w*(t+t0)) + (2.090-a)*t + (a/w)*cos(w*t0) + (rand() % (rand_num - 0) + 1) / 10.0;
        txt.writeToFile(SpeedTime(angle, t), 1);
        timestamp += dt;        
    }
    
    return 0;
}