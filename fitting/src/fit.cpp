#include "../include/fit.hpp"

bool newFitting::fit(vector<SpeedTime> Fittingdata, double w, Eigen::Vector4d &output, double &error)
{
    if(Fittingdata.empty())
        return false;

    // X,y 赋值
    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
    Eigen::Vector4d N = Eigen::Vector4d::Zero();
    for(int i = 0; i < Fittingdata.size(); i++)
    {
        double x1i = sin(w * (Fittingdata[i].time - Fittingdata[0].time));
        double x2i = cos(w * (Fittingdata[i].time - Fittingdata[0].time));
        double x3i = Fittingdata[i].time - Fittingdata[0].time;
        Eigen::Vector4d Xi(x1i, x2i, x3i, 1);
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
        double x1i = sin(w * (Fittingdata[i].time - Fittingdata[0].time));
        double x2i = cos(w * (Fittingdata[i].time - Fittingdata[0].time));           
        double x3i = Fittingdata[i].time - Fittingdata[0].time;
        
        double yi = Fittingdata[i].angle_speed;
        double temp = x1i*output(0) + x2i*output(1) + x3i*output(2) + output(3) - yi;
        temp_error += pow(temp,2);
    }
    error = temp_error;
    return true;
}


bool newFitting::run(vector<SpeedTime>& Fittingdata, double& w, Eigen::Vector4d& result)
{
    bool change_data = false;   // 拟合是否成功标志位
    double w_temp = this->w_min; 
    double w_best = w_min;
    double min_error = DBL_MAX;
    while(w_temp <= this->w_max)
    {
        Eigen::Vector4d output;
        double error;
        if(fit(Fittingdata, w_temp, output, error))
        {
            // cout<<"error:"<<error<<endl;
            if(min_error > error)
            {
                change_data = true;
                min_error = error;
                result = output;
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
    w = w_best;;
    return true;
}

// bool newFitting::runCeres(vector<SpeedTime>& Fittingdata, double& w, Eigen::Vector4d& result)
// {
//     // 构建最小二乘问题
//     w = 1.884;
//     double abcd[4];
//     ceres::Problem problem;
//     for(int i = 0; i < Fittingdata.size(); i++)
//     {
//         double time = Fittingdata[i].time;
//         double y = Fittingdata[i].angle_speed;
//         Eigen::Vector3d x;
//         x << sin(time*w), cos(time*w), time;
//         //添加误差项。使用自动求导，模板参数：误差类型、输出维度、输入维度、维数要与前面struct中一致
//         problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 4>(new CURVE_FITTING_COST(x,y)),nullptr,abcd);
//     }

//     //配置并运行求解器
//     ceres::Solver::Options options;     //定义配置项
//     options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  //配置增量方程的解法
//     options.minimizer_progress_to_stdout = true;    //输出到cout
//     ceres::Solver::Summary summary; //定义优化信息
//     // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();  //计时：求解开始时间
//     ceres::Solve(options, &problem, &summary);  //开始优化求解！
//     // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();  //计时：求解结束时间
//     // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);  //计算求解耗时

//     result << abcd[0],abcd[1],abcd[2],abcd[3];
//     return true;
// }

void DataWrite::countFilesInDirectory()
 {
    this->fileCount = 0;
    for (const auto& entry : std::filesystem::directory_iterator(path)) 
    {
        if (entry.is_regular_file() || entry.is_directory()) {
            this->fileCount++;
        }
    }
     // 创建文件夹
    this->filename = path + to_string(fileCount) + ".txt";
}

void DataWrite::writeParam(double w, Eigen::Vector4d param) 
{
    txt.open(filename, ios::app);
    if (txt.is_open()) 
    {
        txt << w << " " << param(0) << " " << param(1) << " " << param(2) << " "  << param(3) << endl;
        txt.close();
    }
    else
    {
        std::cerr << "Unable to open the file.";
    }
}

void DataWrite::writeToFile(SpeedTime data, int label) {
    txt.open(filename, ios::app);
    if (txt.is_open()) {
        txt << data.time << " " << data.angle_speed << " " << label << endl;
        txt.close();
    } else {
        std::cerr << "Unable to open the file.";
    }
}