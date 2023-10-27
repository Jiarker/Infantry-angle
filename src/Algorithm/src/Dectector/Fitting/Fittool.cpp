#include "Dectector/Fitting/Fittool.h"

namespace detector
{
    bool Judgement::Judge(double angle)
    {
        // 速度初筛选

        // 3o筛选
        
        if(angleJudge.size() < 15)
        {
            angleJudge.push_back(angle);
            judge_clear_num = 0;
            return true;
        }

        getN();
        getMean();
        getVariance();

        if(angle > mean + 3*standard_deviation || angle < mean - 3*standard_deviation)
        {
            solveBadData();
            return false;
        }
        
        // 确认为正确值

        angleJudge.push_back(angle);

        while (angleJudge.size() > 15)
            angleJudge.erase(angleJudge.begin());
        
        judge_clear_num = 0;
        return true;

    }

    void Judgement::resetJudge()
    {
        judge_clear_num = 0;
        angleJudge.clear();
    }

    void Judgement::getN()
    {
        this->n = angleJudge.size();
    }

    void Judgement::getMean()
    {
        double sum = 0;
        for(int i = 0; i < n; i++)
            sum += angleJudge[i];
        this->mean = sum / n;
    }

    void Judgement::getVariance()
    {
        double square_sum = 0;
        for(int i = 0; i < n; i++)
            square_sum += pow(angleJudge[i], 2);
        this->variance = (square_sum - n*pow(mean,2)) / n;
        // 防止标准差为0,设置标准差恒为0.1
        if(this->variance < 0.01)
            this->variance = 0.01;
            
        this->standard_deviation = pow(this->variance, 0.5);
    }

    void Judgement::solveBadData()
    {
        cout<<"bad data"<<endl;
        judge_clear_num++;
        if(judge_clear_num > 3)
        {
            judge_clear_num = 0;
            angleJudge.clear();
        }    
    }


    Fit::Fit()
    {
        cv::FileStorage fs("./src/Algorithm/configure/Detector/detector/rune_detector/Rune.xml", cv::FileStorage::READ);
        if(!fs.isOpened())
        {
            std::cout<<"open rune detect param fail"<<std::endl;
        }
        fs["delay_time"] >> delay_time;
        fs["save_txt"] >> save_txt;
        fs["print_result"] >> print_result;
        fs.release();
    }

    double Fit::run(vector<AngleTime> Fittingdata, double N)
    {        
        bool change_data = false;   // 拟合是否成功标志位
        double w_temp = this->w_min; 
        double w_best = this->w_min;
        double min_error = DBL_MAX;

        this->start_time = Fittingdata[0].time;

        cout<<"delta_angle:"<<Fittingdata[Fittingdata.size()-1].angle-Fittingdata[0].angle<<"\tdelta_time:"<<Fittingdata[Fittingdata.size()-1].time-Fittingdata[0].time<<endl;

        // for(int i = 0; i < Fittingdata.size(); i++)
        //     cout<<Fittingdata[i].angle<<endl;

        while(w_temp <= this->w_max)
        {
            Eigen::Vector4d output;
            double error;
            if(fitting(Fittingdata, w_temp, output, error))
            {
                if(min_error > error)
                {
                    change_data = true;
                    min_error = error;
                    
                    this->start_time = Fittingdata[0].time;
                    this->w = w_temp;
                    this->P1 = output(0);
                    this->P2 = output(1);
                    this->P3 = output(2);  
                    this->P4 = output(3);                                      
                }
            }
            w_temp += this->dw;
        }

        // 没有成功拟合
        if(!change_data)
        {
            cout<<"fit error"<<endl;
            
            // 若数据总量小于最大拟合数量N,或start_time未初始化,则给予start_time初值
            if(Fittingdata.size() < N || this->start_time < 0)
                this->start_time = Fittingdata[0].time;
        }

        cout<<"min_error:"<<min_error<<endl;

        // 返回旋转角度
        
        double t = Fittingdata[Fittingdata.size() - 1].time - start_time;
        cout<<"t:"<<t<<endl;

        if(this->print_result)
            cout<< "w:" << w << "\tP1:" << P1 << "\tP2:" << P2 << "\tP3:" << P3 << "\tP4:" << P4 << endl;   

        if(this->save_txt)
        {
            AngleTime predict = predictAngle(t);
            drawData(predict, Fittingdata[Fittingdata.size() - 1]);
        }

        return getRotateAngle(t);
    }

    void Fit::clear()
    {
        w = 1.9;
        P1 = 0.9;
        P2 = 0.0;
        P3 = 1.19;
        P4 = 0;        
    }

    bool Fit::fitting(vector<AngleTime> Fittingdata, double w, Eigen::Vector4d &output, double &error)
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
            double yi = Fittingdata[i].angle;
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

            double yi = Fittingdata[i].angle;

            double temp = x1i*output(0) + x2i*output(1) + x3i*output(2) + output(3) - yi;
            temp_error += pow(temp,2);
        }
        error = temp_error;
        return true;
    }

    double Fit::getRotateAngle(double t)
    {
        double temp1 = P1*(sin(w*(t+delay_time)) - sin(w*t));
        double temp2 = P2*(cos(w*(t+delay_time)) - sin(w*t));
        double temp3 = P3 * delay_time;       
        double rotate_angle = fabs(temp1 + temp2 + temp3);
        cout<<"rotate_angle:"<<rotate_angle<<endl;
        return rotate_angle;
    }

    AngleTime Fit::predictAngle(double t)
    {
        double predict_t = t + delay_time;
        double angle = P1*sin(w*predict_t) + P2*cos(w*predict_t) + P3*predict_t + P4; 
        double time = start_time + predict_t;
        return AngleTime(angle, time);
    }

    void Fit::drawData(AngleTime predict, AngleTime now)
    {        
        if(!have_file_count)
        {
            countFilesInDirectory();
            have_file_count = true;
        }

        if(!have_first_time)
        {
            first_time = min(predict.time, now.time);
            have_first_time = true;
        }

        txt.open(filename, ios::app);
        if (txt.is_open()) 
        {
            txt << predict.angle<< " " << predict.time - first_time << " " << now.angle<< " " << now.time - first_time << endl;
            txt.close();
        }
        else
        {
            std::cerr << "Unable to open the file.";
        }        
    } 

    void Fit::countFilesInDirectory()
    {
        this->fileCount = 0;
        for (const auto& entry : std::filesystem::directory_iterator(path)) 
        {
            if (entry.is_regular_file() || entry.is_directory()) {
                this->fileCount++;
            }
        }
        // 创建文件
        this->filename = path + to_string(fileCount) + ".txt";
    }

}