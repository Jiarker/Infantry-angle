#include "Dectector/Fitting/Fitting.h"

namespace detector
{
    double AngleLinear::run(double origin_angle)
    {
        double temp_angle = fmod(origin_angle, base_delta_angle);
      
        if(!have_first_angle)
        {
            last_angle = temp_angle;
            have_first_angle = true;
            return temp_angle;
        }

        temp_angle += T * base_delta_angle;

        // 角度突变
        double min_delta = DBL_MAX;
        int dT = 0;
        for(int i = -2; i < 3; i++)
        {
            double delta = fabs(temp_angle + i*base_delta_angle - last_angle);
            if(delta < min_delta)
            {
                dT = i;
                min_delta = delta;
            }
        }
        T += dT;
        temp_angle += dT * base_delta_angle;
        // cout<<"T:"<<T<<"\torigin_angle:"<<origin_angle<<"\ttrue_angle:"<<temp_angle<<"\tmin_delta:"<<min_delta<<"\tdT:"<<dT<<endl;
        last_angle = temp_angle;
        return temp_angle;
    }    

    void AngleLinear::clear()
    {
        have_first_angle = false;
        T = 0;
    }

    /*-----------Fitting-----------*/
    Fitting::Fitting(){}

    bool Fitting::run(base::RuneArmor armor_1, vector<cv::Point2f> &nextPosition, TrackState armor_state, base::Mode rune_mode)
    {
        bool result = false;
        switch (rune_mode){
            case base::Mode::NORMAL_RUNE:
                return runNormalRune(armor_1, nextPosition, armor_state);

            case base::Mode::RUNE:
                return runRune(armor_1, nextPosition, armor_state);
        
            default:
                return false;
        }

    }

    void Fitting::clearData()
    {
        cout << "Clear Fitting Data!" << endl;
        fit.clear();
        fitting_data.clear();
        judge.resetJudge();
        linear.clear();
        is_direction_inited = false;
    }

    bool Fitting::runRune(RuneArmor armor_1, vector<cv::Point2f> &nextPosition, TrackState armor_state)
    {
        if (!processDataState(armor_1, armor_state))
            return false;
 
        initDirection();
        if (is_direction_inited && fitting_data.size() > N_min)
        {
            nextPosition.clear();
            vector<cv::Point2f> pts;
            armor_1.getPoints(pts);            
            
            double delta = fit.run(fitting_data, N);            // 旋转角度
           
            for (int i = 0; i < 4; i++)
                nextPosition.push_back(calNextPosition(pts[i], armor_1.circle_center, delta));

            return true;             
        }
        return false;
    }

    bool Fitting::runNormalRune(RuneArmor armor_1, vector<cv::Point2f> &nextPosition, TrackState armor_state)
    {   
        // find the target
        if(!processDataState(armor_1, armor_state))
            return false;

        initDirection();

        if (is_direction_inited)
        {
            nextPosition.clear();
            vector<cv::Point2f> pts;
            armor_1.getPoints(pts);
            double delta = CV_PI / 3 * DELAY_TIME;
            for (int i = 0; i < 4; i++)
            {
                nextPosition.push_back(calNextPosition(pts[i], armor_1.circle_center, delta));
            }
            return true; 
        }

    }

    cv::Point2f Fitting::calNextPosition(cv::Point2f point, cv::Point2f org, float rotate_angle)
    {
        double radius = calDistance(point, org);
        cv::Point2f relative_point = point - org;                                         // 相对坐标
        double relative_angle = atan2(relative_point.y, relative_point.x);                // 与圆心所成角度
        double next_angle;

        if (is_clockwise) // 顺时针运动
        {
            next_angle = relative_angle + rotate_angle;
            if (next_angle > CV_PI)
                next_angle -= 2.0 * CV_PI;
        }
        else
        {
            next_angle = relative_angle - rotate_angle;
            if (next_angle < - CV_PI)
                next_angle += 2.0 * CV_PI;
        }

        return cv::Point2f(cos(next_angle) * radius, sin(next_angle) * radius) + org;
    }

    bool Fitting::processDataState(RuneArmor armor_1, TrackState armor_state)
    {
        // find the target
        if(armor_state == base::TrackState::DETECTING || armor_state == base::TrackState::TRACKING)
        {
            armor_1.angle = linear.run(armor_1.angle);
            if(judge.Judge(armor_1.angle));
                pushFittingData(AngleTime(armor_1.angle, armor_1.timestamp));
            
            while (fitting_data.size() > N)
                fitting_data.erase(fitting_data.begin());

            return true;
        }

        else if (armor_state == base::TrackState::LOST)
            clearData();

        return false;
    }

    void Fitting::pushFittingData(AngleTime new_data)
    {
        if (fitting_data.empty())
        {
            fitting_data.push_back(new_data);
            return;
        }       
        AngleTime flag_data = fitting_data[fitting_data.size()-1];
        double n = new_data.time - flag_data.time;

        // 时间差过长则清除数据;0.8为超参数,可调 
        if (n > DT_clear)
        {
            clearData();
            return;
        }
        fitting_data.push_back(new_data);

    }

    void Fitting::initDirection()
    {
        if (fitting_data.size() >= N_min)
        {
            int clock = 0, clock_inv = 0;
            for (int i = DN_direction; i < fitting_data.size(); i++)
            {
                if (fitting_data[i].angle - fitting_data[i-DN_direction].angle > 0)
                    clock++;
                else
                    clock_inv++;
            }
            is_direction_inited = true;
            is_clockwise = clock > clock_inv;
        }
        // cout<<"is_direction_inited:"<<is_direction_inited<<"\tis_clockwise:"<<is_clockwise<<endl;
    }

}
