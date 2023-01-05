#include "selector/Selector.hpp"

namespace wmj
{   
/*-----------------------------------------------------------------初始化-----------------------------------------------------------------*/
    //初始化筛选器参数类
    SelectorParam::SelectorParam()
    {
        setParam(wmj::SelectorMode::INFANTRY);
    }
    SelectorParam::SelectorParam(SelectorMode a_selector_mode)
    {
        setParam(a_selector_mode);
    }
    SelectorParam::~SelectorParam(){}
    //初始化筛选器类
    Selector::Selector()
    {
        m_param = std::make_shared<wmj::SelectorParam>();
        m_mode = wmj::ROBO_STATE::STATE_ARMOR;
        m_state = SELECTOR_SEARCHING;
        m_deep_roi_state = wmj::DeepROISizeState::ROI_BIG;
        m_tracklost = 0;

        m_return_roi_left = m_param->m_camera_resolution;
        m_return_roi_right = m_param->m_camera_resolution;

        m_search_center_lost = 0;
        m_id_search2track = 0;
    }

    Selector::~Selector(){}
   
   //初始化参数
    void SelectorParam::setParam(SelectorMode a_selector_mode)
    {
        cv::FileStorage fs("../libVision/Select/ArmorSelector.yaml", cv::FileStorage::READ);

        fs["Selector"]["camera_resolution"]             >> m_camera_resolution;             // 相机分辨率
        fs["Selector"]["aim_point_dis"]                 >> m_aim_point_dis;                 // 左右目准心距离
        
        fs["Selector"]["max_second_width_diff"]         >> m_max_second_width_diff;         // 次要目标与主要目标最大距离差
        fs["Selector"]["min_second_width_diff"]         >> m_min_second_width_diff;         // 次要目标与主要目标最小距离差
        fs["Selector"]["max_second_height_diff"]        >> m_max_second_height_diff;        // 次要目标与主要目标最大高度差

        fs["Selector"]["max_compare_ratio_diff"]        >> m_max_compare_ratio_diff;        // 两装甲板相似最大面积比差
        fs["Selector"]["max_compare_width_diff"]        >> m_max_compare_width_diff;        // 两装甲板相似最大宽度差
        fs["Selector"]["max_compare_height_diff"]       >> m_max_compare_height_diff;       // 两装甲板相似最大高度差
        fs["Selector"]["max_compare_center_diff"]       >> m_max_compare_center_diff;       // 两装甲板相似最大最大视差比例差
        fs["Selector"]["max_compare_center_distance"]   >> m_max_compare_center_distance;   // 两装甲板相似最小视差比例差
        fs["Selector"]["max_compare_tiltangle_diff"]    >> m_max_compare_tiltangle_diff;    // 两装甲板相似最大倾斜角差
        fs["Selector"]["max_compare_distance_diff"]     >> m_max_compare_distance_diff;     // 两装甲板相似最大测距差

        fs["Selector"]["max_mono_distance"]             >> m_max_mono_distance;             // 单目距离限制  
        fs["Selector"]["max_track_yaw_angle"]           >> m_max_track_yaw_angle;           // 最大追踪yaw angle  
        fs["Selector"]["min_switch_yaw_angle"]          >> m_min_switch_yaw_angle;          // 最小同车切换yaw angle

        fs["Selector"]["max_searching_center_diff"]     >> m_max_searching_center_diff;     // 准心优先阈值
        
        fs["Selector"]["max_track_lost"]                >> m_max_track_lost;                // 跟踪最大丢弃数
        fs["Selector"]["max_center_searching"]          >> m_max_center_searching;          // 操作手最大信任次数
        fs["Selector"]["max_id_search2track"]           >> m_max_id_search2track;           // 确认id中间状态帧数

        switch (a_selector_mode)
        {
        case INFANTRY:
            fs["Infantry"]["armor_priority"]                >> m_armor_priority;                // 装甲板优先级序列
            
            fs["Infantry"]["roi_up_ratio"]                  >> m_roi_up_ratio;                  // roi识别区域向上扩展原高度倍数
            fs["Infantry"]["roi_down_ratio"]                >> m_roi_down_ratio;                // roi识别区域向下扩展原高度倍数
            fs["Infantry"]["roi_left_ratio"]                >> m_roi_left_ratio;                // roi识别区域向左扩展原高度倍数
            fs["Infantry"]["roi_right_ratio"]               >> m_roi_right_ratio;               // roi识别区域向右扩展原高度倍数

            fs["Infantry"]["sentry_roi_up_ratio"]           >> m_sentry_roi_up_ratio;           // 哨兵模式roi识别区域向上扩展原高度倍数
            fs["Infantry"]["sentry_roi_down_ratio"]         >> m_sentry_roi_down_ratio;         // 哨兵模式roi识别区域向下扩展原高度倍数
            fs["Infantry"]["sentry_roi_left_ratio"]         >> m_sentry_roi_left_ratio;         // 哨兵模式roi识别区域向左扩展原高度倍数
            fs["Infantry"]["sentry_roi_right_ratio"]        >> m_sentry_roi_right_ratio;        // 哨兵模式roi识别区域向右扩展原高度倍数
            
            fs["Infantry"]["roi_lost_mul"]                  >> m_roi_lost_mul;                  // 丢弃时roi扩大倍率
            break;
        case HERO:
            fs["Hero"]["armor_priority"]                    >> m_armor_priority;                // 装甲板优先级序列
            
            fs["Hero"]["roi_up_ratio"]                      >> m_roi_up_ratio;                  // roi识别区域向上扩展原高度倍数
            fs["Hero"]["roi_down_ratio"]                    >> m_roi_down_ratio;                // roi识别区域向下扩展原高度倍数
            fs["Hero"]["roi_left_ratio"]                    >> m_roi_left_ratio;                // roi识别区域向左扩展原高度倍数
            fs["Hero"]["roi_right_ratio"]                   >> m_roi_right_ratio;               // roi识别区域向右扩展原高度倍数

            fs["Hero"]["sentry_roi_up_ratio"]               >> m_sentry_roi_up_ratio;           // 哨兵模式roi识别区域向上扩展原高度倍数
            fs["Hero"]["sentry_roi_down_ratio"]             >> m_sentry_roi_down_ratio;         // 哨兵模式roi识别区域向下扩展原高度倍数
            fs["Hero"]["sentry_roi_left_ratio"]             >> m_sentry_roi_left_ratio;         // 哨兵模式roi识别区域向左扩展原高度倍数
            fs["Hero"]["sentry_roi_right_ratio"]            >> m_sentry_roi_right_ratio;        // 哨兵模式roi识别区域向右扩展原高度倍数
            
            fs["Hero"]["roi_lost_mul"]                      >> m_roi_lost_mul;                  // 丢弃时roi扩大倍率
            break;
        case SENTRY:
            fs["Sentry"]["armor_priority"]                  >> m_armor_priority;                // 装甲板优先级序列
            
            fs["Sentry"]["roi_dis"]                         >> m_roi_dis;                       // 哨兵使用roi距离阶段阈值
            fs["Sentry"]["roi_mul"]                         >> m_roi_mul;                       // 哨兵使用roi距离阈值对应倍率

            fs["Sentry"]["roi_up_ratio"]                    >> m_roi_up_ratio;                  // roi识别区域向上扩展原高度倍数
            fs["Sentry"]["roi_down_ratio"]                  >> m_roi_down_ratio;                // roi识别区域向下扩展原高度倍数
            fs["Sentry"]["roi_left_ratio"]                  >> m_roi_left_ratio;                // roi识别区域向左扩展原高度倍数
            fs["Sentry"]["roi_right_ratio"]                 >> m_roi_right_ratio;               // roi识别区域向右扩展原高度倍数

            fs["Sentry"]["roi_lost_mul"]                    >> m_roi_lost_mul;                  // 丢弃时roi扩大倍率
            break;
        default:
            break;
        }
    }
    
    bool Selector::SetDeepRoiSize(cv::Size2i deep_roi_size)
    {
        m_param->m_deep_roi_size = deep_roi_size;
	    double rate = (double)deep_roi_size.height / (double)deep_roi_size.width;
	    m_param->m_deep_default_roi = cv::Rect2d
            (0,
            (m_param->m_camera_resolution.height - m_param->m_camera_resolution.width * rate) / 2,
            m_param->m_camera_resolution.width,
            (m_param->m_camera_resolution.width * rate)); 

        m_return_roi_left = m_param -> m_deep_default_roi;
        m_return_roi_right = m_param -> m_deep_default_roi;
        
	    return true;
    }
/*-----------------------------------------------------------------结果回传----------------------------------------------------------------*/
    // 回传左目ROI区域
    cv::Rect2d Selector::getLeftROI()
    {
        return m_return_roi_left;
    }
    // 回传右目ROI区域
    cv::Rect2d Selector::getRightROI()
    {
        return m_return_roi_right;
    }

    // 回传主要目标
    Armor Selector::getBestArmor()
    {
        return m_bestarmor;
    }
    // 回传次要目标
    Armor Selector::getSecondArmor()
    {
        return m_secondarmor;
    }
    // 回传是否发生了同车切换
    bool Selector::getSwitchInSameId()
    {
        return m_same_id_switch;
    }

    //输出识别结果
    void Selector::OutputResult(Armor a_armor)
    {
        switch(a_armor.m_id)
        {
            case 0:
                std::cout << "主要目标：数字识别未成功" << std::endl;
                break;
            case 1:
                std::cout << "主要目标：筛选结果为英雄" << std::endl;
                break;
            case 2:
                std::cout << "主要目标：筛选结果为工程" << std::endl;
                break;
            case 3:
                std::cout << "主要目标：筛选结果为3号步兵" << std::endl;
                break;
            case 4:
                std::cout << "主要目标：筛选结果为4号步兵" << std::endl;
                break;
            case 5:
                std::cout << "主要目标：筛选结果为5号步兵" << std::endl;
                break;
            case 7:
                std::cout << "主要目标：筛选结果为哨兵" << std::endl;
                break;
            case 8:
                std::cout << "主要目标：筛选结果为基地" << std::endl;
                break;
            case 11:
                std::cout << "主要目标：筛选结果为前哨站" << std::endl;
                break;
        }
    }

/*----------------------------------------------------------------筛选条件判断---------------------------------------------------------------*/
    // position异常值（nan、inf）处理
    void Selector::PositionErrorHandling()
    {
        for (auto it = m_armors.begin(); it != m_armors.end();)
        {
            if (isinf(it->m_position.x) || isnan(it->m_position.x) ||
                isinf(it->m_position.y) || isnan(it->m_position.y) ||
                isinf(it->m_position.z) || isnan(it->m_position.z)
                )
            {
                std::cout << _lightred("Error data: inf or nan") << std::endl;
                it = m_armors.erase(it);
            }
            else
            {
                ++it;
            }
        }
        // for (auto armor : m_armors)
        // {
        //     std::cout << "armor: " << armor.m_position << " " << armor.m_yaw_angle << std::endl;
        // }
    }
    
    // 得到装甲板中心距离差值
    double Selector::GetDistance(cv::Point2f a_center,cv::Point2f b_center)
    {
        return sqrt(pow(a_center.x - b_center.x,2) + pow(a_center.y - b_center.y,2));
    }

    // 解算绝对距离
    double Selector::GetDistance(cv::Point3f a_position)
    {
        return std::sqrt(std::pow(a_position.x/1000, 2) 
                         + std::pow(a_position.y/1000, 2) 
                         + std::pow(a_position.z/1000, 2));
    }
    
    // 得到装甲板优先级
    int Selector::GetArmorPriority(int id )
    {
        for (int i = 0; i < m_param->m_armor_priority.size(); i++)
        {
            for (int j = 0; j < m_param->m_armor_priority[i].size();j++)
            {
                if(id == m_param->m_armor_priority[i][j])
                    return i;
            }
        }
        return m_param->m_armor_priority.size();
    }

    // 根据准心距离排序
    void Selector::SortByCenter()
    {
        sort(m_armors.begin(),m_armors.end(),[&](const Armor &a, const Armor &b)
        {
            double a_center_diff = GetDistance(a.m_center, cv::Point2f(660, 552));
            double b_center_diff = GetDistance(b.m_center, cv::Point2f(660, 552));
            return a_center_diff < b_center_diff;
        });
    }

    // 单目识别情况下距离限制（单目过远标记为未识别）
    bool Selector::MonoDisLimit(Armor a)
    {
        if(!double_use)
        {
            return true;
        }

        float armorDis = GetDistance(a.m_position);

        if(a.m_detectedtype != 2 && armorDis > m_param->m_max_mono_distance)
        {
            // std::cout <<_lightpurple("mono_distance:  ") << armorDis << std::endl;
            return false;
        }
        return true;
    }

    // 次要目标判断
    bool Selector::SecondJudge(Armor best, Armor second)
    {
        double second_width_diff = abs(best.m_center.x - second.m_center.x) / ((best.m_width + second.m_width) / 2);
        double second_height_diff = abs(best.m_center.y - second.m_center.y) / ((best.m_height + second.m_height) / 2);
        
        // std::cout <<_lightpurple("second_width_diff:  ") << second_width_diff << std::endl;
        // std::cout <<_lightpurple("second_height_diff:  ") << second_height_diff << std::endl;

        if( second_width_diff > m_param->m_max_second_width_diff || 
            second_width_diff < m_param->m_min_second_width_diff ||
            second_height_diff > m_param->m_max_second_height_diff )
            {
                return false;
            }
        return true;
    }
    // 判断装甲板是否相似
    double Selector::Compare(Armor a, Armor b)
    {
        double ratio_diff = a.m_ratio > b.m_ratio ? a.m_ratio / b.m_ratio : b.m_ratio / a.m_ratio;

        double width_diff = a.m_width > b.m_width ? a.m_width / b.m_width : b.m_width / a.m_width;

        double height_diff = a.m_height > b.m_height ? a.m_height / b.m_height : b.m_height / a.m_height;

        double center_diff = abs(a.m_center.x - b.m_center.x) / b.m_width;

        double tiltangle_diff = abs(a.m_tiltAngle - b.m_tiltAngle);

        double distance_diff = abs(GetDistance(a.m_position) - GetDistance(b.m_position));

        //double center_distance = GetDistance(a.m_center, b.m_center);
        
        // std::cout <<_lightpurple("ratio_diff:  ") << ratio_diff << std::endl;
        // std::cout <<_lightpurple("width_diff:  ") << width_diff << std::endl;
        // std::cout <<_lightpurple("height_diff:  ") << height_diff << std::endl;
        // std::cout <<_lightpurple("tiltangle_diff:  ") << tiltangle_diff << std::endl;
        // std::cout <<_lightpurple("center_diff:  ") << center_diff << std::endl;
        // std::cout <<_lightpurple("distance_a:  ") << GetDistance(a.m_position) << std::endl;
        // std::cout <<_lightpurple("distance_b:  ") << GetDistance(b.m_position) << std::endl;
        // std::cout <<_lightpurple("distance_diff:  ") << distance_diff << std::endl;
        // std::cout <<_lightpurple("m_id:  ") << a.m_id << std::endl;

        if(ratio_diff > m_param->m_max_compare_ratio_diff || 
           width_diff > m_param->m_max_compare_width_diff || 
           height_diff > m_param->m_max_compare_height_diff)
            return -1;
         
        if(distance_diff > m_param->m_max_compare_distance_diff ||
           tiltangle_diff > m_param->m_max_compare_tiltangle_diff)
            return -1;
        if (center_diff > m_param->m_max_compare_center_diff)
            return -1;

        return ratio_diff + width_diff + height_diff + center_diff*2 + tiltangle_diff/3 + distance_diff;
    }

    void Selector::ChooseBestInSameId(Armor track, Armor second)
    {
        // std::cout << "main: " << track.m_yaw_angle << std::endl;
        // std::cout << "second: " << second.m_yaw_angle << std::endl;

        // if(track.m_yaw_angle > m_param->m_max_track_yaw_angle && 
        //    second.m_yaw_angle < m_param->m_min_switch_yaw_angle)
        //     updateFinalArmor(second, track);
        // else
            updateFinalArmor(track, second);
    }

/*----------------------------------------------------------------ROI设置----------------------------------------------------------------*/
    //得到返回roi区域
    void Selector::SetROI()
    {
        cv::Rect2d roi;
        double m_roi_value = m_param->m_roi_lost_mul[m_tracklost];

#ifdef USE_DEEP
            // std::cout << "DEEP SELECT" << std::endl;
            if (m_bestarmor.m_armor_type == wmj::ARMORTYPE::ARMOR_NONE) 
            {
                m_deep_roi_state = wmj::DeepROISizeState::ROI_BIG;
                m_return_roi_left = m_param->m_deep_default_roi;          //roi返回为中央roi
                m_return_roi_right =  m_param->m_deep_default_roi;        //roi返回为中央roi
                return;
            }
            else if(m_bestarmor.m_height * 7.5 > m_param->m_deep_roi_size.height || m_bestarmor.m_rect.width > m_param->m_deep_roi_size.width)
            {
                m_deep_roi_state = wmj::DeepROISizeState::ROI_BIG;
                roi = m_param->m_deep_default_roi; 
                roi.y = m_bestarmor.m_rect.y + (m_bestarmor.m_rect.height - roi.height) / 2.0;
                roi.y = std::max(std::min((float)roi.y, (float)(m_param->m_camera_resolution.height - 1 - m_param->m_deep_default_roi.height)), 0.f);
            }
            else if(m_bestarmor.m_height * 8.7 > m_param->m_deep_roi_size.height || m_bestarmor.m_rect.width > m_param->m_deep_roi_size.width)
            {
                if(m_deep_roi_state == wmj::DeepROISizeState::ROI_SMALL)
                {
                    roi = m_bestarmor.m_rect;
                    roi.y += (roi.height - m_param->m_deep_roi_size.height) / 2.0;
                    roi.height = m_param->m_deep_roi_size.height;
                    roi.x += (roi.width - m_param->m_deep_roi_size.width) / 2.0;
                    roi.width = m_param->m_deep_roi_size.width;
                    roi.x = std::max(std::min((float)roi.x, (float)(m_param->m_camera_resolution.width - 1 - m_param->m_deep_roi_size.width)), 0.f);
                    roi.y = std::max(std::min((float)roi.y, (float)(m_param->m_camera_resolution.height - 1 - m_param->m_deep_roi_size.height)), 0.f);
                }
                else
                {
                    roi = m_param->m_deep_default_roi; 
                    roi.y = m_bestarmor.m_rect.y + (m_bestarmor.m_rect.height - roi.height) / 2.0;
                    roi.y = std::max(std::min((float)roi.y, (float)(m_param->m_camera_resolution.height - 1 - m_param->m_deep_default_roi.height)), 0.f);
                }
            }
            else
            {
                m_deep_roi_state = wmj::DeepROISizeState::ROI_SMALL;
                roi = m_bestarmor.m_rect;
                roi.y += (roi.height - m_param->m_deep_roi_size.height) / 2.0;
                roi.height = m_param->m_deep_roi_size.height;
                roi.x += (roi.width - m_param->m_deep_roi_size.width) / 2.0;
                roi.width = m_param->m_deep_roi_size.width;
                roi.x = std::max(std::min((float)roi.x, (float)(m_param->m_camera_resolution.width - 1 - m_param->m_deep_roi_size.width)), 0.f);
                roi.y = std::max(std::min((float)roi.y, (float)(m_param->m_camera_resolution.height - 1 - m_param->m_deep_roi_size.height)), 0.f);
            }

            m_return_roi_right = roi;
            m_return_roi_left = roi;
            return;

#else
            if (m_bestarmor.m_armor_type == wmj::ARMORTYPE::ARMOR_NONE)
            {
                m_return_roi_left = m_param->m_camera_resolution;        //roi返回为全图
                m_return_roi_right =  m_param->m_camera_resolution;      //roi返回为全图
                return;
            }

            //如果为哨兵模式则返回一个轨道狭长长矩形的roi区域
            if(m_mode == wmj::ROBO_STATE::STATE_DARK)
            {
                roi = m_bestarmor.m_rect;
                roi.y -= roi.height * m_param->m_sentry_roi_up_ratio * m_roi_value;
                roi.height *= 1.0 + m_param->m_sentry_roi_up_ratio * m_roi_value + m_param->m_sentry_roi_down_ratio * m_roi_value;
                roi.x -= roi.width * m_param->m_sentry_roi_left_ratio * m_roi_value;
                roi.width *= 1.0 + m_param->m_sentry_roi_left_ratio * m_roi_value + m_param->m_sentry_roi_right_ratio * m_roi_value;
                roi &= m_param->m_camera_resolution;      //防止越界 
            }
            //如果为反陀螺和常规模式，则返回一个全车装甲板的矩形roi区域
            else
            {
                roi = m_bestarmor.m_rect;
                roi.y -= roi.height * m_param->m_roi_up_ratio * m_roi_value;
                roi.height *= 1.0 + m_param->m_roi_up_ratio * m_roi_value + m_param->m_roi_down_ratio * m_roi_value;
                roi.x -= roi.width * m_param->m_roi_left_ratio * m_roi_value;
                roi.width *= 1.0 + m_param->m_roi_left_ratio * m_roi_value + m_param->m_roi_right_ratio* m_roi_value;
                roi &= m_param->m_camera_resolution;      //防止越界
            }
            //双目或左目
            if(m_bestarmor.m_detectedtype == 0 || 2)
            {
                m_return_roi_left = roi;
                m_return_roi_right = m_return_roi_left;
                m_return_roi_right.x -= m_param->m_aim_point_dis;
                m_return_roi_right &= m_param->m_camera_resolution;
            }
            //右目
            else
            {
                m_return_roi_right = roi;
                m_return_roi_left = roi;
                m_return_roi_left.x += m_param->m_aim_point_dis;
                m_return_roi_right &= m_param->m_camera_resolution;
            }
#endif
        return;
    }
/*-----------------------------------------------------------------筛选流程----------------------------------------------------------------*/
    // 初始化追踪id
    void Selector::TrackInit()
    {
        m_track_id = -1;
    }
    // 设置装甲id，用于避免id误识别
    void Selector::updateArmorId(Armor a)
    {
        if(m_track_id != -1)
            a.m_id = m_track_id;
    }
    // 设置装甲id，用于避免id误识别
    void Selector::updateArmorId(Armor a,Armor b)
    {
        if(m_track_id != -1)
        {
            a.m_id = m_track_id;
            b.m_id = m_track_id;
        }
    }

    // 更新筛选结果(结果为空)
    void Selector::updateFinalArmor()
    {
        m_bestarmor.m_armor_type = wmj::ARMORTYPE::ARMOR_NONE;
        m_secondarmor.m_armor_type = wmj::ARMORTYPE::ARMOR_NONE;
        m_prebestarmor = m_bestarmor;
        m_presecondarmor = m_secondarmor;
        SetROI();
        return;
    }
    // 更新筛选结果(有主要目标无次要目标)
    void Selector::updateFinalArmor(Armor bestarmor)
    {
        m_bestarmor = bestarmor;
        m_secondarmor.m_armor_type = wmj::ARMORTYPE::ARMOR_NONE;
        m_prebestarmor = m_bestarmor;
        m_presecondarmor = m_secondarmor;
        SetROI();
        updateArmorId(bestarmor);
        return;
    }
    // 更新筛选结果(主要次要目标均存在)
    void Selector::updateFinalArmor(Armor bestarmor,Armor secondarmor)
    {
        m_bestarmor = bestarmor;
        m_secondarmor = secondarmor;
        m_prebestarmor = m_bestarmor;
        m_presecondarmor = m_secondarmor;
        SetROI();
        updateArmorId(bestarmor,secondarmor);
        return;
    }

    void Selector::ConfirmTrackId(Armor track)
    {
        int now_track_id = -1;
        
        // 确认锁id状态
        if(m_track_id != -1)
            now_track_id = m_track_id;
        else
            now_track_id = m_prebestarmor.m_id;

        if ( m_track_id != m_prebestarmor.m_id && 
             track.m_id == m_prebestarmor.m_id)
        {
            m_id_search2track++;
            // 追踪id更新
            if (m_id_search2track > m_param->m_max_id_search2track)
            {
                m_track_id = m_prebestarmor.m_id; 
                std::cout << _lightred("update id!") << std::endl;
                m_id_search2track = 0;
            }
        }
        else
        {
            m_id_search2track = 0;
        }
    }

    // Searching状态
    bool Selector::SelectorSearching()
    {
        // std::cout << "m_track_id: " << m_track_id << std::endl;
        // 未识别到装甲板
        if(m_armors.size() == 0)
        {
            updateFinalArmor();
            return false;
        }
        // 成功识别到装甲板
        else
        {
            // 按距离准心距离排序
            SortByCenter();

            int IdScoreMIN = m_param->m_armor_priority.size() - 1; 
            int index = -1;
            int sindex = -1;
            for (size_t i = 0; i < m_armors.size(); i++)
            {
                // std::cout << "distance :" << GetDistance(m_armors[i].m_center, cv::Point2f(660, 552)) << std::endl;
                // 如果在准心优先距离内
                if(GetDistance(m_armors[i].m_center, cv::Point2f(660, 552)) < m_param->m_max_searching_center_diff)
                {
                    //如果大于单目距离限制直接认为未识别
                    if(!MonoDisLimit(m_armors[i]))
                        continue;
                    
                    //如果识别结果为灰装甲板
                    if(m_armors[i].m_color == wmj::_COLOR::_WHITE)
                        continue;

                    // id优先级
                    int priority = 0;
                    
                    priority = GetArmorPriority(m_armors[i].m_id);
                    
                    if(IdScoreMIN > priority)
                    {
                        index = i;
                        sindex = -1;
                        IdScoreMIN = priority;
                    }
                    else if (IdScoreMIN == priority)
                    { 
                        if(SecondJudge(m_armors[index],m_armors[i]))
                            sindex = i;
                    }
                }
            }

            // 如果准心优先距离内无装甲板
            if(index == -1)
            {
                m_search_center_lost++;
                // 准心优先距离内一直无装甲板（直接选取最近）
                if (m_search_center_lost >= m_param->m_max_center_searching)
                {
                    for(auto armor: m_armors)
                    {
                        if(armor.m_color != wmj::_COLOR::_WHITE)
                        {
                            m_search_center_lost = 0;
                            m_state = SELECTOR_TRACKING;
                            updateFinalArmor(armor);
                            return true;
                        }
                    }
                }

                // 在操作手置信度范围内
                updateFinalArmor();
                return false;
            }

            else
            {
                m_state = SELECTOR_TRACKING;
                if(sindex != -1)
                    updateFinalArmor(m_armors[index], m_armors[sindex]);
                else 
                    updateFinalArmor(m_armors[index]);
                return true;
            }
        }
        return false;
    }

    // Tracking状态
    bool Selector::SelectorTracking()
    {
        // std::cout << "m_track_id: " << m_track_id << std::endl;
        // 未识别到装甲板
        if(m_armors.size()==0)
        {
            if(m_tracklost >= m_param->m_max_track_lost)
            {
                m_tracklost = 0;
                m_state = SELECTOR_SEARCHING;
                updateFinalArmor();
                return false;
            }
            else
            {
                m_tracklost++;
                updateFinalArmor(m_prebestarmor);
                return false;   
            }
        }
        // 识别到装甲板
        else
        {
            // 按距离前一装甲板距离排序
            SortByCenter();

            // 找最相似的装甲板
            float ScoreMIN = 8;
            int index = -1;
            std::vector<int> sindex;
            m_same_id_switch = false;

            for (size_t i = 0; i < m_armors.size(); i++)
            {
                // 不使用数字识别则id全为-1，一定被使用
                if (m_armors[i].m_id == m_track_id)
                {
                    sindex.push_back(i);
                }

                //如果大于单目距离限制直接认为未识别
                if(!MonoDisLimit(m_armors[i]))
                    continue;
                
                double score = Compare(m_armors[i], m_prebestarmor);
                // std::cout <<_lightpurple("score:  ") << score << std::endl;
                // std::cout << std::endl;

                // 如果为同一装甲板
                if (score != -1 && score < ScoreMIN)
                {
                    m_same_id_switch = false;
                    ScoreMIN = score;
                    index = i;
                }
                // 如果不为同一装甲板但为同一车(未找到相似装甲板前提下)
                else if (index == -1 && m_armors[i].m_id == m_track_id)
                {
                    m_same_id_switch = true;
                    index = i;
                }
            }
            // if(m_same_id_switch)
            //     std::cout << _lightred("Switch") << std::endl;
            
            //未发现相似或者同车装甲板
            if(index == -1)
            {
                if(m_tracklost >= m_param->m_max_track_lost)
                {    
                    m_tracklost = 0;
                    m_state = SELECTOR_SEARCHING;
                    updateFinalArmor();
                    return false;
                }
                //如果判定继续跟踪
                else
                {
                    m_tracklost++;
                    updateFinalArmor(m_prebestarmor);
                    return false;
                }
            }

            //存在相似装甲板
            else if(index != -1)
            {
                // 确认追踪id
                ConfirmTrackId(m_armors[index]);

                float SencondScoreMIN = 8;
                int secondindex = -1;
                for (size_t i = 0; i < sindex.size(); i++)
                {
                    if(sindex[i]!=index &&
                       SecondJudge(m_armors[index],m_armors[sindex[i]]))
                    {
                        // 首次选取非主要目标的同车装甲板中离准心最近装甲板作为次要目标
                        if(m_presecondarmor.m_armor_type == wmj::ARMORTYPE::ARMOR_NONE || m_same_id_switch)
                        {
                            secondindex = i;
                            break;
                        }
                        // 之后对次要目标进行相似判断,认为是同一装甲板才被选为次要目标
                        else
                        {
                            double score = Compare(m_armors[i], m_presecondarmor);
                            // 如果为同一装甲板
                            // std::cout <<_lightpurple("secondscore:  ") << score << std::endl;
                            // std::cout << std::endl;
                            if (score != -1 && score < SencondScoreMIN)
                            {
                                SencondScoreMIN = score;
                                secondindex = i;
                            }
                        }
                    }
                }

                if(secondindex != -1)
                    ChooseBestInSameId(m_armors[index], m_armors[sindex[secondindex]]);
                else
                    updateFinalArmor(m_armors[index]);

                if(m_bestarmor.m_color == wmj::_COLOR::_WHITE)
                {
                    m_tracklost++;
                }
                else
                    m_tracklost = 0;

                return true;
            }   
        }
        return false;
    }

    //常规及反陀螺模式
    bool Selector::SelectorNormal()
    {
        // std::cout << _lightred("State:  ") << m_state << std::endl;
        if(m_state == SELECTOR_SEARCHING)
        {
            // 锁id
            if(m_track_id != -1)
            {
                for (std::vector<wmj::Armor>::iterator it = m_armors.begin(); it != m_armors.end();)
                {
                    if(it->m_id != m_track_id)
                        it = m_armors.erase(it);
                    else
                        it++;
                }
            }
            
            return SelectorSearching();
        }
        else if(m_state == SELECTOR_TRACKING)
        {
            return SelectorTracking();
        }
        return false;
    }

    //哨兵模式
    bool Selector::SelectorSentry()
    {
        // 存在哨兵
        for (size_t i = 0; i < m_armors.size();i++)
        {
            if(m_armors[i].m_id == 7)
            {
                m_tracklost = 0;
                m_state = SELECTOR_TRACKING;
                updateFinalArmor(m_armors[0]);
                return true;
            }
        }

        // 不存在哨兵
        if(m_state == SELECTOR_SEARCHING)
        {
            updateFinalArmor();
            return false;
        }
        else if (m_state == SELECTOR_TRACKING)
        {
            if(m_tracklost >= m_param->m_max_track_lost)
            {
                m_tracklost = 0;
                m_state = SELECTOR_SEARCHING;
                updateFinalArmor();
                return false;
            }
            else
            {
                m_tracklost++;
                updateFinalArmor(m_prebestarmor);
                return false;
            }
        }

        return false;
    }
/*------------------------------------------------------------------主函数----------------------------------------------------------------*/
    //目标筛选主函数
    bool Selector::SelectorMain(std::vector<Armor> armors,wmj::ROBO_STATE mode)
    {
        m_mode = mode;
        m_armors = armors;

        PositionErrorHandling();

        switch (m_mode)
        {
            case wmj::ROBO_STATE::STATE_ARMOR:
                return SelectorNormal();
                break;
            case wmj::ROBO_STATE::STATE_DARK:
                return SelectorSentry();
                break;
            default:
                break;
        }
        return false;
    }

    //目标筛选主函数
    bool Selector::SelectorSingle(std::vector<Armor> armors,wmj::ROBO_STATE mode)
    {
        m_mode = mode;
        m_armors = armors;

        PositionErrorHandling();
        double_use = false;
        switch (m_mode)
        {
            case wmj::ROBO_STATE::STATE_ARMOR:
                return SelectorNormal();
                break;
            case wmj::ROBO_STATE::STATE_DARK:
                return SelectorSentry();
                break;
            default:
                break;
        }
        return false;
    }

}   //wmj 
