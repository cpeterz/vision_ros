#include "top_aimer/Aimer.hpp"

namespace wmj
{
#ifdef CA_MODEL
    KinematicStatus::KinematicStatus()
    :   KinematicStatus(Eigen::Matrix<double, 13, 1>::Zero())
    {}
    KinematicStatus::KinematicStatus(const Eigen::Matrix<double, 13, 1> &X)
    :   center(cv::Point2d(X(0, 0), X(1, 0))),
        velocity(cv::Point2d(X(2, 0), X(3, 0))),
        accelerate(cv::Point2d(X(4, 0), X(5, 0))),
        phase(X(10, 0)),
        palstance(X(11, 0)),
        anglar_accelerate(X(12, 0)),
        index(0)
    {
        height[0] = X(6, 0);
        height[1] = X(7, 0);
        radius[0] = X(8, 0);
        radius[1] = X(9, 0);
    }
#else
    KinematicStatus::KinematicStatus()
    :   KinematicStatus(Eigen::Matrix<double, 10, 1>::Zero())
    {}
    KinematicStatus::KinematicStatus(const Eigen::Matrix<double, 10, 1> &X)
    :   center(cv::Point2d(X(0, 0), X(1, 0))),
        velocity(cv::Point2d(X(2, 0), X(3, 0))),
        phase(X(8, 0)),
        palstance(X(9, 0)),
        index(0)
    {
        height[0] = X(4, 0);
        height[1] = X(5, 0);
        radius[0] = X(6, 0);
        radius[1] = X(7, 0);
    }
#endif

    KinematicStatus KinematicStatus::operator=(const KinematicStatus &status)
    {
        this->center    = status.center;
        this->velocity  = status.velocity;
        this->accelerate = status.accelerate;
        this->height[0] = status.height[0];
        this->height[1] = status.height[1];
        this->radius[0] = status.radius[0];
        this->radius[1] = status.radius[1];
        this->phase     = status.phase;
        this->palstance = status.palstance;
        this->anglar_accelerate = status.anglar_accelerate;
        return *this;
    }

    Armors KinematicStatus::getArmors(double predict_time)
    {
        Armors armors(4);
        for (int i = 0; i < 4; i++)
        {
            double angle = _std_radian(phase + palstance * predict_time + i * PI / 2);
            cv::Point2d point = center + velocity * predict_time +
                                cv::Point2d(radius[i % 2] * cos(angle), radius[i % 2] * sin(angle));
            // double angle = _std_radian(phase + palstance * predict_time + 0.5 * anglar_accelerate * predict_time * predict_time + i * PI / 2);
            // cv::Point2d point = center + velocity * predict_time + 0.5 * accelerate * predict_time * predict_time +
            //                     cv::Point2d(radius[i % 2] * cos(angle), radius[i % 2] * sin(angle));
            armors[i].m_position = cv::Point3d(point.x, point.y, height[i % 2]);
            armors[i].m_yaw_angle = angle;
        }
        return armors;
    }

    Armor KinematicStatus::getClosestArmor(double predict_time, double switch_threshold)
    {
        /**
         * 严格来说，理应选择给定延迟时间后最近的一块装甲板
         * 但是，由于测距抖动可能会在两块距离相近的装甲板之间来回跳变，所以在取最小值时加了一段小阈值，需要比最小值减去该阈值更小才认为是新的最小值
         * 再但是，由于目标快速移动时该小阈值可能造成目标切换的延迟，所以给求最小值时的时间再加一段提前量，即预测其在该小段时间后可能会切换目标
         * 举个例子，如果认为在predict_time的switch_advanced_time之后，比上一个目标还要近switch_threshold的目标出现了，则直接切换在predict_time时的目标
         * 这个提前量的计算方法如下：如果接近要切换的位置了，那么应该装甲板是近似斜45度的位置，此时装甲板距离我本身的距离的变化率为 角速度·半径/√2，再除切换阈值即可得到该微小提前量
         * 另外，为了防止角速度接近0时计算得一个过大的提前量，将上述结果与0.1取较小值
         */
        double switch_advanced_time = std::min(0.1, switch_threshold * sqrt(2.0) / (abs(palstance) * radius[index % 2]));
        Armors armors = getArmors(predict_time + switch_advanced_time);
        for (int i = 0; i < 4; i++)
        {
            if (getDistance(armors[i].m_position) + switch_threshold < getDistance(armors[index].m_position))
            {
                index = i;
            }
        }
        return getArmors(predict_time)[index];
    }

    void KinematicStatus::print(std::string tag)
    {
        std::string prefix = "[KinematicStatus] " + tag + " ";
        std::cout << prefix << "center: " << center << " += " << velocity << std::endl;
        std::cout << prefix << "height: " << height[0] << " " << height[1] << std::endl;
        std::cout << prefix << "radius: " << radius[0] << " " << radius[1] << std::endl;
        std::cout << prefix << "angle: " << phase << " += " << palstance << std::endl;
        std::cout << prefix << "index: " << index << std::endl;
    }


    Aimer::Aimer()
    {
        setParam(TOP_CFG);
        setCameraParam();

        m_roi_params = std::make_shared<SelectorParam>();

        reset();

        m_EKF = std::make_shared<Aimer::EKF>();

        // m_match.debug = m_debug;
    }
    Aimer::~Aimer()
    {}

    void Aimer::setParam(const std::string &file_path)
    {
        cv::FileStorage fs(file_path, cv::FileStorage::READ);
        fs["Aimer"]["debug"]         >> m_debug;
        fs["Aimer"]["ekf_on"]        >> m_ekf_on;
        fs["Aimer"]["predict_on"]    >> m_predict_on;

        fs["Aimer"]["time_off"]                          >> m_time_off;
        fs["Aimer"]["switch_threshold"]                  >> m_switch_threshold;
        fs["Aimer"]["init_pose_tolerance"]               >> m_init_pose_tolerance;
        fs["Aimer"]["aim_pose_tolerance"]                >> m_aim_pose_tolerance;
        fs["Aimer"]["score_tolerance"]                   >> m_score_tolerance;
        fs["Aimer"]["all_white_tolerance_stop_shoot"]    >> m_all_white_tolerance_stop_shoot;
        fs["Aimer"]["all_white_tolerance_reset"]         >> m_all_white_tolerance_reset;
        fs["Aimer"]["shoot_interval"]                    >> m_shoot_interval;

        fs["Aimer"]["ROI"]["min_height"]                 >> m_min_roi_height;
        fs["Aimer"]["ROI"]["roi_height_zoom_rate"]       >> m_roi_height_zoom_rate;
        fs["Aimer"]["ROI"]["roi_width_zoom_rate"]        >> m_roi_width_zoom_rate;


        fs.release();
    }

    void Aimer::reset()
    {
        m_tracking = false;
        m_enable_shoot = false;
        m_tracked_ID = -1;
        m_all_white_cnt = 0;
        m_next_shoot_time = now();

        m_deep_roi_state = ROI_BIG;
#ifdef USE_DEEP
        m_return_roi_left = m_roi_params->m_deep_default_roi;
        m_return_roi_right = m_roi_params->m_deep_default_roi;
#else
        m_return_roi_right = m_roi_params->m_camera_resolution;
        m_return_roi_left = m_roi_params->m_camera_resolution;
#endif
        m_track_lost_cnt = 0;

        m_status = KinematicStatus();

        if (m_EKF)
        {
            m_EKF->reset();
        }

        if (m_debug)
            std::cout << _warning("[Aimer] Reset") << std::endl;
    }

    GimbalPose Aimer::getTargetPose(const Armors &armors, const GimbalPose &cur_pose, double bullet_speed)
    {
        m_cur_pose = cur_pose;

        // 初始化锁ID
        if (m_tracked_ID == -1)
        {
            // 建立一个初始化序列，根据瞄准所需位姿和当前位姿的差值对所有识别到的装甲板正序排序
            Armors init_armors = armors;
            sort(init_armors.begin(), init_armors.end(),
                [&](Armor &a, Armor &b)
                {
                    return (m_angle_solver.getAngle(a.m_position, m_cur_pose, bullet_speed) - m_cur_pose).norm() < 
                           (m_angle_solver.getAngle(b.m_position, m_cur_pose, bullet_speed) - m_cur_pose).norm();
                }
            );
            // 在最接近当前瞄准的装甲板排序中，选择最优的非白色装甲板ID作为目标，且该目标与当前瞄准位置偏差在容忍范围内
            // 即防止直接瞄准死车，或因为近处目标丢识别而甩飞；但是当与死车同ID的活车出现时，还是可以锁上该ID
            for (auto armor : init_armors)
            {
                if (armor.m_color != _COLOR::_WHITE)
                {
                    if ((m_angle_solver.getAngle(armor.m_position, m_cur_pose, bullet_speed) - m_cur_pose).norm() <
                        D2R(m_init_pose_tolerance))
                    {
                        m_tracked_ID = armor.m_id;
                        break;
                    }
                }
            }
        }
        // 如果ID未初始化则直接返回当前位姿
        if (m_tracked_ID == -1)
        {
            m_tracking = false;
            m_enable_shoot = false;
            return m_cur_pose;
        }
        if (m_debug)
            std::cout << "[Aimer] id: " << m_tracked_ID << std::endl;

        // 建立目标装甲板序列
        Armors target_armor_seq;
        bool all_white = true;
        for (auto armor : armors)
        {
            // 将与目标ID相同的装甲板填入序列中。但是目前还是相机系。
            if (armor.m_id == m_tracked_ID)
            {
                target_armor_seq.emplace_back(armor);
                if (armor.m_color != _COLOR::_WHITE)
                {
                    all_white = false;
                }
            }
        }
        // 判断当前序列是否全部为白色装甲板
        if (all_white)
        {
            m_all_white_cnt++;
        }
        else
        {
            m_all_white_cnt = 0;
        }

        // 若连续多帧没有识别到目标ID下亮着的装甲板，则认为目标丢失，自动重置
        if (m_all_white_cnt > m_all_white_tolerance_reset)
        {
            reset();
            return m_cur_pose;
        }

        m_tracking = true;

        // 异常数据检查
        errorHandling(target_armor_seq);

        // 根据瞄准所需位姿和当前位姿的差值对目标ID的装甲板正序排序
        sort(target_armor_seq.begin(), target_armor_seq.end(),
            [&](Armor &a, Armor &b)
            {
                return (m_angle_solver.getAngle(a.m_position, m_cur_pose, bullet_speed) - m_cur_pose).norm() < 
                       (m_angle_solver.getAngle(b.m_position, m_cur_pose, bullet_speed) - m_cur_pose).norm();
            }
        );

        // 将目标装甲板序列的坐标和角度转为绝对系
        for (auto &armor : target_armor_seq)
        {
            armor.m_position = m_angle_solver.cam2abs(armor.m_position, m_cur_pose);
            armor.m_yaw_angle = _std_radian(armor.m_yaw_angle + m_cur_pose.yaw + PI);
        }

        // 目标装甲板
        Armor abs_target_armor;
        int last_index = m_status.index;
        if (m_ekf_on)
        {
            // 使用EKF进行建模
            m_status = resolve(target_armor_seq);

            double hit_time = 0;
            if (m_predict_on)
            {
                /**
                 * 严格意义上来说，如果要准确预测击中时刻的装甲板位置的话，需要解一个非线性方程。此处采用一种近似的解法
                 * 根据当前最近装甲板距离计算击中时间，用于预测目标装甲板出现的位置
                 * 事实上相当于一步牛顿迭代法，或者说一阶的线性化
                 */
                hit_time = getDistance(m_status.getClosestArmor(0, 0).m_position) / bullet_speed + m_time_off;
            }
            abs_target_armor = m_status.getClosestArmor(hit_time, m_switch_threshold);
        }
        else
        {
            // 否则直接返回瞄准最佳的装甲板
            abs_target_armor = target_armor_seq[0];
        }

        // 计算ROI
        setROI(m_status, target_armor_seq);

        /**
         * 判断自动击发，条件如下：
         * 1. 当前目标装甲板与上一帧没有切换
         * 2. EKF先验稳定
         * 3. 没有长时间丢识别
         * 4. 当前云台跟随稳定，即当前位姿和目标位姿相近
         */
        if (last_index == m_status.index &&
            m_EKF->stable() &&
            m_all_white_cnt < m_all_white_tolerance_stop_shoot &&
            (m_target_pose - m_cur_pose).norm() < m_aim_pose_tolerance
            )
        {
            m_enable_shoot = true;
        }

        // 计算目标位姿
        m_target_pose = m_angle_solver.getAngle(m_angle_solver.abs2cam(abs_target_armor.m_position, m_cur_pose), m_cur_pose, bullet_speed);

        return m_target_pose;
    }

    KinematicStatus Aimer::resolve(const Armors &armors)
    {
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> prior = m_EKF->predict(armors);
        KinematicStatus prior_status(prior.first);
        if (m_debug)
            prior_status.print("prior");

        Eigen::MatrixXd score = getScoreMat(armors, prior_status.getArmors(0));
        if (m_debug)
            std::cout << "score: " << std::endl << score << std::endl;

        // std::map<int, int> armor_match = match(score);
        std::map<int, int> armor_match = m_match.getMatch(score, m_score_tolerance);

        KinematicStatus posterior_status = m_EKF->update(armors, prior.first, prior.second, armor_match);
        if (m_debug)
            posterior_status.print("posterior");

        return posterior_status;
    }

    bool Aimer::isReady()
    {
        return m_tracking;
    }

    bool Aimer::shootable()
    {
        if (m_enable_shoot && now() > m_next_shoot_time)
        {
            m_next_shoot_time = now() + m_shoot_interval;
            return true;
        }
        return false;
    }

    void Aimer::errorHandling(Armors &armors)
    {
        for (auto it = armors.begin(); it != armors.end();)
        {
            if (isinf(it->m_position.x) || isnan(it->m_position.x) ||
                isinf(it->m_position.y) || isnan(it->m_position.y) ||
                isinf(it->m_position.z) || isnan(it->m_position.z) ||
                isinf(it->m_yaw_angle)  || isnan(it->m_yaw_angle)
                )
            {
                std::cout << _lightred("[Aimer] Error data: inf or nan") << std::endl;
                it = armors.erase(it);
            }
            else
            {
                ++it;
            }
        }
        for (auto armor : armors)
        {
            std::cout << "armor: " << armor.m_position << " " << armor.m_yaw_angle << std::endl;
        }
    }

    Eigen::MatrixXd Aimer::getScoreMat(const Armors &detect_armors, const Armors &standard_armors)
    {
        int m = detect_armors.size();
        int n = standard_armors.size();
        // 计算两组装甲板之间的坐标差和角度差两个负向指标
        Eigen::Matrix<double, Eigen::Dynamic, 2> negative_score;
        negative_score.resize(m * n, 2);
        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
            {
                negative_score(i * n + j, 0) = getDistance(detect_armors[i].m_position - standard_armors[j].m_position);
                negative_score(i * n + j, 1) = abs(_std_radian(detect_armors[i].m_yaw_angle - standard_armors[j].m_yaw_angle));
            }
        }

        // 数据标准化
        Eigen::Matrix<double, Eigen::Dynamic, 2> regular_score;
        regular_score.resize(m * n, 2);
        for (int i = 0; i < regular_score.rows(); i++)
        {
            regular_score(i, 0) = (negative_score.col(0).maxCoeff() - negative_score(i, 0)) / (negative_score.col(0).maxCoeff() - negative_score.col(0).minCoeff());
            regular_score(i, 1) = (negative_score.col(1).maxCoeff() - negative_score(i, 1)) / (negative_score.col(1).maxCoeff() - negative_score.col(1).minCoeff());
        }

        // 计算样本值占指标的比重
        Eigen::Matrix<double, Eigen::Dynamic, 2> score_weight;
        score_weight.resize(m * n, 2);
        Eigen::VectorXd col_sum = regular_score.colwise().sum();
        for (int i = 0; i < score_weight.rows(); i++)
        {
            score_weight(i, 0) = regular_score(i, 0) / col_sum(0);
            score_weight(i, 1) = regular_score(i, 1) / col_sum(1);
        }

        // 计算每项指标的熵值
        Eigen::Vector2d entropy = Eigen::Vector2d::Zero();
        for (int i = 0; i < score_weight.rows(); i++)
        {
            if (score_weight(i, 0) != 0)
                entropy(0) -= score_weight(i, 0) * log(score_weight(i, 0));
            if (score_weight(i, 1) != 0)
                entropy(1) -= score_weight(i, 1) * log(score_weight(i, 1));
        }
        entropy /= log(score_weight.rows());

        // 计算权重
        Eigen::Vector2d weight = (Eigen::Vector2d::Ones() - entropy) / (2 - entropy.sum());

        // 计算匹配得分矩阵
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> score;
        score.resize(m, n);
        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
            {
                if (i < detect_armors.size() && j < standard_armors.size())
                {
                    score(i, j) = negative_score.row(i * standard_armors.size() + j) * weight;
                }
            }
        }

        return score;
    }

    std::map<int, int> Aimer::match(const Eigen::MatrixXd &score)
    {
        std::map<int, int> match;
        for (int i = 0; i < score.rows(); i++)
        {
            int index;
            score.row(i).minCoeff(&index);
            match[i] = index;
            // std::cout << "(" << i << ", " << match[i] << ") ";
        }
        // std::cout << std::endl;
        return match;
    }


    Aimer::EKF::EKF()
    {
        setParam(TOP_CFG);

        reset();

#ifdef CA_MODEL
        m_F << 1, 0, m_dt, 0,    0.5 * m_dt * m_dt, 0,     0, 0, 0, 0, 0, 0, 0,
               0, 1, 0,    m_dt, 0,     0.5 * m_dt * m_dt, 0, 0, 0, 0, 0, 0, 0,
               0, 0, 1,    0,    m_dt, 0,     0, 0, 0, 0, 0, 0, 0,
               0, 0, 0,    1,    0,     m_dt, 0, 0, 0, 0, 0, 0, 0,
               0, 0, 0,    0,    1,    0,     0, 0, 0, 0, 0, 0, 0,
               0, 0, 0,    0,    0,     1,    0, 0, 0, 0, 0, 0, 0,
               0, 0, 0,    0,    0,     0,     1, 0, 0, 0, 0, 0, 0,
               0, 0, 0,    0,    0,     0,     0, 1, 0, 0, 0, 0, 0,
               0, 0, 0,    0,    0,     0,     0, 0, 1, 0, 0, 0, 0,
               0, 0, 0,    0,    0,     0,     0, 0, 0, 1, 0, 0, 0,
               0, 0, 0,    0,    0,     0,     0, 0, 0, 0, 1, m_dt, 0.5 * m_dt * m_dt,
               0, 0, 0,    0,    0,     0,     0, 0, 0, 0, 0, 1, m_dt,
               0, 0, 0,    0,    0,     0,     0, 0, 0, 0, 0, 0, 1;
        std::cout << "m_F: " << m_F << std::endl;

        Eigen::VectorXd process_noise(13);
        process_noise << m_process_noise[0],
                         m_process_noise[0],
                         m_process_noise[1],
                         m_process_noise[1],
                         m_process_noise[2],
                         m_process_noise[2],
                         m_process_noise[3],
                         m_process_noise[3],
                         m_process_noise[4],
                         m_process_noise[4],
                         m_process_noise[5],
                         m_process_noise[6],
                         m_process_noise[7];
        m_Q = process_noise.asDiagonal();
        std::cout << "m_Q: " << m_Q << std::endl;

#else
        m_F << 1, 0, m_dt, 0,    0, 0, 0, 0, 0, 0,
               0, 1, 0,    m_dt, 0, 0, 0, 0, 0, 0,
               0, 0, 1,    0,    0, 0, 0, 0, 0, 0,
               0, 0, 0,    1,    0, 0, 0, 0, 0, 0,
               0, 0, 0,    0,    1, 0, 0, 0, 0, 0,
               0, 0, 0,    0,    0, 1, 0, 0, 0, 0,
               0, 0, 0,    0,    0, 0, 1, 0, 0, 0,
               0, 0, 0,    0,    0, 0, 0, 1, 0, 0,
               0, 0, 0,    0,    0, 0, 0, 0, 1, m_dt,
               0, 0, 0,    0,    0, 0, 0, 0, 0, 1;

        Eigen::VectorXd process_noise(10);
        process_noise << m_process_noise[0],
                         m_process_noise[0],
                         m_process_noise[1],
                         m_process_noise[1],
                         m_process_noise[2],
                         m_process_noise[2],
                         m_process_noise[3],
                         m_process_noise[3],
                         m_process_noise[4],
                         m_process_noise[5];
        m_Q = process_noise.asDiagonal();
#endif

        Eigen::VectorXd measurement_noise4(4);
        measurement_noise4 << m_measure_noise[0],
                              m_measure_noise[0],
                              m_measure_noise[1],
                              m_measure_noise[2];
        m_R = measurement_noise4.asDiagonal();
        Eigen::VectorXd measurement_noise8(8);
        measurement_noise8 << measurement_noise4,
                              measurement_noise4;
        m_RR = measurement_noise8.asDiagonal();
    }
    Aimer::EKF::~EKF()
    {}

    void Aimer::EKF::setParam(const std::string &file_path)
    {
        cv::FileStorage fs(file_path, cv::FileStorage::READ);
        fs["Aimer"]["EKF"]["debug"]              >> m_debug;
        fs["Aimer"]["EKF"]["dt"]                 >> m_dt;
        fs["Aimer"]["EKF"]["init_radius"]        >> m_init_radius;

#ifdef CA_MODEL
        fs["Aimer"]["EKF"]["process_noise"]["center"]            >> m_process_noise[0];
        fs["Aimer"]["EKF"]["process_noise"]["velocity"]          >> m_process_noise[1];
        fs["Aimer"]["EKF"]["process_noise"]["accelerate"]        >> m_process_noise[2];
        fs["Aimer"]["EKF"]["process_noise"]["height"]            >> m_process_noise[3];
        fs["Aimer"]["EKF"]["process_noise"]["radius"]            >> m_process_noise[4];
        fs["Aimer"]["EKF"]["process_noise"]["phase"]             >> m_process_noise[5];
        fs["Aimer"]["EKF"]["process_noise"]["palstance"]         >> m_process_noise[6];
        fs["Aimer"]["EKF"]["process_noise"]["anglar_accelerate"] >> m_process_noise[7];
#else
        fs["Aimer"]["EKF"]["process_noise"]["center"]    >> m_process_noise[0];
        fs["Aimer"]["EKF"]["process_noise"]["velocity"]  >> m_process_noise[1];
        fs["Aimer"]["EKF"]["process_noise"]["height"]    >> m_process_noise[2];
        fs["Aimer"]["EKF"]["process_noise"]["radius"]    >> m_process_noise[3];
        fs["Aimer"]["EKF"]["process_noise"]["phase"]     >> m_process_noise[4];
        fs["Aimer"]["EKF"]["process_noise"]["palstance"] >> m_process_noise[5];
#endif

        fs["Aimer"]["EKF"]["measure_noise"]["point"]     >> m_measure_noise[0];
        fs["Aimer"]["EKF"]["measure_noise"]["height"]    >> m_measure_noise[1];
        fs["Aimer"]["EKF"]["measure_noise"]["angle"]     >> m_measure_noise[2];

        fs["Aimer"]["EKF"]["velocity_update_ratio_tolerance"]    >> m_velocity_update_ratio_tolerance;
        fs["Aimer"]["EKF"]["palstance_update_ratio_tolerance"]   >> m_palstance_update_ratio_tolerance;

        fs.release();
    }

    void Aimer::EKF::reset()
    {
        m_init = false;

#ifdef CA_MODEL
        m_P = Eigen::Matrix<double, 13, 13>::Identity() * 1e-5;
#else
        m_P = Eigen::Matrix<double, 10, 10>::Identity() * 1e-5;
#endif
    }

    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> Aimer::EKF::predict(const Armors &armors)
    {
#ifdef CA_MODEL
        if (!m_init)
        {
            for (auto armor : armors)
            {
                // 取排序后第一块非白色装甲板计算初始化状态
                // 如果进入该初始化部分，前置逻辑已保证序列中一定有一个非白色装甲板
                if (armor.m_color != _COLOR::_WHITE)
                {
                    m_X << armor.m_position.x - m_init_radius * cos(armor.m_yaw_angle),
                           armor.m_position.y - m_init_radius * sin(armor.m_yaw_angle),
                           0,
                           0,
                           0,
                           0,
                           armor.m_position.z,
                           armor.m_position.z,
                           m_init_radius,
                           m_init_radius,
                           armor.m_yaw_angle,
                           0,
                           0;
                    m_init = true;
                    break;
                }
            }
        }

        // 预测
        // Eigen::MatrixXd X_ = m_F * m_X + m_G * Eigen::Vector3d(m_X(4, 0), m_X(5, 0), m_X(12, 0));
        Eigen::MatrixXd X_ = m_F * m_X;
        Eigen::MatrixXd P_ = m_F * m_P * m_F.transpose() + m_Q;
        if (m_debug)
        {
            std::cout << "[EKF] X_: " << std::endl << X_ << std::endl;
            std::cout << "[EKF] P_: " << std::endl << P_ << std::endl;
        }

        return std::make_pair(X_, P_);
#else
        if (!m_init)
        {
            for (auto armor : armors)
            {
                // 取排序后第一块非白色装甲板计算初始化状态
                // 如果进入该初始化部分，前置逻辑已保证序列中一定有一个非白色装甲板
                if (armor.m_color != _COLOR::_WHITE)
                {
                    m_X << armor.m_position.x - m_init_radius * cos(armor.m_yaw_angle),
                           armor.m_position.y - m_init_radius * sin(armor.m_yaw_angle),
                           0,
                           0,
                           armor.m_position.z,
                           armor.m_position.z,
                           m_init_radius,
                           m_init_radius,
                           armor.m_yaw_angle,
                           0;
                    m_init = true;
                    break;
                }
            }
        }

        // 预测
        Eigen::MatrixXd X_ = m_F * m_X;
        Eigen::MatrixXd P_ = m_F * m_P * m_F.transpose() + m_Q;
        if (m_debug)
        {
            std::cout << "[EKF] X_: " << std::endl << X_ << std::endl;
            std::cout << "[EKF] P_: " << std::endl << P_ << std::endl;
        }

        return std::make_pair(X_, P_);
#endif
    }

    KinematicStatus Aimer::EKF::update(const Armors &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match)
    {
        // 观测
        Eigen::MatrixXd Z;      // 实际观测量
        Eigen::MatrixXd h;      // 先验观测量
        Eigen::MatrixXd H;      // 观测方程偏导矩阵
        Eigen::MatrixXd K;      // 置信度权重矩阵

#ifdef CA_MODEL
        if (match.size() == 1)
        {
            int num = match.begin()->first;
            Z.resize(4, 1);
            Z << armors[num].m_position.x, armors[num].m_position.y, armors[num].m_position.z, armors[num].m_yaw_angle;
            h.resize(4, 1);
            h << getPredictiveMeasurement(X_, match[num]);
            H.resize(4, 13);
            H << getMeasurementPD(X_, match[num]);

            K = P_ * H.transpose() * ((H * P_ * H.transpose() + m_R)).inverse();
        }
        else if (match.size() == 2)
        {
            int num1 = match.begin()->first;
            int num2 = (++match.begin())->first;
            Z.resize(8, 1);
            Z << armors[num1].m_position.x, armors[num1].m_position.y, armors[num1].m_position.z, armors[num1].m_yaw_angle,
                 armors[num2].m_position.x, armors[num2].m_position.y, armors[num2].m_position.z, armors[num2].m_yaw_angle;
            h.resize(8, 1);
            h << getPredictiveMeasurement(X_, match[num1]),
                 getPredictiveMeasurement(X_, match[num2]);
            H.resize(8, 13);
            H << getMeasurementPD(X_, match[num1]),
                 getMeasurementPD(X_, match[num2]);

            K = P_ * H.transpose() * ((H * P_ * H.transpose() + m_RR)).inverse();
        }
        else
        {
            // 若匹配数目不为1或2，则认为丢识别或者误判，直接返回先验状态
            return KinematicStatus(X_);
        }
#else
        if (match.size() == 1)
        {
            int num = match.begin()->first;
            Z.resize(4, 1);
            Z << armors[num].m_position.x, armors[num].m_position.y, armors[num].m_position.z, armors[num].m_yaw_angle;
            h.resize(4, 1);
            h << getPredictiveMeasurement(X_, match[num]);
            H.resize(4, 10);
            H << getMeasurementPD(X_, match[num]);

            K = P_ * H.transpose() * ((H * P_ * H.transpose() + m_R)).inverse();
        }
        else if (match.size() == 2)
        {
            int num1 = match.begin()->first;
            int num2 = (++match.begin())->first;
            Z.resize(8, 1);
            Z << armors[num1].m_position.x, armors[num1].m_position.y, armors[num1].m_position.z, armors[num1].m_yaw_angle,
                 armors[num2].m_position.x, armors[num2].m_position.y, armors[num2].m_position.z, armors[num2].m_yaw_angle;
            h.resize(8, 1);
            h << getPredictiveMeasurement(X_, match[num1]),
                 getPredictiveMeasurement(X_, match[num2]);
            H.resize(8, 10);
            H << getMeasurementPD(X_, match[num1]),
                 getMeasurementPD(X_, match[num2]);

            K = P_ * H.transpose() * ((H * P_ * H.transpose() + m_RR)).inverse();
        }
        else
        {
            // 若匹配数目不为1或2，则认为丢识别或者误判，直接返回先验状态
            m_X = X_;
            m_P = P_;
            return KinematicStatus(m_X);
        }
#endif
        // if (m_debug)
        // {
        //     std::cout << "[EKF] Z: " << std::endl << Z << std::endl;
        //     std::cout << "[EKF] h: " << std::endl << h << std::endl;
        //     std::cout << "[EKF] H: " << std::endl << H << std::endl;
        // }

        // 更新
        Eigen::MatrixXd tmp = Z - h;
        for (int i = 0; i < match.size(); i++)
        {
            tmp(3 + i * 4, 0) = _std_radian(tmp(3 + i * 4, 0));
        }
        m_X_update = K * tmp;
        if (m_debug)
        {
            std::cout << "[EKF] X_update: " << std::endl << m_X_update << std::endl;
        }

        m_X = X_ + m_X_update;
#ifdef CA_MODEL
        m_P = (Eigen::Matrix<double, 13, 13>::Identity() - K * H) * P_;
#else
        m_P = (Eigen::Matrix<double, 10, 10>::Identity() - K * H) * P_;
#endif
        if (m_debug)
        {
            std::cout << "[EKF] X:" << std::endl << m_X << std::endl;
            std::cout << "[EKF] P:" << std::endl << m_P << std::endl;
        }

        return KinematicStatus(m_X);
    }

    bool Aimer::EKF::stable()
    {
        return true;
        // double velocity_update_ratio = sqrt(pow(m_X_update(2, 0), 2) + pow(m_X_update(3, 0), 2)) / sqrt(pow(m_X(2, 0), 2) + pow(m_X(3, 0), 2));
        // double palstance_update_ratio = m_X_update(9, 0) / m_X(9, 0);
        // return velocity_update_ratio < m_velocity_update_ratio_tolerance && palstance_update_ratio < m_palstance_update_ratio_tolerance;
    }
#ifdef CA_MODEL
    Eigen::Matrix<double, 4, 1> Aimer::EKF::getPredictiveMeasurement(const Eigen::Matrix<double, 13, 1> &X, int i)
    {
        Eigen::Matrix<double, 4, 1> h;
        h << X(0, 0) + X(8 + i % 2, 0) * cos(X(10, 0) + i * PI / 2),
             X(1, 0) + X(8 + i % 2, 0) * sin(X(10, 0) + i * PI / 2),
             X(6 + i % 2, 0),
             X(10, 0) + i * PI / 2;
        return h;
    }

    Eigen::Matrix<double, 4, 13> Aimer::EKF::getMeasurementPD(const Eigen::Matrix<double, 13, 1> &X, int i)
    {
        Eigen::Matrix<double, 4, 13> H;
        H << 1, 0, 0, 0, 0, 0, 0,           0,     ((i + 1) % 2) * cos(X(10, 0) + i * PI / 2), (i % 2) * cos(X(10, 0) + i * PI / 2),-X(8 + i % 2, 0) * sin(X(10, 0) + i * PI / 2), 0, 0,
             0, 1, 0, 0, 0, 0, 0,           0,     ((i + 1) % 2) * sin(X(10, 0) + i * PI / 2), (i % 2) * sin(X(10, 0) + i * PI / 2), X(8 + i % 2, 0) * cos(X(10, 0) + i * PI / 2), 0, 0,
             0, 0, 0, 0, 0, 0, (i + 1) % 2, i % 2, 0,                                         0,                                   0,                                           0, 0,
             0, 0, 0, 0, 0, 0, 0,           0,     0,                                         0,                                   1,                                           0, 0;
        return H;
    }
#else
    Eigen::Matrix<double, 4, 1> Aimer::EKF::getPredictiveMeasurement(const Eigen::Matrix<double, 10, 1> &X, int i)
    {
        Eigen::Matrix<double, 4, 1> h;
        h << X(0, 0) + X(6 + i % 2, 0) * cos(X(8, 0) + i * PI / 2),
             X(1, 0) + X(6 + i % 2, 0) * sin(X(8, 0) + i * PI / 2),
             X(4 + i % 2, 0),
             X(8, 0) + i * PI / 2;
        return h;
    }

    Eigen::Matrix<double, 4, 10> Aimer::EKF::getMeasurementPD(const Eigen::Matrix<double, 10, 1> &X, int i)
    {
        Eigen::Matrix<double, 4, 10> H;
        H << 1, 0, 0, 0, 0,           0,     ((i + 1) % 2) * cos(X(8, 0) + i * PI / 2), (i % 2) * cos(X(8, 0) + i * PI / 2),-X(6 + i % 2, 0) * sin(X(8, 0) + i * PI / 2), 0,
             0, 1, 0, 0, 0,           0,     ((i + 1) % 2) * sin(X(8, 0) + i * PI / 2), (i % 2) * sin(X(8, 0) + i * PI / 2), X(6 + i % 2, 0) * cos(X(8, 0) + i * PI / 2), 0,
             0, 0, 0, 0, (i + 1) % 2, i % 2, 0,                                         0,                                   0,                                           0,
             0, 0, 0, 0, 0,           0,     0,                                         0,                                   1,                                           0;
        return H;
    }
#endif

    Aimer::Match::Match()
    {}
    Aimer::Match::~Match()
    {}

    std::map<int, int> Aimer::Match::getMatch(Eigen::MatrixXd matrix, double score_max)
    {
        /**
         * @brief 初始化各个变量
         *
         */
        /// 初始化最终结果
        this->row_col.clear();
        for (int i = 0; i < matrix.rows(); i++)
        {
            // 值为-1表示不选择该行中的任何数
            this->row_col[i] = -1;
        }
        /// 初始化min
        for (int i = 0; i < matrix.rows() && i < 4; i++)
        {
            this->min += matrix(i, i);
            this->row_col[i] = i;
        }
        /// 初始化行数列
        this->row.clear();
        for (int i = 0; i < matrix.rows(); i++)
        {
            // 有多少行，行数列就有多少个数
            this->row.emplace_back(i);
        }
        /// 初始化列数列，4块装甲板，因此列为4
        this->col.clear();
        this->col = {0, 1, 2, 3};

        this->tmp_v.clear();
        this->result.clear();
        this->nAfour.clear();
        this->fourAfour.clear();

        /**
         * @brief 用A(n,4)求出可能选择的所有行的组合
         * 因为C(n,4) * A(4,4) = A(n,4)，所以先计算C(n,4)，再计算A(4,4)，最后将结果存到result中
         *
         */
        /// 计算C(n,4)
        if (matrix.rows() <= 4)
            this->getCombinationsNumbers(this->row, this->tmp_v, this->result, 0, matrix.rows());
        else
            this->getCombinationsNumbers(this->row, this->tmp_v, this->result, 0, 4);
        // 输出C(n,4)计算结果
        if (this->debug)
        {
            // 红色输出
            std::cout << _red("C(n,4)生成结果") << std::endl;
            for (auto it = this->result.begin(); it != this->result.end(); it++)
            {
                for (std::vector<int>::iterator it2 = it->begin(); it2 != it->end(); it2++)
                {
                    std::cout << *it2 << " ";
                }
                std::cout << std::endl;
            }
            std::cout << "-------------------------------" << std::endl;
        }
        /// 使用上一步的结果计算A(4,4)，最终得到A(n,4)
        for (auto it = this->result.begin(); it != this->result.end(); it++)
        {
            do
            {
                this->nAfour.emplace_back(*it);
            } while (next_permutation(it->begin(), it->end())); // stl自带全排列函数
        }
        // 输出A(n,4)计算结果
        if (this->debug)
        {
            // 绿色输出
            std::cout << _green("A(n,4)生成结果") << std::endl;
            for (auto it = this->nAfour.begin(); it != this->nAfour.end(); it++)
            {
                for (std::vector<int>::iterator it2 = it->begin(); it2 != it->end(); it2++)
                {
                    std::cout << *it2 << " ";
                }
                std::cout << std::endl;
            }
            std::cout << "-------------------------------" << std::endl;
        }

        /**
         * @brief 用A(4,4)求出可能选择的所有列的组合
         *
         */
        /// 计算A(4,4)
        do
        {
            fourAfour.push_back(col);
        } while (next_permutation(col.begin(), col.end())); // stl自带全排列函数
        // 输出A(4,4)计算结果
        if (this->debug)
        {
            // 蓝色输出
            std::cout << _blue("A(4,4)生成结果") << std::endl;
            for (auto it = this->fourAfour.begin(); it != this->fourAfour.end(); it++)
            {
                for (std::vector<int>::iterator it2 = it->begin(); it2 != it->end(); it2++)
                {
                    std::cout << *it2 << " ";
                }
                std::cout << std::endl;
            }
            std::cout << "-------------------------------" << std::endl;
        }

        /**
         * @brief 用A(n,4)和A(4,4)求出可能选择的所有行列的组合的结果
         *
         */
        for (auto it = this->nAfour.begin(); it != this->nAfour.end(); it++)
        {
            for (auto it2 = this->fourAfour.begin(); it2 != this->fourAfour.end(); it2++)
            {
                // 对于每一种组合，都求一遍它们的值，与min比较，如果小于min，则更新min
                this->tmp = 0;
                for (int i = 0; i < it->size(); i++)
                    this->tmp += matrix((*it)[i], (*it2)[i]);
                if (this->tmp < this->min)
                {
                    this->min = this->tmp;
                    // 重新初始化map
                    this->row_col.clear();
                    // 每选择矩阵中的一个数时，就将该位置记录下来存到map中
                    for (int i = 0; i < it->size(); i++)
                    {
                        this->row_col[it->at(i)] = it2->at(i);
                    }
                }
            }
        }
        // 输出删除前的map
        if (this->debug)
        {
            // 紫色输出
            std::cout << _purple("删除-1前的map结果") << std::endl;
            for (auto it = this->row_col.begin(); it != this->row_col.end(); it++)
            {
                std::cout << it->first << " " << it->second << std::endl;
            }
            std::cout << "-------------------------------" << std::endl;
        }
        /// 删除-1的键值对
        for (auto it = this->row_col.begin(); it != this->row_col.end();)
        {
            if (it->second == -1 || (it->second != -1 && matrix(it->first, it->second) > score_max))
            {
                it = this->row_col.erase(it);
            }
            else
                ++it;
        }
        // 输出min的值和删除后的map
        if (this->debug)
        {
            // 白色输出
            std::cout << _white("min和删除-1后的map结果") << std::endl;
            std::cout << "min = " << min << std::endl;
            for (auto it = this->row_col.begin(); it != this->row_col.end(); ++it)
            {
                std::cout << it->first << " " << it->second << " " << matrix(it->first, it->second) << std::endl;
            }
            std::cout << "-------------------------------" << std::endl;
        }
        /// 输出矩阵
        if (this->debug)
        {
            std::cout << "matrix:" << std::endl;
            for (int i = 0; i < matrix.rows(); i++)
            {
                for (int j = 0; j < matrix.cols(); j++)
                {
                    std::cout << matrix(i, j) << " ";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }

        return this->row_col;
    }

    void Aimer::Match::getCombinationsNumbers(std::vector<int> &input, std::vector<int> &tmp_v, std::vector<std::vector<int>> &result, int start, int k)
    {
        for (int i = start; i < input.size(); i++)
        {
            tmp_v.emplace_back(input[i]);
            if (tmp_v.size() == k)
            {
                result.emplace_back(tmp_v);
                tmp_v.pop_back();
                continue;
            }
            // 使用递归计算
            getCombinationsNumbers(input, tmp_v, result, i + 1, k);
            tmp_v.pop_back();
        }
    }

/************ ROI ************/
    void Aimer::setCameraParam()
    {
        Monocular mono;
#ifdef USE_BINO
        m_camera_mat = mono.m_CameraMat_Left;
#else
        m_camera_mat = mono.m_CameraMat;
#endif
        return;
    }

    void Aimer::setROI(KinematicStatus status, const Armors &armors)
    {
        cv::Rect2d roi;
        double m_roi_value = m_roi_params->m_roi_lost_mul[m_track_lost_cnt + 1];

#ifdef USE_DEEP
            // std::cout << "DEEP SELECT" << std::endl;
            if (armors.empty()) 
            {
                if (m_track_lost_cnt > m_roi_params->m_max_track_lost)
                {
                    // roi返回为中央roi
                    m_deep_roi_state = wmj::DeepROISizeState::ROI_BIG;
                    m_return_roi_left = m_roi_params->m_deep_default_roi;
                    m_return_roi_right = m_roi_params->m_deep_default_roi;
                    return;
                }
                else
                {
                    m_track_lost_cnt++;
                }

            }
            else
            {
                m_track_lost_cnt = 0;
            }

            Armors pre_armors = status.getArmors();
            std::vector<cv::Point2f> reprojection_points;
            for (auto &armor : armors)
            {
                reprojection_points.emplace_back(getReProjectPoint(m_angle_solver.abs2cam(armor.m_position, m_cur_pose)));
                for (int i = 0; i < 4; i++)
                {
                    reprojection_points.emplace_back(armor.m_vertices[i]);
                }
            }
            for (auto &armor : pre_armors)
            {
                reprojection_points.emplace_back(getReProjectPoint(m_angle_solver.abs2cam(armor.m_position, m_cur_pose)));
            }
            reprojection_points.emplace_back(getReProjectPoint(m_angle_solver.abs2cam(cv::Point3f(m_status.center.x, m_status.center.y, m_status.height[0]), m_cur_pose)));

            float max_x, min_x, max_y, min_y;
            max_x = min_x = reprojection_points[0].x;
            max_y = min_y = reprojection_points[0].y;
            for (auto &point : reprojection_points)
            {
                max_x = max_x > point.x ? max_x : point.x;
                min_x = min_x < point.x ? min_x : point.x;
                max_y = max_y > point.y ? max_y : point.y;
                min_y = min_y < point.y ? min_y : point.y;
            }
            float height,width;
            height = max_y - min_y;
            width = max_x - min_x;
            roi = cv::Rect2d(min_x - (1.3 - 1) * 0.5 * width, min_y - (1.3 - 1) * 0.5 * height, width * 1.3, height * 1.3);
            roi &= m_roi_params->m_camera_resolution;
            if (roi.height * 1.3 > m_roi_params->m_deep_roi_size.height || roi.width * 1.5 > m_roi_params->m_deep_roi_size.width)
            {
                m_deep_roi_state = wmj::DeepROISizeState::ROI_BIG;
                roi = m_roi_params->m_deep_default_roi; 
                roi.y = min_y + (height - roi.height) / 2.0;
                roi.y = std::max(std::min((float)roi.y, (float)(m_roi_params->m_camera_resolution.height - 1 - m_roi_params->m_deep_default_roi.height)), 1.f);
            }

            else if (height * 1.5 > m_roi_params->m_deep_roi_size.height || width * 1.9 > m_roi_params->m_deep_roi_size.width)
            {
                if (m_deep_roi_state == wmj::DeepROISizeState::ROI_SMALL)
                {
                    roi.y += (roi.height - m_roi_params->m_deep_roi_size.height) / 2.0;
                    roi.height = m_roi_params->m_deep_roi_size.height;
                    roi.x += (roi.width - m_roi_params->m_deep_roi_size.width) / 2.0;
                    roi.width = m_roi_params->m_deep_roi_size.width;
                    roi.x = std::max(std::min((float)roi.x, (float)(m_roi_params->m_camera_resolution.width - 1 - m_roi_params->m_deep_roi_size.width)), 1.f);
                    roi.y = std::max(std::min((float)roi.y, (float)(m_roi_params->m_camera_resolution.height - 1 - m_roi_params->m_deep_roi_size.height)), 1.f);
                }
                else
                {
                    roi = m_roi_params->m_deep_default_roi; 
                    roi.y = min_y + (height - roi.height) / 2.0;
                    roi.y = std::max(std::min((float)roi.y, (float)(m_roi_params->m_camera_resolution.height - 1 - m_roi_params->m_deep_default_roi.height)), 1.f);
                }
            }
            else
            {
                m_deep_roi_state = wmj::DeepROISizeState::ROI_SMALL;
                roi.y += (roi.height - m_roi_params->m_deep_roi_size.height) / 2.0;
                roi.height = m_roi_params->m_deep_roi_size.height;
                roi.x += (roi.width - m_roi_params->m_deep_roi_size.width) / 2.0;
                roi.width = m_roi_params->m_deep_roi_size.width;
                roi.x = std::max(std::min((float)roi.x, (float)(m_roi_params->m_camera_resolution.width - 1 - m_roi_params->m_deep_roi_size.width)), 1.f);
                roi.y = std::max(std::min((float)roi.y, (float)(m_roi_params->m_camera_resolution.height - 1 - m_roi_params->m_deep_roi_size.height)), 1.f);
            }
            if (roi.area() == 0)
            {
                roi = m_roi_params->m_deep_default_roi;
            }
            m_return_roi_right = roi;
            m_return_roi_left = roi;
            return;

#else
            if (armors.empty())
            {
                if (m_track_lost_cnt > m_roi_params->m_max_track_lost)
                {
                    // roi返回为中央roi
                    m_return_roi_left = m_roi_params->m_camera_resolution;
                    m_return_roi_right =  m_roi_params->m_camera_resolution;
                    return;
                }
                else
                {
                    m_track_lost_cnt++;
                }
            }
            else
            {
                m_track_lost_cnt = 0;
            }
            // 如果为哨兵模式则返回一个轨道狭长长矩形的roi区域
            if (m_tracked_ID == 7 && !armors.empty())
            {
                roi = armors[0].m_rect;
                roi.y -= roi.height * m_roi_params->m_sentry_roi_up_ratio * m_roi_value;
                roi.height *= 1.0 + m_roi_params->m_sentry_roi_up_ratio * m_roi_value + m_roi_params->m_sentry_roi_down_ratio * m_roi_value;
                roi.x -= roi.width * m_roi_params->m_sentry_roi_left_ratio * m_roi_value;
                roi.width *= 1.0 + m_roi_params->m_sentry_roi_left_ratio * m_roi_value + m_roi_params->m_sentry_roi_right_ratio * m_roi_value;
                // 防止越界 
                roi &= m_roi_params->m_camera_resolution;
            }
            // 如果为反陀螺和常规模式，则返回一个全车装甲板的矩形roi区域
            else
            {
                Armors pre_armors = status.getArmors();
                std::vector <cv::Point2f> reprojection_points;
                for (auto &armor : armors)
                {
                    reprojection_points.emplace_back(getReProjectPoint(m_angle_solver.abs2cam(armor.m_position, m_cur_pose)));
                }
                for (auto &armor : pre_armors)
                {
                    reprojection_points.emplace_back(getReProjectPoint(m_angle_solver.abs2cam(armor.m_position, m_cur_pose)));
                }
                reprojection_points.emplace_back(getReProjectPoint(m_angle_solver.abs2cam(cv::Point3f(status.center.x, status.center.y, status.height[0]),m_cur_pose)));

                float max_x, min_x, max_y, min_y;
                max_x = min_x = reprojection_points[0].x;
                max_y = min_y = reprojection_points[0].y;
                for(auto &point : reprojection_points)
                {
                    max_x = max_x > point.x ? max_x : point.x;
                    min_x = min_x < point.x ? min_x : point.x;
                    max_y = max_y > point.y ? max_y : point.y;
                    min_y = min_y < point.y ? min_y : point.y;
                }

                float height,width;
                height = max_y - min_y;
		        height = height > m_min_roi_height ? height : m_min_roi_height;
                width = max_x - min_x;
                roi = cv::Rect2d(
                    min_x - (m_roi_width_zoom_rate - 1) * 0.5 * width,
                    min_y - (m_roi_height_zoom_rate - 1) * 0.5 * height,
                    width * m_roi_height_zoom_rate,
                    height * m_roi_height_zoom_rate
                );
                roi &= m_roi_params->m_camera_resolution;
            }

            if (roi.area() == 0)
            {
                roi = m_roi_params->m_camera_resolution;
            }

            // armors为空 双目或左目
            if (armors.empty() || armors[0].m_detectedtype == 0 || armors[0].m_detectedtype == 2)
            {
                m_return_roi_left = roi;
                m_return_roi_right = m_return_roi_left;
                m_return_roi_right.x -= m_roi_params->m_aim_point_dis;
                m_return_roi_right &= m_roi_params->m_camera_resolution;
            }
            // 右目
            else
            {
                m_return_roi_right = roi;
                m_return_roi_left = roi;
                m_return_roi_left.x += m_roi_params->m_aim_point_dis;
                m_return_roi_right &= m_roi_params->m_camera_resolution;
            }
#endif
        return;
    }

    void Aimer::setDeepROISize(cv::Size2i deep_roi_size)
    {
	    m_roi_params->m_deep_roi_size = deep_roi_size;
	    double rate = (double)deep_roi_size.height / (double)deep_roi_size.width;
	    m_roi_params->m_deep_default_roi = cv::Rect2d(
            0,
            (m_roi_params->m_camera_resolution.height - m_roi_params->m_camera_resolution.width * rate) / 2,
            m_roi_params->m_camera_resolution.width,
            (m_roi_params->m_camera_resolution.width * rate)
        );
        m_return_roi_left = m_roi_params->m_deep_default_roi;
        m_return_roi_right = m_roi_params->m_deep_default_roi;
    }

    cv::Rect2d Aimer::getLeftROI()
    {
        return m_return_roi_left;
    }
    cv::Rect2d Aimer::getRightROI()
    {
        return m_return_roi_right;
    }

    void Aimer::drawReProjectPoint(cv::Mat &src)
    {
		Armors pre_armors = m_status.getArmors();
        std::vector<cv::Point2f> reprojection_points;
        for (auto &armor : pre_armors)
        {
            reprojection_points.emplace_back(getReProjectPoint(m_angle_solver.abs2cam(armor.m_position, m_cur_pose)));
        }
		cv::Point2f center = getReProjectPoint(m_angle_solver.abs2cam(cv::Point3f(m_status.center.x, m_status.center.y, m_status.height[0]), m_cur_pose));
		cv::circle(src, center, 8, cv::Scalar(255, 0, 255), -1);
		for (auto point : reprojection_points)
		{
		    cv::circle(src, point, 10, cv::Scalar(255, 255, 0), 3);
		}
        cv::rectangle(src, m_return_roi_right, cv::Scalar(0,0,255), 2);
        return;
    }

    cv::Point2f Aimer::getReProjectPoint(const cv::Point3f &point)
    {
        cv::Mat_<double> point_mat = (cv::Mat_<double>(3, 1) << point.y / -1000, point.z / -1000, point.x / 1000);
        cv::Mat reprojection_point = m_camera_mat * point_mat / (point.x / 1000);
        return cv::Point2f(reprojection_point.at<double>(0, 0), reprojection_point.at<double>(1, 0));
    }
/************ ROI ************/

}   // wmj
