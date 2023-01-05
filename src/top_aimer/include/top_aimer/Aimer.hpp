#pragma once

#include "libbase/common.h"
#include "angle_solver/AngleSolver.hpp"
#include "armor/Armor.hpp"
#include "selector/Selector.hpp"
#include "binocular/Binocular.h"

// 将弧度约束在[-pi, pi]范围内
#ifndef _std_radian
#define _std_radian(angle) ((angle) + round((0 - (angle)) / (2 * PI)) * (2 * PI))
#endif

#define CA_MODEL

namespace wmj
{
    /**
     * @brief 运动学状态
     */
    struct KinematicStatus
    {
        KinematicStatus();
#ifdef CA_MODEL
        KinematicStatus(const Eigen::Matrix<double, 13, 1> &X);
#else
        KinematicStatus(const Eigen::Matrix<double, 10, 1> &X);
#endif

        /**
         * @brief 为了保留index，赋值时不继承index
         */
        KinematicStatus operator=(const KinematicStatus &status);

        /**
         * @brief 获取该运动状态下所有装甲板
         * @param predict_time 预测时间
         */
        Armors getArmors(double predict_time = 0);

        /**
         * @brief 获取目标装甲板。方法是取预测时间后最近的一块装甲板
         * @param predict_time 预测时间
         * @param switch_threshold 更新装甲板切换的最小距离差
         */
        Armor getClosestArmor(double predict_time, double switch_threshold);

        /**
         * @brief 打印信息
         */
        void print(std::string tag);

        cv::Point2d center;     // 旋转中心位置
        cv::Point2d velocity;   // 旋转中心速度
        cv::Point2d accelerate; // 加速度
        double height[2];       // 装甲高度。0对应索引0、2，1对应索引1、3
        double radius[2];       // 旋转半径。0对应索引0、2，1对应索引1、3
        double phase;           // 角度（并非定值）
        double palstance;       // 角速度
        double anglar_accelerate; // 角加速度

        int index;              // 当前目标索引
    };

    /**
     * @brief 利用EKF对整车状态进行建模并预测
     * @author 王铭远
     */
    class Aimer
    {
    public:
        Aimer();
        ~Aimer();

        /**
         * @brief 模块主接口。
         * @param armors 相机坐标系装甲板序列
         * @param cur_pose 当前位姿
         * @param bullet_speed 当前射速
         * @return 目标位姿
         */
        GimbalPose getTargetPose(const Armors &armors, const GimbalPose &cur_pose, double bullet_speed);

        /**
         * @brief 数据解算核心过程，同时也是提供给数据模拟器的主接口
         * @param armors 装甲板序列
         * @return 运动学状态
         */
        KinematicStatus resolve(const Armors &armors);

        /**
         * @brief 完全重置
         */
        void reset();

        /**
         * @return 是否允许控制
         */
        bool isReady();

        /**
         * @return 是否允许击发
         */
        bool shootable();

        /**
         * @brief 抛弃装甲板序列中数据异常者
         * @param armors 用于处理的装甲板序列
         */
        void errorHandling(Armors &armors);

    private:
        /**
         * @brief 读取参数配置文件
         * @param file_path 配置文件路径
         */
        void setParam(const std::string &file_path);

        /**
         * @brief 根据熵权法获取装甲板相关性矩阵，相关性指标为负向指标
         * @param armors 识别装甲板序列
         * @param status 当前运动状态
         * @return 相关性矩阵
         */
        Eigen::MatrixXd getScoreMat(const Armors &detect_armors, const Armors &standard_armors);

        /**
         * @brief 装甲板匹配
         * @param score 相关性矩阵。相关性指标为负向指标
         * @return 装甲板关联。键为识别装甲板索引，值为标准装甲板索引
         */
        std::map<int, int> match(const Eigen::MatrixXd &score);

        bool m_debug;
        bool m_ekf_on;                          // 是否使用EKF
        bool m_predict_on;                      // 是否使用预测
        bool m_tracking;                        // 是否跟随
        bool m_enable_shoot;                    // 是否允许自动击发

        int m_tracked_ID;                       // 当前锁定ID
        int m_all_white_cnt;                    // 连续识别到全部为白色装甲板的次数
        double m_next_shoot_time;               // 下次允许射击时间

        // 参数
        double m_time_off;                      // 预测时间补偿
        double m_switch_threshold;              // 更新装甲板切换的最小距离差
        double m_init_pose_tolerance;           // 初始化位姿变化最大值，角度制
        double m_aim_pose_tolerance;            // 自动击发位姿偏差最大值，弧度制
        double m_score_tolerance;               // 装甲板匹配得分最大值
        int m_all_white_tolerance_stop_shoot;   // 连续识别到全部为白色装甲板的次数容忍度，将会停止发射
        int m_all_white_tolerance_reset;        // 连续识别到全部为白色装甲板的次数容忍度，将会重置整个模块
        int m_shoot_interval;                   // 发射间隔，即最多每n帧允许发射一次

        KinematicStatus m_status;               // 当前状态。为了保留最佳目标装甲板，必须声明为类成员变量以保留其索引值
        GimbalPose m_cur_pose;                  // 当前位姿
        GimbalPose m_target_pose;               // 目标位姿

        AngleSolver m_angle_solver;

    private:
        class EKF
        {
        public:
            EKF();
            ~EKF();

            /**
             * @brief 先验预测
             * @param armors 绝对坐标系装甲板序列
             * @return std::pair<Eigen::MatrixXd, Eigen::MatrixXd>(X_, P_)
             */
            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> predict(const Armors &armors);
            
            /**
             * @brief 匹配装甲板序列和标准装甲板
             * @param armors 绝对坐标系装甲板序列
             * @param X_ 先验状态
             * @param P_ 先验状态协方差
             * @param match 装甲板关联
             * @return 运动状态
             */
            KinematicStatus update(const Armors &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match);

            /**
             * @brief 重置
             */
            void reset();

            /**
             * @return 模型是否稳定
             */
            bool stable();

        private:
            /**
             * @brief 读取参数配置文件
             * @param file_path 配置文件路径
             */
            void setParam(const std::string &file_path);

#ifdef CA_MODEL
            /**
             * @brief 获取先验观测量
             * @param X 先验状态
             * @param i 标准装甲板索引
             * @return 先验观测矩阵
             */
            Eigen::Matrix<double, 4, 1> getPredictiveMeasurement(const Eigen::Matrix<double, 13, 1> &X, int i);

            /**
             * @brief 获取观测方程偏导矩阵
             * @param X 先验状态
             * @param i 标准装甲板索引
             * @return 观测偏导矩阵
             */
            Eigen::Matrix<double, 4, 13> getMeasurementPD(const Eigen::Matrix<double, 13, 1> &X, int i);
#else
            /**
             * @brief 获取先验观测量
             * @param X 先验状态
             * @param i 标准装甲板索引
             * @return 先验观测矩阵
             */
            Eigen::Matrix<double, 4, 1> getPredictiveMeasurement(const Eigen::Matrix<double, 10, 1> &X, int i);

            /**
             * @brief 获取观测方程偏导矩阵
             * @param X 先验状态
             * @param i 标准装甲板索引
             * @return 观测偏导矩阵
             */
            Eigen::Matrix<double, 4, 10> getMeasurementPD(const Eigen::Matrix<double, 10, 1> &X, int i);
#endif

            bool m_debug;
            bool m_init;

            // 参数
            double m_dt;                                // 单位时间
            double m_init_radius;                       // 初始半径
#ifdef CA_MODEL
            double m_process_noise[8];                  // 状态转移噪声系数
#else
            double m_process_noise[6];                  // 状态转移噪声系数
#endif
            double m_measure_noise[3];                  // 观测噪声系数

            double m_velocity_update_ratio_tolerance;   // 速度状态修正比例容忍度
            double m_palstance_update_ratio_tolerance;  // 角速度状态修正比例容忍度
#ifdef CA_MODEL
            Eigen::Matrix<double, 13, 1>  m_X;          // 状态
            Eigen::Matrix<double, 13, 1>  m_X_update;   // 状态修正
            Eigen::Matrix<double, 13, 13> m_P;          // 状态协方差
            Eigen::Matrix<double, 13, 13> m_F;          // 状态转移矩阵
            Eigen::Matrix<double, 13, 3>  m_G;          // 控制矩阵
            Eigen::Matrix<double, 13, 13> m_Q;          // 状态转移噪声
#else
            Eigen::Matrix<double, 10, 1>  m_X;          // 状态
            Eigen::Matrix<double, 10, 1>  m_X_update;   // 状态修正
            Eigen::Matrix<double, 10, 10> m_P;          // 状态协方差
            Eigen::Matrix<double, 10, 10> m_F;          // 状态转移矩阵
            Eigen::Matrix<double, 10, 10> m_Q;          // 状态转移噪声
#endif
            Eigen::Matrix<double, 4, 4>   m_R;          // 观测噪声
            Eigen::Matrix<double, 8, 8>   m_RR;         // 观测噪声

        };
        std::shared_ptr<Aimer::EKF> m_EKF;

    private:
        /**
         * @brief 根据相似度矩阵枚举最小代价匹配
         * @author 赵亮程
         */
        class Match
        {
        public:
            Match();
            ~Match();

            /**
             * @brief 获取矩阵中的一组位于互不相同的行和列的数的位置，且选取的这些数的和最小
             * @param matrix 输入的矩阵
             * @return map<int, int> 行和列的位置
             */
            std::map<int, int> getMatch(Eigen::MatrixXd matrix, double score_max);

            bool debug = false;     // 是否输出生成的所有组合数

        private:
            /**
             * @brief 求出在n个数中选取k个数，所有的组合结果，即C(n,k)的所有结果
             * @param input 一组数，由用户决定
             * @param tmp_v 存储中间结果
             * @param result C(n,k)结果
             * @param start 起始位置，应指定为0
             * @param k k个数，由用户决定
             */
            void getCombinationsNumbers(std::vector<int> &input, std::vector<int> &tmp_v, std::vector<std::vector<int>> &result, int start, int k);

            double min = 0, tmp = 0;                    // 最小值

            std::vector<int> col;                       // 数列中的每个数代表矩阵的每一列
            std::vector<int> row;                       // 数列中的每个数代表矩阵的每一行
            std::vector<int> tmp_v;                     // 存储C(n,k)的中间结果

            std::vector<std::vector<int>> result;       // 存储C(n,k)的结果
            std::vector<std::vector<int>> nAfour;       // 存储A(n,4)的结果
            std::vector<std::vector<int>> fourAfour;    // 存储A(4,4)的结果

            std::map<int, int> row_col;                 // 存储最终结果，row_col[i]=j表示矩阵的第i行第j列是要选取的数

        } m_match;

    /************ ROI ************
     * @brief 控制重投影ROI回传
     * @author 王云飞
     */
    public:
        /**
         * @brief 设置深度学习输入图像尺寸
         */
        void setDeepROISize(cv::Size2i deep_roi_size);

        /**
         * @brief 回传左目ROI区域
         */
        cv::Rect2d getLeftROI();
        /**
         * @brief 回传右目ROI区域
         */
        cv::Rect2d getRightROI();

        /**
         * @brief 绘制重投影点
         * @param src 画布
         * @param cur_pose 当前位姿
         */
	    void drawReProjectPoint(cv::Mat &src);

    private:
        /**
         * @brief 传入相机内参矩阵（使用双目时传入左相机参数）
         */
        void setCameraParam();

        /**
         * @brief 内部计算主函数。计算ROI
         */
        void setROI(KinematicStatus status, const Armors &armors);

        /**
         * @brief 获取重投影点
         */
        cv::Point2f getReProjectPoint(const cv::Point3f &point);

        std::shared_ptr<SelectorParam> m_roi_params;    // ROI参数
        DeepROISizeState m_deep_roi_state;              // 深度学习ROI标准
        cv::Rect2d m_deep_default_roi;
        cv::Mat m_camera_mat;

        int m_track_lost_cnt;               // 完全丢识别计数

        int m_min_roi_height;
        float m_roi_height_zoom_rate;
        float m_roi_width_zoom_rate;

        cv::Rect2d m_return_roi_left;       // 左目ROI回传
        cv::Rect2d m_return_roi_right;      // 右目ROI回传
    /************ ROI ************/

    };

}   // wmj
