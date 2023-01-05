#pragma once
#include "libbase/common.h"
#include <Eigen/Dense>
#include <fstream>
#include <sys/time.h>
#include <algorithm>
#include <vector>

namespace wmj
{
    // 灯条预处理类 把灯条矩形内的所有点的图像系坐标和灯条系坐标存储下来
    class LightPreProcess
    {
        public:
            int m_debug = 0; // 改成1会输出很多 建议把要输出的值在cpp里面直接注释debug

        protected:
            cv::Mat m_gray;          // 灯条roi的灰度图
            double m_roi_x, m_roi_y; // 当前roi的位置

            cv::RotatedRect m_left_rect, m_right_rect;
            cv::Point2f m_left_center, m_right_center;
            Eigen::Matrix<float, 2, 2> m_left_rot_mat, m_right_rot_mat;
            Eigen::Matrix<float, 2, 2> m_left_rot_mat_inv, m_right_rot_mat_inv;

            std::vector<cv::Point2f> m_left_points, m_right_points;   // 图像系坐标 用来读通道值时要转换成roi内的坐标
            std::vector<cv::Point2f> m_left_points_, m_right_points_; // 灯条系坐标
            double m_left_channel_sum, m_right_channel_sum;

            double m_x_extend_factor = 1.1; // 扩展采样旋转矩形区域x方向
            double m_y_extend_factor = 1.1; // 扩展采样旋转矩形区域y方向
            double m_pick_step = 1.0;       // 采样点的间距

            void getLeftRectPoints();    // 获取目标旋转矩形内的所有像素点坐标对 m_left_points 和 m_left_points_
            void getRightRectPoints();   // 获取目标旋转矩形内的所有像素点坐标对 m_right_points 和 m_right_points_
    };

    // 灯条优化类
    class LightOptimizer:LightPreProcess
    {
        public:
            int m_debug = 0; // 改成1会输出很多 建议把要输出的值在cpp里面直接注释debug
            std::vector<cv::Point2f> m_final_vertices;

            void LightOptimization(cv::RotatedRect left_rect, cv::RotatedRect right_rect, cv::Rect2d &roi, cv::Mat &gray);
        private:
            float m_light_moved_dis_thres = 3.f; // 灯条是否移动的阈值 用灯条中心相邻帧移动距离判断 单位像素
            int m_left_light_still = 0, m_right_light_still = 0;
            cv::Point2f m_last_left_center = cv::Point2f(-1.f, -1.f), m_last_right_center = cv::Point2f(-1.f, -1.f);
            cv::Point2f m_last_bl_light, m_last_tl_light, m_last_tr_light, m_last_br_light;

            int m_kernel_thres = 250; // 核心初筛阈值
            int m_grad_thres = 250;   // 筛选边缘的y方向梯度阈值 梯度大于此阈值的点为边缘
            float m_left_top_y = 0.0, m_left_bottom_y = 0.0;
            float m_left_top_x_low = 0.0, m_left_top_x_high = 0.0, m_left_bottom_x_low = 0.0, m_left_bottom_x_high = 0.0;
            float m_right_top_y = 0.0, m_right_bottom_y = 0.0;
            float m_right_top_x_low = 0.0, m_right_top_x_high = 0.0, m_right_bottom_x_low = 0.0, m_right_bottom_x_high = 0.0;

            void solveKernelProblem();   // 通过核心高灰度值部分寻找角点
            void findVertices();         // 得出角点
    };

}
