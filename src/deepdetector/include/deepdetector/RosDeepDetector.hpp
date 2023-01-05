#ifndef COMPOSITION__LISTENER_COMPONENT_HPP_
#define COMPOSITION__LISTENER_COMPONENT_HPP_

#pragma once
#include "DeepDetector.hpp"
#include "base_interfaces/msg/mat_with_time.hpp"
#include "base_interfaces/srv/gimbal_pose.hpp"
#include "base_interfaces/msg/armors.hpp"
#include "base_interfaces/msg/roi.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace wmj
{
    class DeepDetector_Node : public rclcpp::Node
    {
    public:
        explicit DeepDetector_Node(const rclcpp::NodeOptions &options);

    private:
        size_t count_;

        // 声明图像订阅者
        rclcpp::Subscription<base_interfaces::msg::MatWithTime>::SharedPtr sub_image;

        // 声明ROI区域，用于储存libControl传过来的数据
        cv::Rect2d roi;

        // 声明ROI区域订阅者
        rclcpp::Subscription<base_interfaces::msg::Roi>::SharedPtr sub_roi;

        // 声明armors发布者
        rclcpp::Publisher<base_interfaces::msg::Armors>::SharedPtr pub_armor;

        // 声明图像订阅回调函数
        void image_call_back(const base_interfaces::msg::MatWithTime::SharedPtr msg);

        // 声明ROI订阅回调函数
        void roi_call_back(const base_interfaces::msg::Roi::SharedPtr msg);

        // 声明图像转换函数
        static cv::Mat ros2_get_img(sensor_msgs::msg::Image img);

        // 声明深度实例
        std::shared_ptr<wmj::DeepDetector> deep_detector;

        // 声明深度参数
        MatWithTime m_img;
    };

}

#endif
