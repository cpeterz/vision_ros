#ifndef COMPOSITION__TALKER_COMPONENT_HPP_
#define COMPOSITION__TALKER_COMPONENT_HPP_

#include "UsbCaptureSystem.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "base_interfaces/msg/mat_with_time.hpp"
#include "base_interfaces/srv/gimbal_pose.hpp"
#include "base_interfaces/srv/trigger.hpp"
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;

namespace wmj
{
    class camera_node : public rclcpp::Node
    {
    public:
        explicit camera_node(const rclcpp::NodeOptions & options);

        //利用 ros2_get_img 获取图像，传入类型为 sensor_msgs::msg::Image ，返回为 cv::Mat 类型
        static cv::Mat ros2_get_img(sensor_msgs::msg::Image img);


    private:
        //创建发布者
        rclcpp::Publisher<base_interfaces::msg::MatWithTime>::SharedPtr pub_image;
        
        //创建时间回调函数
        void timer_call_back();
        
        //创建定时器
        rclcpp::TimerBase::SharedPtr timer_;
        
        //获取相机数量
        int camera_num = 0;
        
        size_t count_;
        
        //声明相机用于读图
        std::shared_ptr<wmj::UsbCaptureSystem> capture ;
        
        //读取图像变量
        std::vector<wmj::MatWithTime> frame;

         // 程序序列
        int i_frame = 0;

        // IMU信息丢包次数
        int IMULostTIme = 0;

        // IMU时间戳
        double last_imu_timestamp, cur_imu_timestamp;

        // 控制
        //  std::shared_ptr<wmj::WMJRobotControl> control;

        // 云台位姿客户端
        rclcpp::Client<base_interfaces::srv::GimbalPose>::SharedPtr GimbalPose_Client;

        // 硬触发开启客户端
        rclcpp::Client<base_interfaces::srv::Trigger>::SharedPtr Trigger_Client;

        // Gimbalpose服务端回调函数
        void GimbalPose_callback(rclcpp::Client<base_interfaces::srv::GimbalPose>::SharedFuture response);

        // Trigger服务端回调函数
        void Trigger_callback(rclcpp::Client<base_interfaces::srv::Trigger>::SharedFuture response);

        // 是否开启硬触发（默认软触发）
        bool Trigger = false;

        //当前硬触发状态(默认false为未开启)
        bool Trigger_Status = false;

        // 发布消息
        std::unique_ptr<base_interfaces::msg::MatWithTime, std::default_delete<base_interfaces::msg::MatWithTime>> msg = std::make_unique<base_interfaces::msg::MatWithTime>();
    };
}

#endif
