#include "base_interfaces/msg/gimbal_pose.hpp"
#include "base_interfaces/msg/armors.hpp"
#include "base_interfaces/msg/roi.hpp"
#include "rclcpp/rclcpp.hpp"
#include "Aimer.hpp"

using std::placeholders::_1;

namespace wmj
{
    class Top_Aimer : public rclcpp::Node
    {
    public:
        explicit Top_Aimer(std::string name);

    private:
        // 声明ROI区域发布者
        rclcpp::Publisher<base_interfaces::msg::Roi>::SharedPtr pub_roi;

        // 声明armors订阅者
        rclcpp::Subscription<base_interfaces::msg::Armors>::SharedPtr sub_armor;

        // 声明发布者
        rclcpp::Publisher<base_interfaces::msg::GimbalPose>::SharedPtr pub_GimbalPose;

        // 声明armors回调函数
        void armors_call_back(const base_interfaces::msg::Armors::SharedPtr armors);

        // Gimbalpose服务端回调函数
        void GimbalPose_callback(const base_interfaces::msg::GimbalPose::SharedPtr msg);

        // 声明gimbalpose订阅者
        rclcpp::Subscription<base_interfaces::msg::GimbalPose>::SharedPtr sub_gimbalpose;

        // 声明Aimer实例
        std::shared_ptr<wmj::Aimer> top_aimer;
        // 声明各类变量用于参数赋值
        Armors armors;
        double bullet_speed = 5;
        GimbalPose cur_pose;
        GimbalPose target_pose;
        cv::Rect2d roi;
    };
}