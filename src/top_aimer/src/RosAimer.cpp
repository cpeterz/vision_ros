#include "top_aimer/RosAimer.hpp"

namespace wmj
{
    Top_Aimer::Top_Aimer(std::string name) : rclcpp::Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "Top_Aimer is working!");
        // 初始化Aimer实例
        top_aimer = std::make_shared<wmj::Aimer>();
        // 创建各类发布接受者
        pub_GimbalPose = this->create_publisher<base_interfaces::msg::GimbalPose>("GimbalPose", 10);
        pub_roi = this->create_publisher<base_interfaces::msg::Roi>("my_roi", 10);
        sub_armor = this->create_subscription<base_interfaces::msg::Armors>("Armors", 10, std::bind(&Top_Aimer::armors_call_back, this, std::placeholders::_1));
        // 创建GimbalPose订阅者
        sub_gimbalpose = this->create_subscription<base_interfaces::msg::GimbalPose>("GimbalPose", 10, std::bind(&Top_Aimer::GimbalPose_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Top_Aimer works successfully");
    }

    void Top_Aimer::armors_call_back(const base_interfaces::msg::Armors::SharedPtr msg)
    {
        Armor a;
        for (int i = 0; i < msg->num; i++)
        {
            a.m_position.x = msg->position_x[i];
            a.m_position.y = msg->position_y[i];
            a.m_position.z = msg->position_z[i];
            a.m_yaw_angle = msg->yaw_angle[i];
            a.m_id = msg->id[i];
            a.m_armor_type = ARMORTYPE(msg->armor_type[i]);
            a.m_color = _COLOR(msg->color[i]);
            a.m_time_seq = msg->time_seq[i];
            armors.push_back(a);
        }
        //硬触发在此处给GimbalPose赋值
        if (msg->trigger)
        {
            cur_pose.pitch = msg->pose.pitch;
            cur_pose.yaw = msg->pose.yaw;
            cur_pose.roll = msg->pose.roll;
            cur_pose.timestamp = msg->pose.timestamp;
        }
        // 核心函数，获取位姿
        target_pose = top_aimer->getTargetPose(armors, cur_pose, bullet_speed);

        roi = top_aimer->getLeftROI();
        base_interfaces::msg::Roi roi_msg;
        roi_msg.roi_height = roi.height;
        roi_msg.roi_width = roi.width;
        roi_msg.roi_x = roi.x;
        roi_msg.roi_y = roi.y;
        pub_roi->publish(roi_msg);
        RCLCPP_INFO(this->get_logger(), "roi message has been sent!!!");

        base_interfaces::msg::GimbalPose GimbalPose_msg;
        GimbalPose_msg.pitch = target_pose.pitch;
        GimbalPose_msg.yaw = target_pose.yaw;
        GimbalPose_msg.roll = target_pose.roll;
        GimbalPose_msg.timestamp = target_pose.timestamp;
        pub_GimbalPose->publish(GimbalPose_msg);
        RCLCPP_INFO(this->get_logger(), "GimbalPose message has been sent!!!");
    }

    void Top_Aimer::GimbalPose_callback(const base_interfaces::msg::GimbalPose::SharedPtr msg)
    {
        cur_pose.pitch = msg->pitch;
        cur_pose.roll = msg->roll;
        cur_pose.yaw = msg->yaw;
        cur_pose.timestamp = msg->timestamp;
        RCLCPP_INFO(this->get_logger(), "target_pose successfully get!!");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<wmj::Top_Aimer>("top_aimer");
    /* 运行节点，并检测状态*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}