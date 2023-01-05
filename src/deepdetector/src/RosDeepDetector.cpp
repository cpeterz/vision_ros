#include "deepdetector/RosDeepDetector.hpp"

namespace wmj
{
    DeepDetector_Node::DeepDetector_Node(const rclcpp::NodeOptions &options) : rclcpp::Node("listener", options)
    {
        RCLCPP_INFO(this->get_logger(), "detector_component init is working ");
        // 初始化深度实例
        deep_detector = std::make_shared<wmj::DeepDetector>();
        // 初始化roi,使用默认值，后面要改
        roi = cv::Rect_<double>(0, 0, 1279, 1023);
        // 创建图像订阅者
        sub_image = this->create_subscription<base_interfaces::msg::MatWithTime>("camera_image", 10, std::bind(&DeepDetector_Node::image_call_back, this, std::placeholders::_1));
        // 创建ROI订阅者
        sub_roi = this->create_subscription<base_interfaces::msg::Roi>("my_roi", 10, std::bind(&DeepDetector_Node::roi_call_back, this, std::placeholders::_1));
        // 创建Armor发布者
        pub_armor = this->create_publisher<base_interfaces::msg::Armors>("Armors", 10);
        RCLCPP_INFO(this->get_logger(), "detector_component successfully init ");
    }

    // 获取图像后利用深度结合roi进行识别，然后将识别结果Armors传给控制节点
    void DeepDetector_Node::image_call_back(const base_interfaces::msg::MatWithTime::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "begin deepdetect...");
        // 获取图像
        cv::Mat image = DeepDetector_Node::ros2_get_img(msg->image);
        // 赋值
        m_img = MatWithTime(image, msg->time_stamp, "single");
        // 核心函数，做深度识别
        // std::cout << "Roi.x:" << roi.x << "    " << "Roi.y:" << roi.y << "  " << "Roi.width:" << roi.width  <<"  "<< "Roi.height:" << roi.height;
        deep_detector->DeepDetectSingle(m_img, roi);

        RCLCPP_INFO(this->get_logger(), "end deepdetect...");

        // 将接收到的数据储存在armors_msg中发出去
        base_interfaces::msg::Armors armors_msg;
        armors_msg.pose = msg->pose;
        armors_msg.num = (deep_detector->m_armors).size();
        if (armors_msg.num == 0)
        {
            armors_msg.position_x.push_back(0);
            armors_msg.position_y.push_back(0);
            armors_msg.position_z.push_back(0);
            armors_msg.yaw_angle.push_back(0);
            armors_msg.time_seq.push_back(0);
            armors_msg.id.push_back(1);
            armors_msg.armor_type.push_back(0);
            armors_msg.color.push_back(0);
        }
        for (int i = 0; i < armors_msg.num; i++)
        {
            armors_msg.position_x.push_back(deep_detector->m_armors[i].m_position.x);
            armors_msg.position_y.push_back(deep_detector->m_armors[i].m_position.y);
            armors_msg.position_z.push_back(deep_detector->m_armors[i].m_position.z);
            armors_msg.yaw_angle.push_back(deep_detector->m_armors[i].m_yaw_angle);
            armors_msg.time_seq.push_back(deep_detector->m_armors[i].m_time_seq);
            armors_msg.id.push_back(deep_detector->m_armors[i].m_id);
            armors_msg.armor_type.push_back(deep_detector->m_armors[i].m_armor_type);
            armors_msg.color.push_back(deep_detector->m_armors[i].m_color);

            std::cout << "x____:" << deep_detector->m_armors[0].m_position.x << "  "
                      << "yaw____:" << deep_detector->m_armors[0].m_yaw_angle << "  "
                      << "id____:" << deep_detector->m_armors[0].m_id << "  "
                      << "timestamp___:" << deep_detector->m_armors[0].m_angle << "  "
                      << "type___:" << deep_detector->m_armors[0].m_detectedtype << "  "
                      << "color___:" << deep_detector->m_armors[0].m_color << "   " << std::endl;
            for (int i = 0; i < (deep_detector->m_armors[0].m_vertices).size(); i++)
                std::cout << "vertice x" << i << "__" << deep_detector->m_armors[0].m_vertices[i].x << "   "
                          << "vertice y" << i << "__" << deep_detector->m_armors[0].m_vertices[i].y << std::endl;
        }
        armors_msg.trigger = msg->trigger;
        pub_armor->publish(armors_msg);
        // test

        std::cout << "armors_msg.num:" << armors_msg.num << std::endl;
        std::cout << "x:" << armors_msg.position_x[0] << "    "
                  << "yaw:" << armors_msg.yaw_angle[0] << std::endl;
        // cv::imshow("image", image);
        // cv::waitKey(1);
        RCLCPP_INFO(this->get_logger(), "armor msg has been sent!!");
    }

    // 接受消息，储存新的ROI值
    void DeepDetector_Node::roi_call_back(const base_interfaces::msg::Roi::SharedPtr msg)
    {
        std::cout << "Roi.x:" << msg->roi_x << "    "
                  << "Roi.y:" << msg->roi_y << "  "
                  << "Roi.width:" << msg->roi_width << "  "
                  << "Roi.height:" << msg->roi_height << std::endl;
        ;
        if (msg->roi_width == 0 || msg->roi_height == 0)
            roi = cv::Rect_<double>(0, 0, 1279, 1023);
        else
            roi = cv::Rect_<double>(msg->roi_x, msg->roi_y, msg->roi_width, msg->roi_height);
        RCLCPP_INFO(this->get_logger(), "get roi message !!!");
    }

    cv::Mat DeepDetector_Node::ros2_get_img(sensor_msgs::msg::Image img)
    {
        // 声明 cv::image 指针，内含cv::Mat
        cv_bridge::CvImageConstPtr cv_ptr;
        // toCvShare() 使用
        // sensor_msgs::msg::Image::ConstPtr img_ptr;

        // 获取图像，最终图像通过 cv_ptr->image 获取，为cv::Mat型
        try
        {
            // 把msg里的 Image 转换为 cv::mat类型 , 有两种方式 toCvCopy() 和 toCvShare() ,toCvShare() 虽然可以实现直接引用不用拷贝，
            // 但需要创建 sensors_msgs::ImageConst 指针，经测试二者差别不大，故用toCvCopy().后续还可改进.

            cv_ptr = cv_bridge::toCvCopy(img);

            // toCvShare() 方法
            // img_ptr = std::make_shared<const sensor_msgs::msg::Image>(msg->image);
            // cv_ptr = cv_bridge::toCvCopy(img_ptr);
        }
        catch (cv_bridge::Exception &e)
        {
            printf("cv_bridge exception:%s", e.what());
        }
        return (cv_ptr->image);
    }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(wmj::DeepDetector_Node)