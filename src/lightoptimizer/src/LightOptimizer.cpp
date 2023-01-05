#include "lightoptimizer/LightOptimizer.hpp"

namespace wmj
{
    /**
     * @brief 从左右灯条的旋转矩形获取所有像素点坐标
     * 
     * @param left_rect  左灯条
     * @param right_rect 右灯条
     * @param roi        原roi
     * @param gray       原灰度图
     */
    void LightOptimizer::LightOptimization(cv::RotatedRect left_rect, cv::RotatedRect right_rect, cv::Rect2d &roi, cv::Mat &gray)
    {
        double begin_time = wmj::now();

        m_left_rect = left_rect;
        m_right_rect = right_rect;

        m_roi_x = roi.x; // roi's top left x
        m_roi_y = roi.y; // roi's top left y

        if(m_debug)
        {
            std::cout << "roi size: " << roi.size() << " roi position: [" << roi.x << "," << roi.y << "]\n";
            std::cout << "gray size: " << gray.size() << std::endl;
        }
        m_gray = gray;

        getLeftRectPoints();
        
        getRightRectPoints();

        solveKernelProblem();

        findVertices();

        double end_time = wmj::now();
        std::cout << "Light opt total cost: " << end_time - begin_time << std::endl;
    }

    /**
     * @brief 从左灯条的旋转矩形获取所有像素点坐标
     */
    void LightPreProcess::getLeftRectPoints()
    {
        m_left_points.clear();
        m_left_points_.clear();
        m_left_channel_sum = 0;

        float left_angle = m_left_rect.angle;
        m_left_rot_mat << std::sin(left_angle / 180.f * M_PI), std::cos(left_angle / 180.f * M_PI), 
                            -std::cos(left_angle / 180.f * M_PI), std::sin(left_angle / 180.f * M_PI);
        m_left_rot_mat_inv = m_left_rot_mat.inverse();

        m_left_center = m_left_rect.center;
        cv::Point2f left_4points[4];
        m_left_rect.points(left_4points);
        for(int i = 0; i < 4; i++)
        {
            left_4points[i].x += (m_x_extend_factor * (left_4points[i].x - m_left_center.x));
            left_4points[i].y += (m_y_extend_factor * (left_4points[i].y - m_left_center.y));
        }
        cv::Point2f left_bounding_tl = m_left_rect.boundingRect2f().tl();
        cv::Point2f left_bounding_br = m_left_rect.boundingRect2f().br();

        if(m_debug)
        {
            std::cout << "left_bounding_tl:" << left_bounding_tl << std::endl;
            std::cout << "left_bounding_br:" << left_bounding_br << std::endl;
        }
        // 扩展采样区域
        left_bounding_tl.x += (m_x_extend_factor * (left_bounding_tl.x - m_left_center.x));
        left_bounding_tl.y += (m_y_extend_factor * (left_bounding_tl.y - m_left_center.y));
        left_bounding_br.x += (m_x_extend_factor * (left_bounding_br.x - m_left_center.x));
        left_bounding_br.y += (m_y_extend_factor * (left_bounding_br.y - m_left_center.y));
        if(m_debug)
        {
            std::cout << "extended left_bounding_tl:" << left_bounding_tl << std::endl;
            std::cout << "extended left_bounding_br:" << left_bounding_br << std::endl;
        }

        std::vector<float> left_4k(4);
        std::vector<float> left_4b(4);
        std::vector<int> left_4directions(4);
        for(int i = 0; i < 4; i++)
        {
            int j = (i + 1) % 4;
            left_4k[i] = (left_4points[i].y - left_4points[j].y) / (left_4points[i].x - left_4points[j].x);
            left_4b[i] = left_4points[i].y - left_4k[i] * left_4points[i].x;
            if((left_4k[i] * m_left_center.x - m_left_center.y + left_4b[i]) < 0)
            {
                left_4directions[i] = -1;
            } else {
                left_4directions[i] = 1;
            }
        }
        // 存入位于采样区域的点坐标
        for(float i = left_bounding_tl.x; i < left_bounding_br.x; i += m_pick_step)
        {
            for(float j = left_bounding_tl.y; j < left_bounding_br.y; j += m_pick_step)
            {
                int ifIN = 1;
                for(int p = 0; p < 4; p++)
                {
                    if(left_4directions[p] * (left_4k[p] * i - j + left_4b[p]) < 0)
                    {
                        ifIN = 0;
                        break;
                    }
                }
                if(ifIN)
                {
                    // 防止采样越界
                    if(int(j - m_roi_y) >= 0 && int(j - m_roi_y) < m_gray.rows && int(i - m_roi_x) >= 0 && int(i - m_roi_x) < m_gray.cols)
                    {
                        m_left_points.push_back(cv::Point2f(i, j));
                        m_left_channel_sum += (double)m_gray.at<u_char>(int(j - m_roi_y), int(i - m_roi_x));
                        float srcX = i - m_left_center.x, srcY = j - m_left_center.y;
                        m_left_points_.push_back(cv::Point2f(m_left_rot_mat_inv(0, 0) * srcX + m_left_rot_mat_inv(0, 1) * srcY,
                                                             m_left_rot_mat_inv(1, 0) * srcX + m_left_rot_mat_inv(1, 1) * srcY));
                        if(m_debug)
                        {
                            std::cout << "coor left_angle: " << left_angle << std::endl;
                            std::cout << "coor src: " << srcX << "," << srcY << "\n";
                            std::cout << "coor dst:[" << m_left_rot_mat_inv(0, 0) * srcX + m_left_rot_mat_inv(0, 1) * srcY << "," << m_left_rot_mat_inv(1, 0) * srcX + m_left_rot_mat_inv(1, 1) * srcY << "]\n";
                        }
                    }
                }
            }
        }

        if(m_debug)
        {
            std::ofstream ofsl("left_points_in_image.txt");
            for(int i = 0; i < m_left_points.size(); i++)
            {
                ofsl << (int)m_left_points[i].x << " " << (int)m_left_points[i].y << std::endl;
            }
            ofsl.close();

            std::cout << "left_channel_sum=" << m_left_channel_sum << std::endl;
            
            ofsl.open("left_points_in_light.txt");
            for(int i = 0; i < m_left_points_.size(); i++)
            {
                ofsl << m_left_points_[i].x << " " << m_left_points_[i].y << std::endl;
            }
            ofsl.close();

            ofsl.open("left_channels_in_light.txt");
            for(int i = 0; i < m_left_points_.size(); i++)
            {
                ofsl << m_left_points_[i].x << " " << m_left_points_[i].y << " " << (int)m_gray.at<u_char>(int(m_left_points[i].y - m_roi_y), int(m_left_points[i].x - m_roi_x)) << std::endl;
            }
            ofsl.close();
        }
    }

    /**
     * @brief 从右灯条的旋转矩形获取所有像素点坐标
     */
    void LightPreProcess::getRightRectPoints()
    {
        m_right_points.clear();
        m_right_points_.clear();
        m_right_channel_sum = 0;

        float right_angle = m_right_rect.angle;
        m_right_rot_mat << std::sin(right_angle / 180.f * M_PI), std::cos(right_angle / 180.f * M_PI), 
                            -std::cos(right_angle / 180.f * M_PI), std::sin(right_angle / 180.f * M_PI);
        m_right_rot_mat_inv = m_right_rot_mat.inverse();

        m_right_center = m_right_rect.center;
        cv::Point2f right_4points[4];
        m_right_rect.points(right_4points);
        for(int i = 0; i < 4; i++)
        {
            right_4points[i].x += (m_x_extend_factor * (right_4points[i].x - m_right_center.x));
            right_4points[i].y += (m_y_extend_factor * (right_4points[i].y - m_right_center.y));
        }
        
        cv::Point2f right_bounding_tl = m_right_rect.boundingRect2f().tl();
        cv::Point2f right_bounding_br = m_right_rect.boundingRect2f().br();
        if(m_debug)
        {
            std::cout << "right_bounding_tl:" << right_bounding_tl << std::endl;
            std::cout << "right_bounding_br:" << right_bounding_br << std::endl;
        }
        // 扩展采样区域
        right_bounding_tl.x += (m_x_extend_factor * (right_bounding_tl.x - m_right_center.x));
        right_bounding_tl.y += (m_y_extend_factor * (right_bounding_tl.y - m_right_center.y));
        right_bounding_br.x += (m_x_extend_factor * (right_bounding_br.x - m_right_center.x));
        right_bounding_br.y += (m_y_extend_factor * (right_bounding_br.y - m_right_center.y));
        if(m_debug)
        {
            std::cout << "extended right_bounding_tl:" << right_bounding_tl << std::endl;
            std::cout << "extended right_bounding_br:" << right_bounding_br << std::endl;
        }

        std::vector<float> right_4k(4);
        std::vector<float> right_4b(4);
        std::vector<int> right_4directions(4);
        for(int i = 0; i < 4; i++)
        {
            int j = (i + 1) % 4;
            right_4k[i] = (right_4points[i].y - right_4points[j].y) / (right_4points[i].x - right_4points[j].x);
            right_4b[i] = right_4points[i].y - right_4k[i] * right_4points[i].x;
            if((right_4k[i] * m_right_center.x - m_right_center.y + right_4b[i]) < 0)
            {
                right_4directions[i] = -1;
            } else {
                right_4directions[i] = 1;
            }
        }
        // 存入位于采样区域的点坐标
        for(float i = right_bounding_tl.x; i < right_bounding_br.x; i += m_pick_step)
        {
            for(float j = right_bounding_tl.y; j < right_bounding_br.y; j += m_pick_step)
            {
                int ifIN = 1;
                for(int p = 0; p < 4; p++)
                {
                    if(right_4directions[p] * (right_4k[p] * i - j + right_4b[p]) < 0)
                    {
                        ifIN = 0;
                        break;
                    }
                }
                if(ifIN)
                {
                    // 防止采样越界
                    if(int(j - m_roi_y) >= 0 && int(j - m_roi_y) < m_gray.rows && int(i - m_roi_x) >= 0 && int(i - m_roi_x) < m_gray.cols)
                    {
                        m_right_points.push_back(cv::Point2f(i, j));
                        m_right_channel_sum += (double)m_gray.at<u_char>(int(j - m_roi_y), int(i - m_roi_x));
                        float srcX = i - m_right_center.x, srcY = j - m_right_center.y;
                        m_right_points_.push_back(cv::Point2f(m_right_rot_mat_inv(0, 0) * srcX + m_right_rot_mat_inv(0, 1) * srcY,
                                                              m_right_rot_mat_inv(1, 0) * srcX + m_right_rot_mat_inv(1, 1) * srcY));
                        if(m_debug)
                        {
                            std::cout << "coor right_angle: " << right_angle << std::endl;
                            std::cout << "coor src: " << srcX << "," << srcY << "\n";
                            std::cout << "coor dst:[" << m_right_rot_mat_inv(0, 0) * srcX + m_right_rot_mat_inv(0, 1) * srcY << "," << m_right_rot_mat_inv(1, 0) * srcX + m_right_rot_mat_inv(1, 1) * srcY << "]\n";
                        }
                    }
                }
            }
        }

        if(m_debug)
        {
            std::ofstream ofsr("right_points_in_image.txt");
            for(int i = 0; i < m_right_points.size(); i++)
            {
                ofsr << (int)m_right_points[i].x << " " << (int)m_right_points[i].y << std::endl;
            }
            ofsr.close();

            std::cout << "right_channel_sum=" << m_right_channel_sum << std::endl;
        
            ofsr.open("right_points_in_light.txt");
            for(int i = 0; i < m_right_points_.size(); i++)
            {
                ofsr << m_right_points_[i].x << " " << m_right_points_[i].y << std::endl;
            }
            ofsr.close();

            ofsr.open("right_channels_in_light.txt");
            for(int i = 0; i < m_right_points_.size(); i++)
            {
                ofsr << m_right_points_[i].x << " " << m_right_points_[i].y << " " << (int)m_gray.at<u_char>(int(m_right_points[i].y - m_roi_y), int(m_right_points[i].x - m_roi_x)) << std::endl;
            }
            ofsr.close();
        }
    }

    /**
     * @brief 通过核心高灰度值部分寻找角点
     */
    void LightOptimizer::solveKernelProblem()
    {
	    m_left_top_y = m_left_top_x_low = m_left_top_x_high = 0.0;
	    m_left_bottom_y = m_left_bottom_x_low = m_left_bottom_x_high = 0.0;
        for(int i = 0; i < m_left_points.size(); i++)
        {
            if((int(m_gray.at<u_char>(int(m_left_points[i].y - m_roi_y), int(m_left_points[i].x - m_roi_x)))) > m_kernel_thres)
            {
                std::vector<cv::Point2f> sobel_points(6);
                sobel_points[0] = cv::Point2f(m_left_points_[i].x - 1.0, m_left_points_[i].y - 1.0);
                sobel_points[1] = cv::Point2f(m_left_points_[i].x,       m_left_points_[i].y - 1.0);
                sobel_points[2] = cv::Point2f(m_left_points_[i].x + 1.0, m_left_points_[i].y - 1.0);
                sobel_points[3] = cv::Point2f(m_left_points_[i].x - 1.0, m_left_points_[i].y + 1.0);
                sobel_points[4] = cv::Point2f(m_left_points_[i].x,       m_left_points_[i].y + 1.0);
                sobel_points[5] = cv::Point2f(m_left_points_[i].x + 1.0, m_left_points_[i].y + 1.0);
                sobel_points[0] = cv::Point2f(m_left_center.x + m_left_rot_mat(0, 0) * sobel_points[0].x + m_left_rot_mat(0, 1) * sobel_points[0].y,
                                              m_left_center.y + m_left_rot_mat(1, 0) * sobel_points[0].x + m_left_rot_mat(1, 1) * sobel_points[0].y);
                sobel_points[1] = cv::Point2f(m_left_center.x + m_left_rot_mat(0, 0) * sobel_points[1].x + m_left_rot_mat(0, 1) * sobel_points[1].y,
                                              m_left_center.y + m_left_rot_mat(1, 0) * sobel_points[1].x + m_left_rot_mat(1, 1) * sobel_points[1].y);
                sobel_points[2] = cv::Point2f(m_left_center.x + m_left_rot_mat(0, 0) * sobel_points[2].x + m_left_rot_mat(0, 1) * sobel_points[2].y,
                                              m_left_center.y + m_left_rot_mat(1, 0) * sobel_points[2].x + m_left_rot_mat(1, 1) * sobel_points[2].y);
                sobel_points[3] = cv::Point2f(m_left_center.x + m_left_rot_mat(0, 0) * sobel_points[3].x + m_left_rot_mat(0, 1) * sobel_points[3].y,
                                              m_left_center.y + m_left_rot_mat(1, 0) * sobel_points[3].x + m_left_rot_mat(1, 1) * sobel_points[3].y);
                sobel_points[4] = cv::Point2f(m_left_center.x + m_left_rot_mat(0, 0) * sobel_points[4].x + m_left_rot_mat(0, 1) * sobel_points[4].y,
                                              m_left_center.y + m_left_rot_mat(1, 0) * sobel_points[4].x + m_left_rot_mat(1, 1) * sobel_points[4].y);
                sobel_points[5] = cv::Point2f(m_left_center.x + m_left_rot_mat(0, 0) * sobel_points[5].x + m_left_rot_mat(0, 1) * sobel_points[5].y,
                                              m_left_center.y + m_left_rot_mat(1, 0) * sobel_points[5].x + m_left_rot_mat(1, 1) * sobel_points[5].y);
                std::vector<int> sobel_channels(6);
                sobel_channels[0] = (int)m_gray.at<u_char>(int(sobel_points[0].y - m_roi_y), int(sobel_points[0].x - m_roi_x));
                sobel_channels[1] = (int)m_gray.at<u_char>(int(sobel_points[1].y - m_roi_y), int(sobel_points[1].x - m_roi_x));
                sobel_channels[2] = (int)m_gray.at<u_char>(int(sobel_points[2].y - m_roi_y), int(sobel_points[2].x - m_roi_x));
                sobel_channels[3] = (int)m_gray.at<u_char>(int(sobel_points[3].y - m_roi_y), int(sobel_points[3].x - m_roi_x));
                sobel_channels[4] = (int)m_gray.at<u_char>(int(sobel_points[4].y - m_roi_y), int(sobel_points[4].x - m_roi_x));
                sobel_channels[5] = (int)m_gray.at<u_char>(int(sobel_points[5].y - m_roi_y), int(sobel_points[5].x - m_roi_x));
                if(m_left_points_[i].y < m_left_top_y) // 在上边缘
                {
                    int grad = sobel_channels[3] - sobel_channels[0] + 2 * (sobel_channels[4] - sobel_channels[1]) + sobel_channels[5] - sobel_channels[2];
                    if(grad < m_grad_thres) { // 梯度过大 是边缘点
                        m_left_top_y = m_left_points_[i].y;
                        m_left_top_x_high = std::max(m_left_top_x_high, m_left_points_[i].x);
                        m_left_top_x_low  = std::min(m_left_top_x_low, m_left_points_[i].x);
                    }
                    if(m_debug) {
                        std::cout << "left top grad = " << grad << std::endl;
                        std::cout << sobel_channels[0] << " " << sobel_channels[1] << " " << sobel_channels[2] << "\n" << sobel_channels[3] << " " << sobel_channels[4] << " " << sobel_channels[5] << "\n";
                    }
                } else if(m_left_points_[i].y > m_left_bottom_y) { // 在下边缘
                    int grad = sobel_channels[0] - sobel_channels[3] + 2 * (sobel_channels[1] - sobel_channels[4]) + sobel_channels[2] - sobel_channels[5];
                    if(grad < m_grad_thres) { // 梯度过大 是边缘点
                        m_left_bottom_y = m_left_points_[i].y;
                        m_left_bottom_x_high = std::max(m_left_bottom_x_high, m_left_points_[i].x);
                        m_left_bottom_x_low  = std::min(m_left_bottom_x_low, m_left_points_[i].x);
                    }
                    if(m_debug) {
                        std::cout << "left bottom grad = " << grad << std::endl;
                        std::cout << sobel_channels[0] << " " << sobel_channels[1] << " " << sobel_channels[2] << "\n" << sobel_channels[3] << " " << sobel_channels[4] << " " << sobel_channels[5] << "\n";
                    }
                }
            }
        }

	    m_right_top_y = m_right_top_x_low = m_right_top_x_high = 0.0;
	    m_right_bottom_y = m_right_bottom_x_low = m_right_bottom_x_high = 0.0;
        for(int i = 0; i < m_right_points.size(); i++)
        {
            if((int(m_gray.at<u_char>(int(m_right_points[i].y - m_roi_y), int(m_right_points[i].x - m_roi_x)))) > m_kernel_thres)
            {
                std::vector<cv::Point2f> sobel_points(6);
                sobel_points[0] = cv::Point2f(m_right_points_[i].x - 1.0, m_right_points_[i].y - 1.0);
                sobel_points[1] = cv::Point2f(m_right_points_[i].x,       m_right_points_[i].y - 1.0);
                sobel_points[2] = cv::Point2f(m_right_points_[i].x + 1.0, m_right_points_[i].y - 1.0);
                sobel_points[3] = cv::Point2f(m_right_points_[i].x - 1.0, m_right_points_[i].y + 1.0);
                sobel_points[4] = cv::Point2f(m_right_points_[i].x,       m_right_points_[i].y + 1.0);
                sobel_points[5] = cv::Point2f(m_right_points_[i].x + 1.0, m_right_points_[i].y + 1.0);
                sobel_points[0] = cv::Point2f(m_right_center.x + m_right_rot_mat(0, 0) * sobel_points[0].x + m_right_rot_mat(0, 1) * sobel_points[0].y,
                                              m_right_center.y + m_right_rot_mat(1, 0) * sobel_points[0].x + m_right_rot_mat(1, 1) * sobel_points[0].y);
                sobel_points[1] = cv::Point2f(m_right_center.x + m_right_rot_mat(0, 0) * sobel_points[1].x + m_right_rot_mat(0, 1) * sobel_points[1].y,
                                              m_right_center.y + m_right_rot_mat(1, 0) * sobel_points[1].x + m_right_rot_mat(1, 1) * sobel_points[1].y);
                sobel_points[2] = cv::Point2f(m_right_center.x + m_right_rot_mat(0, 0) * sobel_points[2].x + m_right_rot_mat(0, 1) * sobel_points[2].y,
                                              m_right_center.y + m_right_rot_mat(1, 0) * sobel_points[2].x + m_right_rot_mat(1, 1) * sobel_points[2].y);
                sobel_points[3] = cv::Point2f(m_right_center.x + m_right_rot_mat(0, 0) * sobel_points[3].x + m_right_rot_mat(0, 1) * sobel_points[3].y,
                                              m_right_center.y + m_right_rot_mat(1, 0) * sobel_points[3].x + m_right_rot_mat(1, 1) * sobel_points[3].y);
                sobel_points[4] = cv::Point2f(m_right_center.x + m_right_rot_mat(0, 0) * sobel_points[4].x + m_right_rot_mat(0, 1) * sobel_points[4].y,
                                              m_right_center.y + m_right_rot_mat(1, 0) * sobel_points[4].x + m_right_rot_mat(1, 1) * sobel_points[4].y);
                sobel_points[5] = cv::Point2f(m_right_center.x + m_right_rot_mat(0, 0) * sobel_points[5].x + m_right_rot_mat(0, 1) * sobel_points[5].y,
                                              m_right_center.y + m_right_rot_mat(1, 0) * sobel_points[5].x + m_right_rot_mat(1, 1) * sobel_points[5].y);
                std::vector<int> sobel_channels(6);
                sobel_channels[0] = (int)m_gray.at<u_char>(int(sobel_points[0].y - m_roi_y), int(sobel_points[0].x - m_roi_x));
                sobel_channels[1] = (int)m_gray.at<u_char>(int(sobel_points[1].y - m_roi_y), int(sobel_points[1].x - m_roi_x));
                sobel_channels[2] = (int)m_gray.at<u_char>(int(sobel_points[2].y - m_roi_y), int(sobel_points[2].x - m_roi_x));
                sobel_channels[3] = (int)m_gray.at<u_char>(int(sobel_points[3].y - m_roi_y), int(sobel_points[3].x - m_roi_x));
                sobel_channels[4] = (int)m_gray.at<u_char>(int(sobel_points[4].y - m_roi_y), int(sobel_points[4].x - m_roi_x));
                sobel_channels[5] = (int)m_gray.at<u_char>(int(sobel_points[5].y - m_roi_y), int(sobel_points[5].x - m_roi_x));
                if(m_right_points_[i].y < m_right_top_y) // 在上边缘
                {
                    int grad = sobel_channels[3] - sobel_channels[0] + 2 * (sobel_channels[4] - sobel_channels[1]) + sobel_channels[5] - sobel_channels[2];
                    if(grad < m_grad_thres) { // 梯度过大 是边缘点
                        m_right_top_y = m_right_points_[i].y;
                        m_right_top_x_high = std::max(m_right_top_x_high, m_left_points_[i].x);
                        m_right_top_x_low  = std::min(m_right_top_x_low, m_left_points_[i].x);
                    }
                    if(m_debug) {
                        std::cout << "right top grad = " << grad << std::endl;
                        std::cout << sobel_channels[0] << " " << sobel_channels[1] << " " << sobel_channels[2] << "\n" << sobel_channels[3] << " " << sobel_channels[4] << " " << sobel_channels[5] << "\n";
                    }
                } else if(m_right_points_[i].y > m_right_bottom_y) { // 在下边缘
                    int grad = sobel_channels[0] - sobel_channels[3] + 2 * (sobel_channels[1] - sobel_channels[4]) + sobel_channels[2] - sobel_channels[5];
                    if(grad < m_grad_thres) { // 梯度过大 是边缘点
                        m_right_bottom_y = m_right_points_[i].y;
                        m_right_bottom_x_high = std::max(m_right_bottom_x_high, m_left_points_[i].x);
                        m_right_bottom_x_low  = std::min(m_right_bottom_x_low, m_left_points_[i].x);
                    }
                    if(m_debug) {
                        std::cout << "right bottom grad = " << grad << std::endl;
                        std::cout << sobel_channels[0] << " " << sobel_channels[1] << " " << sobel_channels[2] << "\n" << sobel_channels[3] << " " << sobel_channels[4] << " " << sobel_channels[5] << "\n";
                    }
                }
            }
        }
    }
    
    /**
     * @brief 得出角点
     */
    void LightOptimizer::findVertices()
    {
        m_final_vertices.clear();
        m_left_bottom_y = std::min(std::fabs(m_left_bottom_y), std::fabs(m_left_top_y));
        m_left_top_y = -std::min(std::fabs(m_left_bottom_y), std::fabs(m_left_top_y));
        m_right_bottom_y = std::min(std::fabs(m_right_bottom_y), std::fabs(m_right_top_y));
        m_right_top_y = -std::min(std::fabs(m_right_bottom_y), std::fabs(m_right_top_y));
        cv::Point2f bl_light = cv::Point2f((m_left_bottom_x_high + m_left_bottom_x_low) / 2.0, m_left_bottom_y);
        cv::Point2f tl_light = cv::Point2f((m_left_top_x_high + m_left_top_x_low) / 2.0, m_left_top_y);
        cv::Point2f tr_light = cv::Point2f((m_right_top_x_low + m_right_top_x_high) / 2.0, m_right_top_y);
        cv::Point2f br_light = cv::Point2f((m_right_bottom_x_low + m_right_bottom_x_high) / 2.0, m_right_bottom_y);

        if(m_debug)
        {
            std::cout << "last&cur left_center: " << m_last_left_center << " & " << m_left_center << std::endl;
            std::cout << "last&cur right_center: " << m_last_right_center << " & " << m_right_center << std::endl;
        }

        if(m_last_left_center.x > 0.f && std::sqrt((m_last_left_center.x - m_left_center.x) * (m_last_left_center.x - m_left_center.x) + (m_last_left_center.y - m_left_center.y) * (m_last_left_center.y - m_left_center.y)) < m_light_moved_dis_thres) {
            m_left_light_still++;
            if(m_debug)
                std::cout << "left use last vertices\n";

            m_last_bl_light = m_last_bl_light + (bl_light - m_last_bl_light) / m_left_light_still;
            m_last_tl_light = m_last_tl_light + (tl_light - m_last_tl_light) / m_left_light_still;
            bl_light = m_last_bl_light;
            tl_light = m_last_tl_light;

            m_last_left_center = m_left_center;
        } else {
            m_left_light_still = 1;
            m_last_bl_light = bl_light;
            m_last_tl_light = tl_light;
            m_last_left_center = m_left_center;
        }
        if(m_last_right_center.x > 0.f && std::sqrt((m_last_right_center.x - m_right_center.x) * (m_last_right_center.x - m_right_center.x) + (m_last_right_center.y - m_right_center.y) * (m_last_right_center.y - m_right_center.y)) < m_light_moved_dis_thres) {
            m_right_light_still++;
            if(m_debug)
                std::cout << "right use last vertices\n";

            m_last_tr_light = m_last_tr_light + (tr_light - m_last_tr_light) / m_right_light_still;
            m_last_br_light = m_last_br_light + (br_light - m_last_br_light) / m_right_light_still;
            tr_light = m_last_tr_light;
            br_light = m_last_br_light;

            m_last_right_center = m_right_center;
        } else {
            m_right_light_still = 1;
            m_last_tr_light = tr_light;
            m_last_br_light = br_light;
            m_last_right_center = m_right_center;
        }
        
        if(m_debug)
            std::cout << "vertices in light: " << tl_light << " " << bl_light << " " << tr_light << " " << br_light << "\n";

        cv::Point2f tl_image, tr_image, bl_image, br_image;
        tl_image = cv::Point2f(m_left_center.x + m_left_rot_mat(0, 0) * tl_light.x + m_left_rot_mat(0, 1) * tl_light.y,
                               m_left_center.y + m_left_rot_mat(1, 0) * tl_light.x + m_left_rot_mat(1, 1) * tl_light.y);
        bl_image = cv::Point2f(m_left_center.x + m_left_rot_mat(0, 0) * bl_light.x + m_left_rot_mat(0, 1) * bl_light.y,
                               m_left_center.y + m_left_rot_mat(1, 0) * bl_light.x + m_left_rot_mat(1, 1) * bl_light.y);
        tr_image = cv::Point2f(m_right_center.x + m_right_rot_mat(0, 0) * tr_light.x + m_right_rot_mat(0, 1) * tr_light.y,
                               m_right_center.y + m_right_rot_mat(1, 0) * tr_light.x + m_right_rot_mat(1, 1) * tr_light.y);
        br_image = cv::Point2f(m_right_center.x + m_right_rot_mat(0, 0) * br_light.x + m_right_rot_mat(0, 1) * br_light.y,
                               m_right_center.y + m_right_rot_mat(1, 0) * br_light.x + m_right_rot_mat(1, 1) * br_light.y);

        m_final_vertices.push_back(bl_image);
        m_final_vertices.push_back(tl_image);
        m_final_vertices.push_back(tr_image);
        m_final_vertices.push_back(br_image);
    }

}
