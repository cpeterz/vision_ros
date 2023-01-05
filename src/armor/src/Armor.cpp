#include "armor/Armor.hpp"

using namespace std;
using namespace cv;
using namespace cv::ml;

namespace wmj
{

    /*-----------------------旋转矩形规范化---------------------------*/
    //?没有用上
    // 旋转矩形规范化 使得width为长边，角度(-180,0]
    void regularLight(Light &light)
    {

        if (light.m_rect.size.width < light.m_rect.size.height)
        {

            swap(light.m_rect.size.width, light.m_rect.size.height);
            light.m_rect.angle -= 90.0f;
        }
    }
    /*-----------------------旋转矩形规范化---------------------------*/

    /*-----------------------LightParam构造函数---------------------------*/


    // 构造函数
    LightParam::LightParam()
    {

        cv::FileStorage light_param(ARMOR_CFG, cv::FileStorage::READ);
        setParam(light_param);
    }

    /*----------------------------------------------*/
    void LightParam::setParam(const FileStorage &fs)
    {
        bool usenumber;
        fs["usenumber"] >> usenumber;
        if (usenumber)
        {
            fs["ArmorDetector"]["light_max_area"] >> m_light_max_area;
            fs["ArmorDetector"]["light_min_area"] >> m_light_min_area;
            fs["ArmorDetector"]["light_min_ratio"] >> m_light_min_ratio;
            fs["ArmorDetector"]["light_max_ratio"] >> m_light_max_ratio;
            fs["ArmorDetector"]["light_min_angle"] >> m_light_min_angle;
            fs["ArmorDetector"]["light_max_angle"] >> m_light_max_angle;
            fs["ArmorDetector"]["light_area_ratio"] >> m_light_area_ratio;
        }
        else
        {
            fs["dimDetect"]["light_max_area"] >> m_light_max_area;
            fs["dimDetect"]["light_min_area"] >> m_light_min_area;
            fs["dimDetect"]["light_min_ratio"] >> m_light_min_ratio;
            fs["dimDetect"]["light_max_ratio"] >> m_light_max_ratio;
            fs["dimDetect"]["light_min_angle"] >> m_light_min_angle;
            fs["dimDetect"]["light_max_angle"] >> m_light_max_angle;
            fs["dimDetect"]["light_area_ratio"] >> m_light_area_ratio;
        }

        fs["ArmorDetector"]["RBdiff"] >> m_min_RBdiff;
        fs["ArmorDetector"]["BRdiff"] >> m_min_BRdiff;
        fs["ArmorDetector"]["enemy_color"] >> m_enemy_color;
    }
    /*----------------------------------------------*/

    /*-----------------------LightParam构造函数---------------------------*/

    /*-----------------------判断灯条---------------------------*/

    // 判断灯条
    bool LightParam::isLight(Light light)
    {
        //  if(!(m_width > 2 && m_length > 4.5))
        //  {
        //      cout<<"width:" << m_width << endl;
        //      cout<<"length:" << m_length << endl;
        //  }
        //  else if(!(m_area < _param.m_light_max_area && m_area >
        //  _param.m_light_min_area  ))
        // {
        //     cout<<"area:"<<m_area<<endl;
        // }
        //  else if(!(m_ratio < _param.m_light_max_ratio && m_ratio >
        //  _param.m_light_min_ratio))
        //  {
        //      cout<<"ratio:"<<m_ratio<<endl;
        //  }
        //  else if(!(m_area_ratio > _param.m_light_area_ratio))
        //  {
        //       cout<<"arearatio:"<<m_area_ratio<<endl;
        //  }
        //  else
        //  {
        //      cout <<"IsLight"<<endl;
        //  }

        return light.m_width > 2 && light.m_length > 4.5 &&
               light.m_ratio < m_light_max_ratio &&
               light.m_area < m_light_max_area &&
               light.m_area_ratio > m_light_area_ratio &&
               light.m_area > m_light_min_area && light.m_ratio > m_light_min_ratio;
    }

    /*-----------------------判断灯条---------------------------*/

    /*-----------------------判断颜色---------------------------*/

    // 判断颜色
    bool LightParam::judgeColor(Light &light, const vector<Point> Contours,
                                const Mat &src)
    {

        Vec3b pixel;
        int red = 0, blue = 0;

        for (int j = 0; j < Contours.size(); j++)
        {

            pixel = src.at<Vec3b>(Point(Contours[j]));
            red += pixel[0];
            blue += pixel[2];
        }

        int RB_diff = (red - blue) / int(Contours.size());

        if (RB_diff > m_min_RBdiff)
        {

            light.m_color = _RED;
        }
        else if (RB_diff < (-1 * m_min_BRdiff))
        {

            light.m_color = _BLUE;
        }
        else
        {

            light.m_color = _WHITE;
        }

        if (light.m_color == (_COLOR)m_enemy_color)
        {

            return true;
        }
        else
        {

            return false;
        }
    }

    /*-----------------------判断颜色---------------------------*/

    /*#########################################
                Light类函数
    ########################################*/
    /*-----------------------Light构造函数---------------------------*/

    // 构造函数
    Light::Light(const vector<Point> contour, const Point2d &base)
    {
        m_rect = fitEllipse(contour);
        regularRect(m_rect);
        m_rectR = boundingRect(contour);
        m_center = m_rect.center;
        m_length = m_rect.size.width;
        m_width = m_rect.size.height;
        m_area = m_rectR.area();
        m_ratio = m_length / m_width;
        m_contour = contour;
        m_area_ratio = contourArea(contour) / (m_width * m_length);
    }

    /*---------------------------------------*/
    void Light::regularRect(cv::RotatedRect &rect)
    {
        if (rect.size.height > rect.size.width)
        {
            std::swap<float>(rect.size.height, rect.size.width);
            rect.angle =
                rect.angle >= 0.0f ? rect.angle - 90.0f : rect.angle + 90.0f;
        }
        if (rect.angle < 0)
        {
            rect.angle += 180.0f;
        }
    }
    /*---------------------------------------*/

    /*-----------------------Light构造函数---------------------------*/

    /*#################################
                ArmorParam类函数
    #################################*/
    /*-----------------------ArmorParam构造函数---------------------------*/

    // 构造函数
    ArmorParam::ArmorParam()
    {
        cv::FileStorage armor_param(ARMOR_CFG, cv::FileStorage::READ);
        setParam(armor_param);
    }

    /*-------------------------------------------*/
    void ArmorParam::setParam(const FileStorage &fs)
    {
        bool usenumber;
        fs["usenumber"] >> usenumber;
        if (usenumber)
        {
            fs["ArmorDetector"]["armor_tiltAngle"] >> ArmorParam::m_armor_tiltAngle;
            fs["ArmorDetector"]["armor_width_min"] >> ArmorParam::m_armor_width_min;
            fs["ArmorDetector"]["armor_ratio_max"] >> ArmorParam::m_armor_ratio_max;
            fs["ArmorDetector"]["armor_ratio_min"] >> ArmorParam::m_armor_ratio_min;
            fs["ArmorDetector"]["lights_diff_max"] >> ArmorParam::m_lights_diff;
            fs["ArmorDetector"]["armor_numberArea_min"] >> ArmorParam::m_armor_numberArea_min;
            fs["ArmorDetector"]["angle_diff"] >> ArmorParam::m_angle_diff;
        }
        else
        {
            fs["dimDetect"]["armor_tiltAngle"] >> ArmorParam::m_armor_tiltAngle;
            fs["dimDetect"]["armor_width_min"] >> ArmorParam::m_armor_width_min;
            fs["dimDetect"]["armor_ratio_max"] >> ArmorParam::m_armor_ratio_max;
            fs["dimDetect"]["armor_ratio_min"] >> ArmorParam::m_armor_ratio_min;
            fs["dimDetect"]["lights_diff_max"] >> ArmorParam::m_lights_diff;
            fs["dimDetect"]["armor_numberArea_min"] >> ArmorParam::m_armor_numberArea_min;
            fs["dimDetect"]["angle_diff"] >> ArmorParam::m_angle_diff;
        }
    }
    /*-------------------------------------------*/

    /*-----------------------ArmorParam构造函数---------------------------*/

    /*-----------------------判断是否为装甲板---------------------------*/

    // 装甲不超出范围且旋转不能超过一定角度
    bool ArmorParam::isArmor(Armor armor)
    {

        /*########################################
        用于装甲板几何筛选debug，如果灯条能框到，但被筛掉，则可以解注释这里查看
        #########################################*/

        //   if(!(m_tiltAngle < _param.m_armor_tiltAngle))
        //   {
        //       cout<<"m_tilAngle:"<<m_tiltAngle<<endl;
        //   }
        //   else if(!(m_ratio < _param.m_armor_ratio_max&&_param.m_armor_ratio_min
        //   < m_ratio))
        //   {
        //       cout<<"ratio"<<m_ratio<<endl;
        //   }
        //   else if(!(m_lighsRatio < _param.m_lights_diff))
        //   {
        //       cout<<"light_diff"<<m_lighsRatio<<endl;
        //   }
        //   else if(!(m_width > 7))
        //   {
        //       cout<<"width:"<<m_width<<endl;
        //   }
        //   else if(!(m_height > 5))
        //   {
        //       cout<<"height:"<<m_width<<endl;
        //   }
        //   else if( !(m_angle_diff < _param.m_angle_diff))
        //   {
        //       cout<<"angle_diff:"<<m_angle_diff<<endl;
        //   }
        //   else
        //   {
        //       cout<<"IsArmor"<<endl;
        //   }

        if (!(armor.m_tiltAngle < m_armor_tiltAngle &&
              armor.m_ratio < m_armor_ratio_max &&
              armor.m_ratio > m_armor_ratio_min &&
              armor.m_lighsRatio < m_lights_diff && armor.m_width > 7 &&
              armor.m_height > 5 && armor.m_angle_diff < m_angle_diff))
        {

            return 0;
        }

        return 1;
    }

    /*-----------------------判断是否为装甲板---------------------------*/

    /*#################################
                Armor类函数
    #################################*/
    /*----------------------------------------------------------------Armor构造函数-------------------------------------------------------------------*/

    /*--------------------------------------------*/
    template <typename _Tp> // 计算两点间的距离
    double getDistance(const Point_<_Tp> &p1, const Point_<_Tp> &p2)
    {

        const Point_<_Tp> tmp = p1 - p2;

        return sqrt(tmp.ddot(tmp));
    }
    /*---------------------------------------------*/

    Armor::Armor(const Light &left, const Light &right, Point2f targetPoint, Rect2d bounds)
    {

        m_vertices.resize(4);

        // 装甲板的两根灯条
        m_pairs.push_back(left);
        m_pairs.push_back(right);

        // 求装甲板ROI，用于下一帧ROI识别
        Point2d leftCenter = left.m_center;
        Point2d rightCenter = right.m_center;

        double Armor_lightRatio = 2.0; // 假定的灯条和装甲板长度的比值

        Point2d LTpoint, RDpoint;
        LTpoint.x = leftCenter.x - left.m_width / 2;
        LTpoint.y = leftCenter.y - left.m_length * Armor_lightRatio / 2;
        RDpoint.x = rightCenter.x + right.m_width / 2;
        RDpoint.y = rightCenter.y + right.m_length * Armor_lightRatio / 2;

        // 得到装甲板框选区域
        m_rect = Rect(LTpoint, RDpoint);
        m_rect &= bounds;

        // 计算装甲板中心
        m_center = Point2f((leftCenter.x + rightCenter.x) / 2,
                           (leftCenter.y + rightCenter.y) / 2);
        m_width = getDistance(left.m_center, right.m_center); // 横向长度
        m_height = (left.m_length + right.m_length) / 2;      // 纵向长度
        m_ratio = m_width / m_height;                         // 长宽比
        m_armor_type =
            m_ratio > 3 ? wmj::ARMOR_LARGE : wmj::ARMOR_SMALL; // 装甲板分类
        m_tiltAngle = asin(abs(left.m_center.y - right.m_center.y) / m_width) *
                      180 / PI; // 倾斜角
        m_angle_diff =
            abs(m_pairs[0].m_angle - m_pairs[1].m_angle); // 装甲板两个灯条角度差
        m_lighsRatio = max(left.m_length, right.m_length) /
                       min(left.m_length, right.m_length); // 两个灯条长度的比值

        // 框出灯条画出的矩形
        Point2f left_points[4], right_points[4];
        m_pairs[0].m_rect.points(left_points);
        m_pairs[1].m_rect.points(right_points);
        m_vertices[0] = (left_points[0] + left_points[1]) / 2;
        m_vertices[1] = (left_points[3] + left_points[2]) / 2;
        m_vertices[2] = (right_points[2] + right_points[3]) / 2;
        m_vertices[3] = (right_points[1] + right_points[0]) / 2;
        m_lightRect = minAreaRect(m_vertices);

        /*###############################
        根据装甲板到屏幕中心的距离和几何形状规范程度计算装甲板得分
        ################################*/
        m_score = exp(-getDistance(m_center, targetPoint) / 100) + m_tiltAngle / 5;
        m_color = (left.m_color == right.m_color) ? left.m_color : _WHITE;
    }

    /*-----------------------------------------------Armor构造函数-----------------------------------------------*/

    // 装甲优先度,找到距离镜头中心最近的装甲板
    bool Armor::operator>(const Armor &armor) const
    {
        if (m_score > armor.m_score)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    /*-----------------------------------------------key_Armor构造函数-----------------------------------------------*/
    //初始化key_Armor
    key_Armor::key_Armor()
    {
        cv::Point3f m_position; // 三维坐标信息
        m_position.x = 0;
        m_position.y = 0;
        m_position.z = 0;
        double m_yaw_angle = 0;                   // 按yaw轴旋转的角度
        double m_time_seq = 0;                    // 时间戳
        wmj::ARMORTYPE m_armor_type = ARMOR_NONE; // 大小装甲，默认ARMOR_NONE
        wmj::_COLOR m_color = _RED;               // 装甲板颜色
        int m_id = -1;                            // 装甲id
    }

} // namespace wmj