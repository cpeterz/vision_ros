#pragma once
#include "libbase/common.h"
namespace wmj
{

  /*------------------------------------------------------------------灯条----------------------------------------------------------------*/

  // 灯条定义
  class Light
  {
  public:
    Light(){};
    Light(const std::vector<cv::Point> contour,
          const cv::Point2d &base);          // 带参构造函数
    void regularRect(cv::RotatedRect &rect); // 外接矩形矫正

  public:
    cv::RotatedRect m_rect;           // 灯条外接矩形
    cv::Point2f m_center;             // 灯条中心
    cv::Rect2d m_rectR;               // 灯条正接矩形
    _COLOR m_color;                   // 灯条颜色
    double m_ratio;                   // 长宽比
    double m_length;                  // 灯条长度
    double m_width;                   // 灯条宽度
    double m_area;                    // 灯条面积
    double m_area_ratio;              // 灯条轮廓面积和最小外接矩形面积比
    double m_angle;                   // 灯条角度
    std::vector<cv::Point> m_contour; // 灯条轮廓点集
  };

  // 灯条几何筛选参数
  class LightParam
  {
  public:
    LightParam();
    void setParam(const cv::FileStorage &fs);
    bool isLight(Light light); // 灯条几何条件筛选
    bool judgeColor(Light &light, const std::vector<cv::Point> Contours,
                    const cv::Mat &src); // 颜色判断

  public:
    float m_light_max_area;   // 灯条最大面积
    float m_light_min_area;   // 灯条最小面积
    float m_light_min_ratio;  // 灯条最小长宽比
    float m_light_max_ratio;  // 灯条最大长宽比
    float m_light_min_angle;  // 灯条最小角度
    float m_light_max_angle;  // 灯条最大角度
    float m_light_area_ratio; // 灯条轮廓面积和最小外接矩形面积比

    /* #################通用参数########################## */
    float m_min_RBdiff; // 红蓝最小差值
    float m_min_BRdiff; // 蓝红最小差值
    int m_enemy_color;  // 敌方颜色，0是红色，1是蓝色
  };

  typedef std::vector<Light> Lights;

  /*------------------------------------------------------------------灯条----------------------------------------------------------------*/

  /*-----------------------------------------------------------------装甲板----------------------------------------------------------------*/

  // 装甲板定义
  class Armor
  {
  public:
    Armor() { m_pairs.resize(2); };
    Armor(const Light &left, const Light &right,
          cv::Point2f targetPoint, cv::Rect2d bounds); // 带参构造
    bool operator>(const Armor &armor) const;          // 装甲板排序

  public:
    Lights m_pairs;                      // 灯条对
    cv::Rect2d m_rect;                   // 装甲板外接矩形
    cv::RotatedRect m_lightRect;         // 灯条构成的矩形
    wmj::_COLOR m_color;                 // 装甲板颜色
    std::vector<cv::Point2f> m_vertices; // 单目测距用角点

    cv::Point3f m_position;           // 三维坐标信息
    cv::Point3f m_armor_abs_position; // 绝对坐标信息
    cv::Point3f m_angle;              // 三维角度坐标，等待运算
    cv::Point2f m_center;             // 中心位置，为了双目识别的
    double m_time_seq;                // 时间戳

    // 装甲板参数
    float m_ratio;      // 装甲长宽比
    double m_yaw_angle; // 按yaw轴旋转的角度
    float m_area;       // 装甲面积
    float m_width;      // 装甲横向长度
    float m_height;     // 装甲纵向长度
    float m_tiltAngle;  // 装甲roll旋转角度
    float m_lighsRatio; // 装甲板两个灯条的长度比
    float m_angle_diff; // 装甲板两个灯条的角度差

    int m_id = -1;  // 装甲id
    double m_score; // 装甲板优先度

    wmj::ARMORTYPE m_armor_type = ARMOR_NONE;  // 大小装甲，默认ARMOR_NONE
    wmj::DETECTEDTYPE m_detectedtype;          // 标记是双目识别到的还是单目识别到的
    wmj::FINDTYPE m_bestArmorStatus = NOTFIND; // 判断识别结果，默认NOTFIND
  };

  class key_Armor
  {
  public:
    key_Armor();

  public:
    cv::Point3f m_position;                   // 三维坐标信息
    double m_yaw_angle;                       // 按yaw轴旋转的角度
    double m_time_seq;                        // 时间戳
    wmj::ARMORTYPE m_armor_type = ARMOR_NONE; // 大小装甲，默认ARMOR_NONE
    wmj::_COLOR m_color;                      // 装甲板颜色
    int m_id = -1;                            // 装甲id
  };
  
  // 装甲板几何筛选参数
  class ArmorParam
  {
  public:
    ArmorParam();
    void setParam(const cv::FileStorage &fs);
    bool isArmor(Armor armor); // 灯条几何条件筛选

  public:
    // float m_armor_angle_diff;     //*暂时没用
    float m_armor_tiltAngle;      // 装甲roll旋转角度
    float m_armor_width_min;      // 装甲板宽度最小值（防止误识别符合比例的非装甲板轮廓）
    float m_armor_ratio_max;      // 装甲板最大长宽比
    float m_armor_ratio_min;      // 装甲板最小长宽比
    float m_lights_diff;          // 装甲板两个灯条长度比
    float m_armor_numberArea_min; // 数字识别区域的最小面积
    float m_angle_diff;           // 装甲板两个灯条的角度差
  };

  // 装甲板对
  struct ArmorPair
  {
    Armor left;  // 左相机看到的
    Armor right; // 右相机看到的
                 //  double time; //*时间戳暂时没用
  };

  typedef std::vector<Armor> Armors;
  typedef std::vector<key_Armor> key_Armors;

  /*-----------------------------------------------------------------装甲板----------------------------------------------------------------*/

  // test
  // MatWithTime a;
} // namespace wmj