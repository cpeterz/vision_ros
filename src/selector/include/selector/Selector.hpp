#pragma once
#include "armor/Armor.hpp"
#include "port/WMJProtocol.h"

namespace wmj
{
    enum SelectorState
    {
        SELECTOR_SEARCHING = 0,  // 首次选择
        SELECTOR_TRACKING = 1,   // 跟踪目标
    }; // 筛选器状态

    enum DeepROISizeState
    {
        ROI_BIG = 0,
        ROI_SMALL = 1,
    };

    enum SelectorMode
    {
        INFANTRY    = 0, // 步兵
        HERO        = 1, // 英雄
        SENTRY      = 2, // 哨兵
    }; // 筛选器模式

    // 筛选器参数类
    class SelectorParam
    {
    public:
        SelectorParam();
        SelectorParam(SelectorMode);
        ~SelectorParam();
        void setParam(SelectorMode);                    // 初始化参数

        std::vector<std::vector<int>> m_armor_priority; // 装甲板优先级序列
        cv::Rect2d m_camera_resolution;                 // 相机分辨率
        cv::Rect2d m_deep_default_roi;                  // 深度识别专用 中央roi
        cv::Size2i m_deep_roi_size;                     // 深度识别专用 roisize
        double m_aim_point_dis;                         // 左右目准心距离

        double m_max_second_width_diff;                 // 次要目标与主要目标最大距离差
        double m_min_second_width_diff;                 // 次要目标与主要目标最小距离差
        double m_max_second_height_diff;                // 次要目标与主要目标最大高度差

        double m_max_compare_ratio_diff;                // 两装甲板相似最大面积比差
        double m_max_compare_width_diff;                // 两装甲板相似最大宽度差
        double m_max_compare_height_diff;               // 两装甲板相似最大高度差
        double m_max_compare_center_diff;               // 两装甲板相似最大视差比例差
        double m_max_compare_tiltangle_diff;            // 两装甲板相似最大倾斜角差
        double m_max_compare_center_distance;           // 两装甲板相同最大中心距离差
        double m_max_compare_distance_diff;             // 两装甲板相似最大测距差

        double m_max_mono_distance;                     // 单目识别距离限制
        double m_max_track_yaw_angle;                   // 最大追踪yaw angle 
        double m_min_switch_yaw_angle;                  // 最小同车切换yaw angle

        std::vector<double> m_roi_mul;                  // 哨兵用roi距离阈值对应倍率
        std::vector<double> m_roi_dis;                  // 哨兵用roi距离阶段阈值
        std::vector<double> m_roi_lost_mul;             // 丢弃时roi扩大倍率

        float m_roi_up_ratio;                           // roi识别区域向上扩展原高度倍数
        float m_roi_down_ratio;                         // roi识别区域向下扩展原高度倍数
        float m_roi_left_ratio;                         // roi识别区域向左扩展原高度倍数
        float m_roi_right_ratio;                        // roi识别区域向右扩展原高度倍数

        float m_sentry_roi_up_ratio;                    // 哨兵模式roi识别区域向上扩展原高度倍数
        float m_sentry_roi_down_ratio;                  // 哨兵模式roi识别区域向下扩展原高度倍数
        float m_sentry_roi_left_ratio;                  // 哨兵模式roi识别区域向左扩展原高度倍数
        float m_sentry_roi_right_ratio;                 // 哨兵模式roi识别区域向右扩展原高度倍数

        double m_max_searching_center_diff;             // 搜索准心优先距离
        
        int m_max_track_lost;                           // 跟踪最大丢弃数
        int m_max_center_searching;                     // 操作手最大信任次数
        int m_max_id_search2track;                      // 确认id中间状态帧数

    };


    class Selector
    {
    public:
        Selector();
        // Selector(cv::Size2i deep_roi_size);
        ~Selector();
        bool SelectorMain(std::vector<Armor> armors,wmj::ROBO_STATE mode);      // 筛选主函数，需传入装甲板识别结果
        bool SelectorSingle(std::vector<Armor> armors,wmj::ROBO_STATE mode);    // 单目筛选主函数

        void TrackInit();                                                       // 初始化跟踪条件（id）

        // 结果回传
        cv::Rect2d getLeftROI();                                                // 回传左目ROI区域    
        cv::Rect2d getRightROI();                                               // 回传右目ROI区域    
        Armor getBestArmor();                                                   // 回传首要目标
        Armor getSecondArmor();                                                 // 回传次要目标
        bool getSwitchInSameId();                                               // 回传是否发生了同车切换
        void OutputResult(Armor a_armor);                                       // 输出识别结果
                                                 
    protected:

        // 筛选条件判断
        void PositionErrorHandling();                                           // position异常值（nan、inf）处理
        void SortByCenter();                                                    // 根据准心距离排序
        int GetArmorPriority(int id);                                           // 得到装甲板优先级
        double GetDistance(cv::Point2f a_center,cv::Point2f b_center);          // 得到中心距离差值
        double GetDistance(cv::Point3f);                                        // 解算绝对距离
        double Compare(Armor a, Armor b);                                       // 比较装甲板是否为相似,返回得分或-1
        bool SecondJudge(Armor best, Armor second);                             // 判断是否为次要目标
        bool MonoDisLimit(Armor a);                                             // 单目识别情况下距离限制（单目过远标记为未识别）
        void ChooseBestInSameId(Armor track, Armor second);                     // 同车选择最优装甲板

        // 筛选流程
        void updateArmorId(Armor a);                                            // 设置装甲id，用于避免id误识别
        void updateArmorId(Armor a, Armor b);                                   // 设置装甲id，用于避免id误识别
        void updateFinalArmor();                                                // 更新筛选结果(结果为空)
        void updateFinalArmor(Armor bestarmor);                                 // 更新筛选结果(有主要目标无次要目标)
        void updateFinalArmor(Armor bestarmor,Armor secondarmor);               // 更新筛选结果(主要次要目标均存在)
        void ConfirmTrackId(Armor trackamor);                                   // 确认跟踪id
        bool SelectorNormal();                                                  // 常规模式（线性预测&反陀螺）
        bool SelectorSearching();                                               // Searching状态筛选
        bool SelectorTracking();                                                // Tracking状态筛选
        bool SelectorSentry();                                                  // 反哨兵模式

        // ROI 
        void SetROI();                                                          // 得到全车roi区域
        bool SetDeepRoiSize(cv::Size2i deep_roi_size);

    protected:
        bool double_use = true;                         // 单目识别 or 双目识别
        std::shared_ptr<wmj::SelectorParam> m_param;    // 筛选器参数
        wmj::ROBO_STATE m_mode;                         // 筛选器模式
        SelectorState m_state;                          // 筛选器状态
        std::vector<Armor> m_armors;                    // 装甲板识别结果
        Armor m_bestarmor;                              // 主要目标
        Armor m_secondarmor;                            // 次要目标
        Armor m_prebestarmor;                           // 上次识别筛选到的主要目标
        Armor m_presecondarmor;                         // 上次识别筛选到的次要目标
        cv::Rect2d m_return_roi_left;                   // 回传下一帧左目识别所需的roi
        cv::Rect2d m_return_roi_right;                  // 回传下一帧右目识别所需的roi
        int m_tracklost;                                // 追踪遗失目标次数
        int m_search_center_lost;                       // 操作手准心置信度
        
        DeepROISizeState m_deep_roi_state;

        int m_track_id = -1;                            // 追踪id
        int m_id_search2track;                          // 确认id
        bool m_same_id_switch;                          // 同id切换装甲板
    };

    class InfantrySelector : public Selector
    {
    public:
        InfantrySelector();
        InfantrySelector(cv::Size2i deep_roi_size);
        ~InfantrySelector();
    };

    class HeroSelector : public Selector
    {
    public:
        HeroSelector();
        HeroSelector(cv::Size2i deep_roi_size);
        ~HeroSelector();
    };

    class SentrySelector : public Selector
    {
    public:
        SentrySelector();
        ~SentrySelector();

        // 哨兵重新SetROI
        void SetROI();
    private:
        double m_roi_value;                             // 哨兵使用roi距离倍率
    };
    
}   