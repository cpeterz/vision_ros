/***************************************
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@@!!!@@@@@@@@@@@@!!!!@@@@@@@@@@@%!!!@@@@@@@@@@@!!!!!!!!!!@@
@@!!!!!@@@@@@@@@%%!!!!!@@@@@@@@@%%!!!!@@@@@@@@@!!!!!!!!!!@@
@@@!!!!!@@@@@@@%%%%!!!!!@@@@@@@%%%!!!!!@@@@@@@@@@@@@!!!!!@@
@@@@!!!!!@@@@@@%%%@@!!!!!@@@@@%%%%@!!!!!@@@@@@@@@@@@!!!!!@@
@@@@@!!!!!@@@@%%%@@@@!!!!@@@@%%%%@@@!!!!!@@@@@@@@@@@!!!!!@@
@@@@@@!!!!!@@%%%%@@@@!!!!!@@@%%%@@@@@!!!!!@@@@@@@@@@!!!!!@@
@@@@@@!!!!!@%%%%@@@@@@!!!!!@%%%$@@@@@@!!!!@@@@@@@@@@!!!!!@@
@@@@@@@!!!!!%%%@@@@@@@@!!!!!%%%@@@@@@@@!!!!@@@@@@@@!!!!!!@@
@@@@@@@@!!!!!%@@@@@@@@@@!!!!!%@@@@@@@@@!!!!!!!!!!!!!!!!!!@@
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

WMJ战队深度学习装甲板目标检测类
依赖库：             OpenVINO_2022版本
目前支持使用的模型：   yolov5改四点模型；yolox改四点模型
时间：               2022.07.03
***************************************/
#pragma once
#define USE_DEEP
#ifdef USE_DEEP
#include "armor/Armor.hpp"
#include <openvino/openvino.hpp>
#include "lightoptimizer/LightOptimizer.hpp"
#include "binocular/Binocular.h"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/opencv.hpp"
#include "libbase/common.h"



namespace wmj
{
    struct bbox_t {
    cv::Point2f pts[4];    // [pt0, pt1, pt2, pt3] 四点组，顺序为：左上↖，左下↙，右下↘，右上↗
    float conf;            // 综合置信度
    cv::Rect_<float> rect; // 外接轮廓
    float color_conf;      // 颜色置信度 
    int color_id; // 0: red, 1: blue, 2: gray
    int tag_id;
    };

    struct GridAndStride
    {
        int grid0;
        int grid1;
        int stride;
    };

    class DeepDetector
    {
        /*********************深度学习检测器主类*********************/
        public:
        DeepDetector();
        DeepDetector(const std::string xml, const std::string bin);                // 带参构造指定模型文件
                                                                                                                                                  
        void init(std::string xml = "", std::string bin = "");                     // 初始化
        bool DeepDetectSingle(wmj::MatWithTime& frame, cv::Rect2d& roi);           // 模型推理检测主程序
        void setParam(cv::FileStorage fs);                                         // 设置参数
        void DebugOutput();                                                        // Debug输出
        cv::Size2i getInputSize(); 
        private:                                                                                                                                                             
        bool yolov5_model_infer();                              // YOLOv5模型推理
        bool yolox_model_infer();                               // YOLOX模型推理
        void input_preprocess(cv::Mat& frame, cv::Rect2d& roi); // 输入图像处理
        void bbox2Armor();                                      // 输出bounding_boxs转化Armor类
                                                                                                                                         
        public:                                         
        int Use_Deep;                       // 是否使用深度检测标志位
        std::string m_model_format = "xml"; // 模型所使用格式 OpenVINO转换IR格式(xml,bin) or ONNX格式
        std::string m_xml;                  // 模型文件名称(不带文件类型后缀)
        std::string m_bin;
        std::string m_onnx;
        std::string m_model_type = "v5";    // 模型类型，目前为 “v5” or “x”
        std::string DEVICE = "CPU";         // 推理设备
                                                                       
        cv::Mat m_src; 
        cv::Rect2d m_roi;
        cv::Mat m_gray;      // 灯条优化所需灰度图
        Armors m_armors;     // 检测出装甲板组
        key_Armors m_key_armors;    //需要传输给控制的装甲板信息
        // Armor m_final_armor; // 经过筛选器的最优击打装甲
        int m_enemy_color;   // 敌方颜色
        int ncolor = 3;      // 当前模型颜色数量
        int ntag = 9;        // 当前模型类别数量

        // wmj::ArmorSelector m_selector; // 筛选器实例

        private:
        int input_height;              // 模型输入图像高度
        int input_width;               // 模型输入图像宽度
        int maxProposalCount;          // 模型输出通道数
        int objectLength;              // 模型输出
        int numberOfClass;             // 模型输出
        float m_resize_scale;          // resize缩放尺寸
        std::map<int, int> m_label2id; // 模型输出类别序号与原类别对应表
        float m_nmsThreshold;          // nms阈值
        float m_confThreshold;         // 置信度阈值

        float m_Lightratio = 4;        // 生成灯条比例常数

        //YOLOX输出解析
        std::vector<bbox_t> proposals;
        std::vector<int> strides = {16, 32};
        std::vector<GridAndStride> grid_strides;

        void decode_YOLOX_outputs(const float* prob, std::vector<bbox_t>& objects, const int img_w, const int img_h);                                      // YOLOX输出解析主函数
        void generate_grids_and_stride(const int target_w, const int target_h, std::vector<int>& strides, std::vector<GridAndStride>& grid_strides);       // 初始化grids_and_stride
        void generate_yolox_proposals(std::vector<GridAndStride> grid_strides, const float* feat_ptr, float prob_threshold, std::vector<bbox_t>& objects); // 生成预测
        void nms_sorted_bboxes(const std::vector<bbox_t>& faceobjects, std::vector<int>& picked, float nms_threshold);                                     // 非极大值抑制

        //OpenVINO implementation
        ov::Core core;                        // 
        std::string m_input_precision;        // 输入精度
        std::shared_ptr<ov::Model>  m_model;  // ov模型
        ov::CompiledModel m_compiled_model;   // 编译后模型
        ov::InferRequest m_infer_request;     // 推理请求

        cv::Mat input_mat;                    // 图像输入
        ov::Tensor input_T;                   // 模型输入张量
        ov::Tensor output_T;                  // 模型输出张量
        std::vector<bbox_t> m_bboxes;         // 模型预测bboxes

        // Lights m_lights;
        wmj::Monocular m_monocular; //单目测距
        wmj::LightOptimizer m_light_optimizer; //灯条优化（未使用）
        

    };
}
#endif