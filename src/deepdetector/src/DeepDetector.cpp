#include "deepdetector/DeepDetector.hpp"

#define USE_LIGHT_OPTIMIZER 0


// Tools Functions
ov::Tensor wrapMat2Tensor(const cv::Mat& mat) 
{
const size_t channels = mat.channels();
const size_t height = mat.size().height;
const size_t width = mat.size().width;

const size_t strideH = mat.step.buf[0];
const size_t strideW = mat.step.buf[1];

const bool is_dense = strideW == channels && strideH == channels * width;
OPENVINO_ASSERT(is_dense, "Doesn't support conversion from not dense cv::Mat");

return ov::Tensor(ov::element::u8, ov::Shape{1, height, width, channels}, mat.data);
}

static inline float sigmoid(float x) 
{
    return 1 / (1 + std::exp(-x));
}

static float maxclass(const float *ptr, int nc, int &max_class) 
{
    float max = 0;
    max_class = -1;
    for(int i = 0; i < nc; i++) 
    {
        if(ptr[i] > max)
        {
            max_class = i;
            max = ptr[i];
        } 
    }
    return max;
}



template<class F, class T, class ...Ts>
T reduce(F &&func, T x, Ts ...xs) {
    if constexpr (sizeof...(Ts) > 0) {
        return func(x, reduce(std::forward<F>(func), xs...));
    } else {
        return x;
    }
}

template<class T, class ...Ts>
T reduce_min(T x, Ts ...xs) {
    return reduce([](auto a, auto b) { return std::min(a, b); }, x, xs...);
}

template<class T, class ...Ts>
T reduce_max(T x, Ts ...xs) {
    return reduce([](auto a, auto b) { return std::max(a, b); }, x, xs...);
}

static inline bool is_overlap(const cv::Point2f pts1[4], const cv::Point2f pts2[4]) {
    cv::Rect2f box1, box2;
    box1.x = reduce_min(pts1[0].x, pts1[1].x, pts1[2].x, pts1[3].x);
    box1.y = reduce_min(pts1[0].y, pts1[1].y, pts1[2].y, pts1[3].y);
    box1.width = reduce_max(pts1[0].x, pts1[1].x, pts1[2].x, pts1[3].x) - box1.x;
    box1.height = reduce_max(pts1[0].y, pts1[1].y, pts1[2].y, pts1[3].y) - box1.y;
    box2.x = reduce_min(pts2[0].x, pts2[1].x, pts2[2].x, pts2[3].x);
    box2.y = reduce_min(pts2[0].y, pts2[1].y, pts2[2].y, pts2[3].y);
    box2.width = reduce_max(pts2[0].x, pts2[1].x, pts2[2].x, pts2[3].x) - box2.x;
    box2.height = reduce_max(pts2[0].y, pts2[1].y, pts2[2].y, pts2[3].y) - box2.y;
    return (box1 & box2).area() > 0;
}

static void qsort_descent_inplace(std::vector<wmj::bbox_t>& faceobjects, int left, int right)
{
    int i = left;
    int j = right;
    float p = faceobjects[(left + right) / 2].conf;

    while (i <= j)
    {
        while (faceobjects[i].conf > p)
            i++;

        while (faceobjects[j].conf < p)
            j--;

        if (i <= j)
        {
            // swap
            std::swap(faceobjects[i], faceobjects[j]);

            i++;
            j--;
        }
    }

    #pragma omp parallel sections
    {
        #pragma omp section
        {
            if (left < j) qsort_descent_inplace(faceobjects, left, j);
        }
        #pragma omp section
        {
            if (i < right) qsort_descent_inplace(faceobjects, i, right);
        }
    }
}


static void qsort_descent_inplace(std::vector<wmj::bbox_t>& objects)
{
    if (objects.empty())
        return;

    qsort_descent_inplace(objects, 0, objects.size() - 1);
}

static inline float intersection_area(const wmj::bbox_t& a, const wmj::bbox_t& b)
{
    cv::Rect_<float> inter = a.rect & b.rect;
    return inter.area();
}

namespace wmj
{
    DeepDetector::DeepDetector()
    {
        init();
    }

    DeepDetector::DeepDetector(const std::string xml, const std::string bin)
    {
        init(xml,bin);
    }

    void DeepDetector::setParam(cv::FileStorage fs)
    {

        fs["DeepDetector"]["USE_DEEP"] >> Use_Deep;
        fs["DeepDetector"]["DEVICE"] >> DEVICE;
        fs["DeepDetector"]["Lightratio"] >> m_Lightratio;
        fs["DeepDetector"]["nms_threshold"] >> m_nmsThreshold;
        fs["DeepDetector"]["conf_threshold"] >> m_confThreshold;
        std::string model_file;
        std::string input_precision;
        fs["DeepDetector"]["model_name"] >> model_file;
        fs["DeepDetector"]["input_precision"] >> m_input_precision;
        fs["DeepDetector"]["model_type"] >> m_model_type;
        fs["DeepDetector"]["ncolor"] >> ncolor;
        fs["DeepDetector"]["ntag"] >> ntag;
        fs["DeepDetector"]["model_format"] >> m_model_format;
        fs.release();
        cv::FileStorage file_param(ARMOR_CFG,cv::FileStorage::READ);
        file_param["ArmorDetector"]["enemy_color"] >> m_enemy_color;
        file_param.release();
        if(m_model_format == "xml")
        {
            m_xml = MODEL_DIR + model_file + ".xml";
            m_bin = MODEL_DIR + model_file + ".bin";
        }
        else
        {
            m_onnx = MODEL_DIR + model_file + ".onnx";
        }

    }
    
    void DeepDetector::init(std::string xml, std::string bin)
    {
        
        cv::FileStorage file_param(DEEP_CFG,cv::FileStorage::READ);
        setParam(file_param);
        if(m_model_format == "xml")
        {
            if(xml != "")
            {
                m_model = core.read_model(MODEL_DIR + xml, MODEL_DIR + bin);
            }
            else
            {
                m_model = core.read_model(m_xml,m_bin);
            }
        }
        else
        {
            m_model = core.read_model(m_onnx);

        }
        
        // m_model = core.read_model(MODEL_DIR + std::string("scale_enhanced0403_320480.onnx"));

        ov::preprocess::PrePostProcessor ppp(m_model);
        ov::preprocess::InputInfo& input_info = ppp.input();
        ov::preprocess::OutputInfo& output_info = ppp.output();
        
        input_info.tensor()
                    .set_element_type(ov::element::u8)
                    .set_layout("NHWC")
                    .set_color_format(ov::preprocess::ColorFormat::BGR);
        input_info.model().set_layout("NCHW");
        if(m_model_type == "x")
        {
            input_info.preprocess()
            .convert_element_type(ov::element::f32);
        }
        else
        {
            if(m_input_precision == "u8")
            {
                input_info.preprocess()
                        .convert_color(ov::preprocess::ColorFormat::RGB);
            }
            else if(m_input_precision == "f32")
            {
                input_info.preprocess()
                .convert_element_type(ov::element::f32)
                .scale({255,255,255})
                .convert_color(ov::preprocess::ColorFormat::RGB);
            }
        }
        output_info.tensor().set_element_type(ov::element::f32);

        m_model = ppp.build();

        m_compiled_model = core.compile_model(m_model,DEVICE);
        m_infer_request = m_compiled_model.create_infer_request();

        input_T = m_infer_request.get_input_tensor();

        ov::Shape input_shape = m_model->input().get_shape();
        input_height = input_shape[1];
        input_width = input_shape[2];
        ov::Shape output_shape = m_model->output().get_shape();
        maxProposalCount = output_shape[1];
        objectLength = output_shape[2];
        input_mat = cv::Mat(input_height, input_width, CV_8UC3,cv::Scalar(105,105,105));

        m_label2id = {{0, 7}, {1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5}, {6, 11}, {7, 8}, {8, 8}};
        if(m_model_type == "x")
        {
            generate_grids_and_stride(input_width, input_height, strides, grid_strides);
        }
        return;

    }



    template <typename _Tp> //计算两点间的距离
    double getDistance(const cv::Point_<_Tp> &p1, const cv::Point_<_Tp> &p2) {

        const cv::Point_<_Tp> tmp = p1 - p2;

        return sqrt(tmp.ddot(tmp));
    }

    cv::Size2i DeepDetector::getInputSize()
    {
        return cv::Size2i(cv::Size2i(input_width, input_height));
    }

    bool DeepDetector::DeepDetectSingle(wmj::MatWithTime& frame,cv::Rect2d& roi)
    {
        m_src = frame.m_img;
        m_armors.clear();
        m_bboxes.clear();

        m_roi = roi;
        input_preprocess(frame.m_img,m_roi);
        bool infer_result = false;
        if(m_model_type == "v5")
        {
            infer_result = yolov5_model_infer();
        }
        else
        {
            infer_result = yolox_model_infer();
        }
        if(infer_result)
        {
            bbox2Armor();
            if(USE_LIGHT_OPTIMIZER)
            {
                for(int i = 0; i < m_armors.size(); i++)
                {   
                    m_light_optimizer.LightOptimization(m_armors[i].m_pairs[0].m_rect, m_armors[i].m_pairs[1].m_rect,
                                                    m_roi, m_gray); // TODO RISIZE ROI 
                    std::tie(m_armors[i].m_position, m_armors[i].m_yaw_angle) =
                            m_monocular.getPositionAngle(m_light_optimizer.m_final_vertices,
                                                        m_armors[i].m_armor_type);
                }
            }
            else
            {
                for(int i = 0; i < m_armors.size(); i++)
                {   
                    std::tie(m_armors[i].m_position, m_armors[i].m_yaw_angle) =
                            m_monocular.getPositionAngle(m_armors[i].m_vertices,
                                                        m_armors[i].m_armor_type);
                    std::cout << "deep_X:" << m_armors[i].m_position.x << std::endl;
                }
            }
        }
        
        bool m_true_detected = m_armors.size();
        // m_final_armor = m_selector.getBestArmor();
        // m_final_armor = m_armors[0];
        return m_true_detected;



        //0. preprocess
        //1. detect
        //2. lightOptimize (option)
        //3. measure distance
        //4. select
        //5. retrun best armor 
    }

    void DeepDetector::input_preprocess(cv::Mat& frame,cv::Rect2d& ROI)
    {
        if(ROI.height != input_height || ROI.width != input_width)
        {
            m_resize_scale = (input_height / ROI.height) > (input_width / ROI.width) ? (input_width / ROI.width) : (input_height / ROI.height);
        }
        else
            m_resize_scale = 1;
        cv::Mat roi_mat = frame(ROI);
        cv::resize(roi_mat, roi_mat, {(int)round(roi_mat.cols * m_resize_scale), (int)round(roi_mat.rows * m_resize_scale)});
        roi_mat.copyTo(input_mat(cv::Rect(0,0,roi_mat.cols,roi_mat.rows)));        
        cv::cvtColor(frame(ROI),m_gray,CV_BGR2GRAY);
        input_T = wrapMat2Tensor(input_mat);
        m_infer_request.set_input_tensor(input_T);
        return ;
    }

    bool DeepDetector::yolov5_model_infer()
    {
        
        m_infer_request.infer();

        output_T = m_infer_request.get_output_tensor();
        const float* detections = output_T.data<const float>();
        numberOfClass = objectLength - 13;
        int max_class;
        float conf_class = 0;
        std::vector<bbox_t> before_nms;
        for(int i = 0; i < maxProposalCount; i++)
        {
            float *result = (float *)detections + i * objectLength;
            if (result[4] < m_confThreshold || (conf_class = maxclass(result + 13, numberOfClass, max_class)) < m_confThreshold) continue;
            bbox_t box;
                for (int i = 0; i < 4; i++) 
                {
                    box.pts[i].x = (result[i * 2 + 5]) / m_resize_scale;
                    box.pts[i].y = (result[i * 2 + 6]) / m_resize_scale;
                }
           
            box.tag_id = max_class;
            if(box.tag_id < 5)
            {
                box.color_id = 1;
            }
            else if(box.tag_id < 10)
            {
                box.color_id = 0;
                box.tag_id = box.tag_id - 5;
            }
            else 
            {
                box.color_id = 2;
                box.tag_id = box.tag_id - 10;
            }
            box.conf = result[4] * conf_class;
            // box.cls_conf = conf_class;
            if (box.color_id != m_enemy_color)
                continue;
            before_nms.emplace_back(box);
        }
        std::sort(before_nms.begin(), before_nms.end(), [](bbox_t &b1, bbox_t &b2) {
                return b1.conf > b2.conf;
            });
        m_bboxes.reserve(before_nms.size());
        std::vector<bool> is_removed(before_nms.size());
        for (int i = 0; i < before_nms.size(); i++) {
            if (is_removed[i]) continue;
            m_bboxes.emplace_back(before_nms[i]);
            for (int j = i + 1; j < before_nms.size(); j++) {
                if (is_removed[j]) continue;
                if (is_overlap(before_nms[i].pts, before_nms[j].pts)) is_removed[j] = true;
            }
        }
        // std::cout << "boxsize " << m_bboxes.size() << std::endl;
        if(!m_bboxes.size())
        {
            return false;
        }
        return true;
    }

    bool DeepDetector::yolox_model_infer()
    {
        m_infer_request.infer();

        output_T = m_infer_request.get_output_tensor();
        const float* detections = output_T.data<const float>();
        // numberOfClass = objectLength - 13;
        decode_YOLOX_outputs(detections,m_bboxes,input_width,input_height);
        return !m_bboxes.empty();

    }

    void DeepDetector::bbox2Armor()
    {
        
        for(int i = 0; i < m_bboxes.size(); i++)
        {
            Armor armor;
            
            armor.m_color = wmj::_COLOR(m_bboxes[i].color_id);
            
            armor.m_vertices.reserve(4);
            
            for(int j = 0; j < 4; j++)
            {
        
                armor.m_vertices.emplace_back((m_bboxes[i].pts[j] + cv::Point2f(m_roi.tl())));

            }
        
            armor.m_pairs.reserve(2);


            Light left,right;
            left.m_center = cv::Point2f((armor.m_vertices[0].x + armor.m_vertices[1].x) / 2, (armor.m_vertices[0].y + armor.m_vertices[1].y) / 2);
            right.m_center = cv::Point2f((armor.m_vertices[2].x + armor.m_vertices[3].x) / 2, (armor.m_vertices[2].y + armor.m_vertices[3].y) / 2);
            left.m_length = getDistance(armor.m_vertices[1] , armor.m_vertices[0]);
            right.m_length = getDistance(armor.m_vertices[2] , armor.m_vertices[3]);
            left.m_width = left.m_length / m_Lightratio;
            right.m_width = right.m_length / m_Lightratio;
            left.m_ratio = m_Lightratio;
            right.m_ratio = m_Lightratio;
            left.m_angle = std::atan2((armor.m_vertices[1].y - armor.m_vertices[0].y) , (armor.m_vertices[1].x - armor.m_vertices[0].x)) * 180 / PI -90;
            right.m_angle = std::atan2((armor.m_vertices[2].y - armor.m_vertices[3].y) , (armor.m_vertices[2].x - armor.m_vertices[3].x)) * 180 / PI -90;

            left.m_rect = cv::RotatedRect(left.m_center,cv::Size(left.m_width * 1.4,left.m_length * 1.4),left.m_angle);
            right.m_rect = cv::RotatedRect(right.m_center,cv::Size(right.m_width * 1.4,right.m_length * 1.4),right.m_angle);

            left.regularRect(left.m_rect);
            right.regularRect(right.m_rect);
            
            armor.m_pairs[0] = left;
            armor.m_pairs[1] = right;

            //计算装甲板中心
            armor.m_center = cv::Point2f((left.m_center.x + right.m_center.x) / 2,
                            (left.m_center.y + right.m_center.y) / 2);
            armor.m_width = getDistance(left.m_center, right.m_center); //横向长度
            armor.m_height = (left.m_length + right.m_length) / 2;      //纵向长度
            armor.m_ratio = armor.m_width / armor.m_height;                         //长宽比
            armor.m_armor_type = armor.m_ratio > 3 ? wmj::ARMOR_LARGE : wmj::ARMOR_SMALL; //装甲板分类

            armor.m_id = m_label2id[m_bboxes[i].tag_id];
            armor.m_rect = cv::Rect2d(armor.m_vertices[0],cv::Size2d{armor.m_width,armor.m_height});
            armor.m_rect &= cv::Rect2d(cv::Point2d(0, 0), cv::Point2d(1280, 1024));

            m_armors.emplace_back(armor);
        }
    }

    void DeepDetector::generate_grids_and_stride(const int target_w, const int target_h, std::vector<int>& strides, std::vector<GridAndStride>& grid_strides)
    {
        for (auto stride : strides)
        {
            int num_grid_w = target_w / stride;
            int num_grid_h = target_h / stride;
            for (int g1 = 0; g1 < num_grid_h; g1++)
            {
                for (int g0 = 0; g0 < num_grid_w; g0++)
                {
                    grid_strides.push_back((GridAndStride){g0, g1, stride});
                }
            }
        }
    }

    void DeepDetector::decode_YOLOX_outputs(const float* prob, std::vector<bbox_t>& objects, const int img_w, const int img_h)
    {
        proposals.clear();
        generate_yolox_proposals(grid_strides, prob,  m_confThreshold, proposals);
        qsort_descent_inplace(proposals);
        std::vector<int> picked;
        nms_sorted_bboxes(proposals, picked, m_nmsThreshold);
        int count = picked.size();
        objects.resize(count);
        for (int i = 0; i < count; i++)
        {
            objects[i] = proposals[picked[i]];
        }
    }

    void DeepDetector::generate_yolox_proposals(std::vector<GridAndStride> grid_strides, const float* feat_ptr, float prob_threshold, std::vector<bbox_t>& objects)
    {

        const int num_anchors = grid_strides.size();

        for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++)
        {
            const int grid0 = grid_strides[anchor_idx].grid0;
            const int grid1 = grid_strides[anchor_idx].grid1;
            const int stride = grid_strides[anchor_idx].stride;

            const int basic_pos = anchor_idx * (objectLength); 

            // yolox/models/yolo_head.py decode logic
            //  outputs[..., :2] = (outputs[..., :2] + grids) * strides
            //  outputs[..., 2:4] = torch.exp(outputs[..., 2:4]) * strides
            float x_center = (feat_ptr[basic_pos + 0] + grid0) * stride;
            float y_center = (feat_ptr[basic_pos + 1] + grid1) * stride;
            cv::Point2f _pts[4];
            for(int i = 0; i < 4; i++)
            {
                _pts[i].x = (feat_ptr[basic_pos + i * 2 + 5] + grid0) * stride;
                _pts[i].y = (feat_ptr[basic_pos + i * 2 + 6] + grid1) * stride;
            }
            
            float w = exp(feat_ptr[basic_pos + 2]) * stride;
            float h = exp(feat_ptr[basic_pos + 3]) * stride;
            float x0 = x_center - w * 0.5f;
            float y0 = y_center - h * 0.5f;

            int tag_id;
            float box_objectness = feat_ptr[basic_pos + 4];
            float box_cls_score = maxclass(&feat_ptr[basic_pos + 13 + ncolor],ntag,tag_id);
            float box_prob = box_objectness * box_cls_score;
            if (box_prob > prob_threshold)
            {
                bbox_t obj;
                obj.color_conf = maxclass(&feat_ptr[basic_pos + 13],ncolor,obj.color_id);
                // obj.color_id = feat_ptr[basic_pos + 13] > feat_ptr[basic_pos + 14] ? 1 : 0;
            //    switch (obj.color_id)
            //    {
            //    case 0:
            //        obj.color_id = wmj::_COLOR::_BLUE;
            //        break;
            //    case 1:
            //        obj.color_id = wmj::_COLOR::_RED;
            //        break;
            //    case 2:
            //    default:
            //        obj.color_id = wmj::_COLOR::_WHITE;
            //        break;
            //    }
                if(obj.color_id != m_enemy_color && obj.color_id != wmj::_COLOR::_WHITE)
                    continue;
                obj.rect.x = x0 / m_resize_scale;
                obj.rect.y = y0 / m_resize_scale;
                obj.rect.width = w / m_resize_scale;
                obj.rect.height = h / m_resize_scale;
                for(int i = 0; i < 4; i++)
                    obj.pts[i] = _pts[i] / m_resize_scale;
                obj.conf = box_prob;
                obj.tag_id = tag_id;
                objects.push_back(obj);
            }

            // for (int class_idx = 0; class_idx < objectLength - 15; class_idx++)
            // {
            //     float box_cls_score = feat_ptr[basic_pos + 15 + class_idx];
            //     float box_prob = box_objectness * box_cls_score;
            //     if (box_prob > prob_threshold)
            //     {
            //         bbox_t obj;
            //         obj.color_id = feat_ptr[basic_pos + 13] > feat_ptr[basic_pos + 14] ? 1 : 0;
            //         if(obj.color_id != m_enemy_color)
            //             continue;
            //         obj.rect.x = x0 / m_resize_scale;
            //         obj.rect.y = y0 / m_resize_scale;
            //         obj.rect.width = w / m_resize_scale;
            //         obj.rect.height = h / m_resize_scale;
            //         for(int i = 0; i < 4; i++)
            //             obj.pts[i] = _pts[i] / m_resize_scale;
            //         obj.conf = box_prob;
            //         obj.tag_id = class_idx;
            //         objects.push_back(obj);
            //     }

            // } // class loop

        } // point anchor loop
    }

    void DeepDetector::nms_sorted_bboxes(const std::vector<bbox_t>& faceobjects, std::vector<int>& picked, float nms_threshold)
    {
        picked.clear();

        const int n = faceobjects.size();

        std::vector<float> areas(n);
        for (int i = 0; i < n; i++)
        {
            areas[i] = faceobjects[i].rect.area();
        }

        for (int i = 0; i < n; i++)
        {
            const bbox_t& a = faceobjects[i];

            int keep = 1;
            for (int j = 0; j < (int)picked.size(); j++)
            {
                const bbox_t& b = faceobjects[picked[j]];

                // intersection over union
                float inter_area = intersection_area(a, b);
                float union_area = areas[i] + areas[picked[j]] - inter_area;
                // float IoU = inter_area / union_area
                if (inter_area / union_area > nms_threshold)
                    keep = 0;
            }

            if (keep)
                picked.push_back(i);
        }
    }

    void DeepDetector::DebugOutput()
    {
        std::cout << _yellow("***************DeepDetector Debug Output***************") << std::endl;
        cv::imshow("input_mat",input_mat);
        std::cout << _lightcyan("Detected Armor number: ") << m_bboxes.size() << std::endl;
        
        //draw box
        for(auto bbox : m_bboxes)
        {
            cv::line(m_src, bbox.pts[0] + cv::Point2f(m_roi.tl()), bbox.pts[2] + cv::Point2f(m_roi.tl()), cv::Scalar(255, 255, 255), 2);
            cv::line(m_src, bbox.pts[1] + cv::Point2f(m_roi.tl()), bbox.pts[3] + cv::Point2f(m_roi.tl()), cv::Scalar(255, 255, 255), 2);
            cv::putText(m_src,"ID: " + std::to_string(m_label2id[bbox.tag_id]),
                        bbox.pts[1] + cv::Point2f(m_roi.tl()) + cv::Point2f(0,30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 1);
            cv::putText(m_src,"conf: " + std::to_string(bbox.conf),
                        bbox.pts[1] + cv::Point2f(m_roi.tl()) + cv::Point2f(0,60), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 1);
            cv::putText(m_src,"color: " + std::to_string(bbox.color_id),
                        bbox.pts[1] + cv::Point2f(m_roi.tl()) + cv::Point2f(0,90), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 1);
            cv::putText(m_src,"color_conf: " + std::to_string(bbox.color_conf),
                        bbox.pts[1] + cv::Point2f(m_roi.tl()) + cv::Point2f(0,120), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 1);
            cv::rectangle(m_src, m_roi, cv::Scalar(255), 2);
        }

        cv::imshow("result",m_src);
	    //cv::waitKey(1);
        // std::cout << _yellow("***********************Debug END***********************") << std::endl;
        return ;
    }



}
