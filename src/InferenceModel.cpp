#include "InferenceModel.hpp"

//==============================================================================
// YOLOv6ONNX й╣ож
//==============================================================================
YOLOv6ONNX::YOLOv6ONNX(const std::wstring &model_path, int input_height, int input_width)
    : m_input_height(input_height), m_input_width(input_width)
{
    m_env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "YOLOv6");
    Ort::SessionOptions session_options;
    m_session = std::make_unique<Ort::Session>(*m_env, model_path.c_str(), session_options);
}

std::vector<InferenceModel::DetectionResult> YOLOv6ONNX::infer(const cv::Mat &image)
{
    float ratio, pad_w, pad_h;
    cv::Mat processed = preprocess(image, ratio, pad_w, pad_h);

    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    std::vector<int64_t> input_shape = {1, 3, m_input_height, m_input_width};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, m_input_data.data(), m_input_data.size(),
        input_shape.data(), input_shape.size());

    auto output_tensors = m_session->Run(Ort::RunOptions{nullptr},
                                         m_input_names.data(), &input_tensor, 1,
                                         m_output_names.data(), 1);

    float *output_data = output_tensors[0].GetTensorMutableData<float>();
    auto output_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();

    return postprocess(output_data, output_shape, ratio, pad_w, pad_h);
}

cv::Mat YOLOv6ONNX::preprocess(const cv::Mat &image, float &ratio, float &pad_w, float &pad_h)
{
    const cv::Scalar pad_color(114, 114, 114);
    cv::Size orig_size = image.size();
    ratio = std::min(static_cast<float>(m_input_height) / orig_size.height,
                     static_cast<float>(m_input_width) / orig_size.width);
    cv::Size new_unpad(static_cast<int>(round(orig_size.width * ratio)),
                       static_cast<int>(round(orig_size.height * ratio)));

    cv::Mat resized;
    if (orig_size != new_unpad)
    {
        cv::resize(image, resized, new_unpad, 0, 0, cv::INTER_LINEAR);
    }
    else
    {
        resized = image.clone();
    }

    int dw = m_input_width - new_unpad.width;
    int dh = m_input_height - new_unpad.height;
    pad_w = dw / 2.0f;
    pad_h = dh / 2.0f;

    cv::copyMakeBorder(resized, resized,
                       static_cast<int>(pad_h), dh - static_cast<int>(pad_h),
                       static_cast<int>(pad_w), dw - static_cast<int>(pad_w),
                       cv::BORDER_CONSTANT, pad_color);

    cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
    processToInputData(resized);
    return resized;
}

void YOLOv6ONNX::processToInputData(const cv::Mat &processed)
{
    cv::Mat float_image;
    processed.convertTo(float_image, CV_32F, 1.0f / 255.0f);

    std::vector<cv::Mat> chw_channels;
    cv::split(float_image, chw_channels);

    m_input_data.clear();
    for (const auto &channel : chw_channels)
    {
        m_input_data.insert(m_input_data.end(),
                            channel.ptr<float>(),
                            channel.ptr<float>() + channel.total());
    }
}

std::vector<InferenceModel::DetectionResult> YOLOv6ONNX::postprocess(
    const float *output_data,
    const std::vector<int64_t> &output_shape,
    float ratio, float pad_w, float pad_h)
{

    constexpr float confidence_threshold = 0.6f;
    constexpr float nms_threshold = 0.3f;
    constexpr int num_classes = 2;
    constexpr int data_per_box = 5 + num_classes;

    std::vector<DetectionResult> results;
    const int num_boxes = static_cast<int>(output_shape[1]);

    for (int i = 0; i < num_boxes; ++i)
    {
        const float *ptr = output_data + i * data_per_box;
        const float obj_score = ptr[4];
        const float *cls_scores = ptr + 5;

        // conf = obj_conf * cls_conf
        int class_id = static_cast<int>(std::max_element(cls_scores, cls_scores + num_classes) - cls_scores);
        const float final_score = obj_score * cls_scores[class_id];

        if (final_score > confidence_threshold)
        {
            DetectionResult res;
            res.confidence = final_score;
            res.class_id = class_id;

            const float x_center = (ptr[0] - pad_w) / ratio;
            const float y_center = (ptr[1] - pad_h) / ratio;
            const float width = ptr[2] / ratio;
            const float height = ptr[3] / ratio;

            res.box.x = static_cast<int>(x_center - width / 2);
            res.box.y = static_cast<int>(y_center - height / 2);
            res.box.width = static_cast<int>(width);
            res.box.height = static_cast<int>(height);

            // Boundary check
            res.box.x = std::clamp(res.box.x, 0, static_cast<int>((m_input_width - pad_w * 2) / ratio));
            res.box.y = std::clamp(res.box.y, 0, static_cast<int>((m_input_height - pad_h * 2) / ratio));
            res.box.width = std::clamp(res.box.width, 1,
                                       static_cast<int>((m_input_width - pad_w * 2) / ratio) - res.box.x);
            res.box.height = std::clamp(res.box.height, 1,
                                        static_cast<int>((m_input_height - pad_h * 2) / ratio) - res.box.y);

            results.push_back(res);
        }
    }

    // NMS processing
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    for (const auto &res : results)
    {
        boxes.push_back(res.box);
        scores.push_back(res.confidence);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, scores, confidence_threshold, nms_threshold, indices);

    std::vector<DetectionResult> final_results;
    for (int idx : indices)
    {
        final_results.push_back(results[idx]);
    }

    return final_results;
}