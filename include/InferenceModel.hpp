#pragma once

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

// 基类：通用推理模型接口
class InferenceModel
{
public:
    struct DetectionResult
    {
        cv::Rect box;
        float confidence;
        int class_id;
    };

    virtual ~InferenceModel() = default;

    // 通用推理接口
    virtual std::vector<DetectionResult> infer(const cv::Mat &image) = 0;

protected:
    // 通用预处理（可被派生类覆盖）
    virtual cv::Mat preprocess(const cv::Mat &image, float &ratio, float &pad_w, float &pad_h) = 0;

    // 通用后处理（需派生类实现）
    virtual std::vector<DetectionResult> postprocess(
        const float *output_data,
        const std::vector<int64_t> &output_shape,
        float ratio, float pad_w, float pad_h) = 0;
};

class YOLOv6ONNX : public InferenceModel
{
private:
    // ONNX相关资源
    std::unique_ptr<Ort::Env> m_env;
    std::unique_ptr<Ort::Session> m_session;
    std::vector<const char *> m_input_names = {"images"};
    std::vector<const char *> m_output_names = {"outputs"};

    // 模型参数
    int m_input_height;
    int m_input_width;
    std::vector<float> m_input_data;

public:
    YOLOv6ONNX(const std::wstring &model_path, int input_height = 640, int input_width = 640);
    std::vector<DetectionResult> infer(const cv::Mat &image) override;

private:
    void processToInputData(const cv::Mat &processed);
    cv::Mat preprocess(const cv::Mat &image, float &ratio, float &pad_w, float &pad_h) override;
    std::vector<DetectionResult> postprocess(const float *output_data,
                                             const std::vector<int64_t> &output_shape,
                                             float ratio, float pad_w, float pad_h) override;
};