// DisplayManager.hpp - 显示管理类
#pragma once

#include "Visualizer.hpp"
#include <memory>
#include <unordered_map>

class DisplayManager {
public:
    enum class DisplayType{
        IMAGE,
        POINT_CLOUD,
        FUSION
    };
private:
    std::unordered_map<DisplayType, std::shared_ptr<IVisualizer>> visualizers;
    bool initialized = false;

public:
    DisplayManager() = default;
    // 添加可视化器
    void addVisualizer(DisplayType type, std::shared_ptr<IVisualizer> visualizer);
    // 更新可视化器
    void updateDisplay(const std::shared_ptr<SensorData>& data);
    // 渲染可视化器
    void renderAll();
    // 设置窗口布局
    void setLayout(DisplayType type, int x, int y, int width, int height);
};

