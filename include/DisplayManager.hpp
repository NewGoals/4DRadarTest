// DisplayManager.hpp - ��ʾ������
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
    // ��ӿ��ӻ���
    void addVisualizer(DisplayType type, std::shared_ptr<IVisualizer> visualizer);
    // ���¿��ӻ���
    void updateDisplay(const std::shared_ptr<SensorData>& data);
    // ��Ⱦ���ӻ���
    void renderAll();
    // ���ô��ڲ���
    void setLayout(DisplayType type, int x, int y, int width, int height);
};


void DisplayManager::addVisualizer(DisplayType type, std::shared_ptr<IVisualizer> visualizer){
    visualizers[type] = visualizer;
}

void DisplayManager::updateDisplay(const std::shared_ptr<SensorData>& data){
    if(!data) return;

    // ������������ѡ����ʵĿ��ӻ���
    DisplayType type;
    if(std::dynamic_pointer_cast<ImageData>(data)) type = DisplayType::IMAGE;
    else if(std::dynamic_pointer_cast<RadarData>(data)) type = DisplayType::POINT_CLOUD;
    else return;

    auto it = visualizers.find(type);
    // �п��ӻ�����������Ч�����¿��ӻ���
    if(it != visualizers.end() && it->second) it->second->update(data);
}

void DisplayManager::renderAll(){
    for(auto& [type, vis] : visualizers){
        if(vis) vis->render();
    }
}

void DisplayManager::setLayout(DisplayType type, int x, int y, int width, int height){
    auto it = visualizers.find(type);
    if(it != visualizers.end() && it->second) it->second->setWindowLayout(x, y, width, height);
}

