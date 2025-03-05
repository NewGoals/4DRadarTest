#include "DisplayManager.hpp"

void DisplayManager::addVisualizer(DisplayType type, std::shared_ptr<IVisualizer> visualizer){
    visualizers[type] = visualizer;
}

void DisplayManager::updateDisplay(const std::shared_ptr<SensorData>& data){
    if(!data) return;

    // 根据数据类型选择合适的可视化器
    DisplayType type;
    if(std::dynamic_pointer_cast<ImageData>(data)) type = DisplayType::IMAGE;
    else if(std::dynamic_pointer_cast<RadarData>(data)) type = DisplayType::POINT_CLOUD;
    else return;

    auto it = visualizers.find(type);
    // 有可视化器且数据有效，更新可视化器
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