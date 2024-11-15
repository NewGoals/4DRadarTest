// DisplayManager.hpp - 显示管理器
#pragma once
#include <map>
#include <string>
#include "Visualizer.hpp"
#include "DataReader.hpp"

class DisplayManager {
private:
    // GLFWwindow* window;
    std::map<std::string, std::unique_ptr<IVisualizer>> visualizers;
    std::map<std::string, std::unique_ptr<IDataReader>> readers;

public:
    bool init(int width, int height, const std::string& title);
    void addVisualizer(const std::string& name, std::unique_ptr<IVisualizer> visualizer);
    void addDataReader(const std::string& name, std::unique_ptr<IDataReader> reader);
    void run();
    void cleanup();
};