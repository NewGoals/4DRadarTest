#pragma once
#include "DataReader.hpp"

enum class ReaderType {
    IMAGE_FILE,
    RADAR_FILE,
    SERIAL,
    VIDEO_STREAM
};

class DataReaderFactory {
public:
    static std::shared_ptr<IDataReader> createReader(
        ReaderType type, 
        const std::string& source,      // 文件路径或视频流URL
        int baudRate = 115200
    );
};