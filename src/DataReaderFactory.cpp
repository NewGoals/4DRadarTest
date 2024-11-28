#include "DataReaderFactory.hpp"

std::shared_ptr<IDataReader> DataReaderFactory::createReader(
    ReaderType type,
    const std::string& source,
    int baudRate
) {
    switch (type) {
        case ReaderType::IMAGE_FILE:
            return std::make_shared<ImageFileReader>(source);
            
        case ReaderType::RADAR_FILE:
            return std::make_shared<RadarFileReader>(source);
            
        case ReaderType::SERIAL:
            return std::make_shared<SerialReader>(source, baudRate);
            
        case ReaderType::VIDEO_STREAM:
            return std::make_shared<VideoStreamReader>(source);
            
        default:
            throw std::runtime_error("Unknown reader type");
    }
} 