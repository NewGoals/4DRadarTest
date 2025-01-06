#include "TcpCommandHandler.hpp"  // ����������Windows��ض���
#include "SensorData.hpp"
#include "DataReaderFactory.hpp"
#include "DisplayManager.hpp"   // ���м���eigen����opencv
#include "SynchronizedCollector.hpp"
#include "PclTools.hpp"
// #include "toolstest.hpp"
#include <filesystem>
#include <opencv2/core/utils/logger.hpp>
#include <pcl/visualization/cloud_viewer.h>


// ���ڴ�ӡʮ����������
void printHex(const std::vector<uint8_t>& data) {
    for (uint8_t byte : data) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;
}

// void testVideoReader() {
//     // ������Ƶ�ļ�·��
//     std::string videoPath = "E:/dataset/How Tower Cranes Build Themselves1080p.mp4";  // �滻Ϊ�����Ƶ·��
//     std::string savePath = "test_output.avi";  // ����·��
    
//     // ʹ�ù����ഴ����Ƶ��ȡ��
//     auto reader = DataReaderFactory::createReader(ReaderType::VIDEO_STREAM, videoPath);
//     if (!reader) {
//         std::cerr << "Failed to create video reader!" << std::endl;
//         return;
//     }
    
//     // ת��ΪVideoStreamReader��ʹ���ض�����
//     auto videoReader = std::dynamic_pointer_cast<VideoStreamReader>(reader);
//     if (!videoReader) {
//         std::cerr << "Failed to cast to VideoStreamReader!" << std::endl;
//         return;
//     }
    
//     // ��ʼ��
//     if (!videoReader->init()) {
//         std::cerr << "Failed to initialize video reader!" << std::endl;
//         return;
//     }
    
//     // ���ñ���
//     if (!videoReader->enableSave(savePath, 30.0)) {
//         std::cerr << "Failed to enable video saving!" << std::endl;
//         return;
//     }
    
//     int frameCount = 0;
//     // ��ȡ��������Ƶ֡
//     while (videoReader->grabFrame()) {
//         auto data = videoReader->getData();
//         if (data) {
//             frameCount++;
//             std::cout << "Frame " << frameCount 
//                       << ", Timestamp: " << data->timestamp << "ms" << std::endl;
            
//             // ��ʾ��ǰ֡
//             cv::Mat& frame = std::dynamic_pointer_cast<ImageData>(data)->frame;
//             cv::imshow("Video", frame);
//             if (cv::waitKey(1) == 27) {  // ESC���˳�
//                 break;
//             }
//         }
//     }
    
//     videoReader->disableSave();
//     cv::destroyAllWindows();
//     std::cout << "Processed " << frameCount << " frames" << std::endl;
// }

// void testTcpClient() {
//     // ����TCP�ͻ���ʵ��
//     TcpClient client("192.168.10.117", 50000);
    
//     // �������ӷ�����
//     if (!client.connect()) {
//         std::cout << "���ӷ�����ʧ�ܣ�" << std::endl;
//         return;
//     }
    
//     std::cout << "�ɹ����ӵ���������" << std::endl;
    
//     // ׼�����͵Ĳ�������
//     std::vector<uint8_t> sendData = {0x01, 0x02, 0x03, 0x04, 0x05};
    
//     // ��������
//     ssize_t sendResult = client.write(sendData.data(), sendData.size());
//     if (sendResult < 0) {
//         std::cout << "��������ʧ�ܣ�" << std::endl;
//         client.disconnect();
//         return;
//     }
    
//     std::cout << "�ɹ����� " << sendResult << " �ֽڵ�����" << std::endl;
    
//     // ׼�����ջ�����
//     std::vector<uint8_t> recvBuffer(1024);
    
//     // ������
//     ssize_t recvResult = client.read(recvBuffer.data(), recvBuffer.size());
//     if (recvResult < 0) {
//         std::cout << "��������ʧ�ܣ�" << std::endl;
//     } else if (recvResult > 0) {
//         std::cout << "���յ� " << recvResult << " �ֽڵ����ݣ�";
//         // ��ӡ���յ������ݣ�ʮ�����Ƹ�ʽ��
//         for (ssize_t i = 0; i < recvResult; ++i) {
//             printf("%02X ", recvBuffer[i]);
//         }
//         std::cout << std::endl;
//     }
    
//     // �ȴ�һ��ʱ��
//     std::this_thread::sleep_for(std::chrono::seconds(1));
    
//     // �Ͽ�����
//     client.disconnect();
//     std::cout << "�ѶϿ�����" << std::endl;
// }

// �����״� 
void testRadar() {
    try {
        // �����������
        TcpCommandHandler handler("192.168.53.168", 12345);
        
        // ��������Ŀ¼
        std::string saveDir = "radar_data";
        if (!std::filesystem::exists(saveDir)) {
            std::filesystem::create_directory(saveDir);
        }
        
        // �����״�
        if (!handler.connect()) {
            std::cerr << "�޷����ӵ��״��豸" << std::endl;
            return;
        }
        std::cout << "�ɹ����ӵ��״��豸" << std::endl;

        // ��ѭ��
        int count = 0;
        const int totalFrames = 200;
        const int printInterval = 10;  // ÿ10֡��ӡһ��
        auto lastTime = std::chrono::steady_clock::now();
        auto lastPrintTime = std::chrono::steady_clock::now();
        
        while (count < totalFrames) {
            CommandParseResult result;
            if (handler.receiveAndParseFrame(result)) {
                if (auto* targets = std::get_if<std::vector<TargetInfoParse_0xA8::TargetInfo>>(&result)) {
                    // ����ʱ����ļ���
                    auto now = std::chrono::system_clock::now();
                    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch()).count();
                    std::string filename = saveDir + "/radar_" + std::to_string(timestamp) + ".csv";
                    
                    // ���浱ǰ֡����
                    if (handler.startRecording(filename, "csv")) {
                        std::cout << "����֡���ݵ�: " << filename << std::endl;
                        std::cout << "���յ�Ŀ������: " << targets->size() << std::endl;
                        handler.saveTargetData(*targets, "csv");
                        handler.stopRecording();
                    }
                    
                    // ÿ��һ��֡���Ŵ�ӡ����
                    if (count % printInterval == 0) {
                        auto now = std::chrono::steady_clock::now();
                        auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - lastPrintTime).count();
                        lastPrintTime = now;
                        
                        float progress = (count + 1) * 100.0f / totalFrames;
                        std::cout << "\r�ɼ�����: " << std::fixed << std::setprecision(1) 
                                  << progress << "% (" << (count + 1) << "/" << totalFrames 
                                  << "), ƽ���ɼ����: " << interval / printInterval << "ms" 
                                  << std::flush;  // ʹ��\r��std::flushʵ��ԭ�ظ���
                    }
                }
                count++;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << std::endl;  // ��ɺ���
        
        handler.disconnect();
        std::cout << "������ɣ��ѶϿ�����" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "�״�����쳣: " << e.what() << std::endl;
    }
}

// void testMultiSourceCapture() {
//     try {
//         SynchronizedCollector collector;
//         std::cout << "�����ɼ����ɹ�" << std::endl;
        
//         // ���������Դ���״
//         auto radarSource = std::make_unique<RadarSource>("192.168.10.117", 50000, "radar");
//         if (!radarSource->init()) {
//             throw std::runtime_error("�״��ʼ��ʧ��");
//         }
//         collector.addSource(std::move(radarSource), true);
//         std::cout << "����״�Դ�ɹ�" << std::endl;
        
//         // ��Ӵ�����Դ����Ƶ��
//         // auto videoSource = std::make_unique<VideoSource>(
//         //     "E:/dataset/How Tower Cranes Build Themselves1080p.mp4", "camera1");
//         // // auto videoSource = std::make_unique<VideoSource>(
//         // //     "rtsp://192.168.88.216/live/chn0/stream_0", "camera1");
//         // if (!videoSource->init()) {
//         //     throw std::runtime_error("��ƵԴ��ʼ��ʧ��");
//         // }
//         // collector.addSource(std::move(videoSource), false);
//         // std::cout << "�����ƵԴ�ɹ�" << std::endl;
        
//         collector.start();
        
//         while (true) {
//             if (cv::waitKey(1000) == 27) {
//                 std::cout << "��⵽ESC����׼���˳�..." << std::endl;
//                 break;
//             }
//             collector.printStats();
//         }
        
//         collector.stop();
//         std::cout << "�ɼ���ֹͣ" << std::endl;
        
//     } catch (const std::exception& e) {
//         std::cerr << "��Դ�ɼ��쳣: " << e.what() << std::endl;
//     }
// }

void testRadarFileReader() {
    std::cout << "��ʼ�����״����ݶ�ȡ..." << std::endl;
    
    // �����״����ݶ�ȡ��
    auto reader = DataReaderFactory::createReader(
        ReaderType::RADAR_FILE,
        "E:/dataset/radar_data_20241211/sync_data_20241211_151805/camera"  // �״������ļ���·��
    );
    
    if (!reader) {
        std::cerr << "������ȡ��ʧ��" << std::endl;
        return;
    }
    
    // ��ʼ����ȡ��
    if (!reader->init()) {
        std::cerr << "��ʼ����ȡ��ʧ��" << std::endl;
        return;
    }
    
    // ��ȡ����ӡ����
    int frameCount = 0;
    while (!reader->isEnd() && frameCount < 100) {  // ��ȡ10֡����
        if (reader->readNext()) {
            auto data = std::dynamic_pointer_cast<RadarData>(reader->getData());
            if (data) {
                std::cout << "֡ " << frameCount 
                         << " ʱ���: " << data->timestamp
                         << " ����: " << data->points.size() << std::endl;
                
                // ��ӡ��һ��������ݣ�����У�
                if (!data->points.empty()) {
                    const auto& p = data->points[0];
                    std::cout << "��һ����: x=" << p.x 
                             << ", y=" << p.y 
                             << ", z=" << p.z 
                             << ", rcs=" << p.rcs 
                             << ", v_r=" << p.v_r << std::endl;
                }
            }
            frameCount++;
        }
    }
    
    std::cout << "������ɣ�����ȡ " << frameCount << " ֡����" << std::endl;
}

void testDisplayManager(){
    try
    {
        std::cout << "��ʼ������ʾ������..." << std::endl;
        DisplayManager displayManager;

        // ��ӿ��ӻ���
        displayManager.addVisualizer(DisplayManager::DisplayType::POINT_CLOUD, std::make_shared<PointCloudVisualizer>());
        displayManager.addVisualizer(DisplayManager::DisplayType::IMAGE, std::make_shared<ImageVisualizer>());
        
        // ���ô��ڲ���
        displayManager.setLayout(DisplayManager::DisplayType::POINT_CLOUD, 100, 100, 800, 600);

        displayManager.setLayout(DisplayManager::DisplayType::IMAGE, 150, 400, 800, 600);

        // ���ļ��ж�ȡ����
        auto radar_reader = DataReaderFactory::createReader(ReaderType::RADAR_FILE, "E:/Source/4DRadarTest/build/Debug/sync_data_20241231_155245/radar");
        if (!radar_reader->init()) {
            std::cerr << "��ʼ����ȡ��ʧ��" << std::endl;
            return;
        }

        auto image_reader = DataReaderFactory::createReader(ReaderType::IMAGE_FILE, "E:/Source/4DRadarTest/build/Debug/sync_data_20241231_155245/camera_near");
        if (!image_reader->init()) {
            std::cerr << "��ʼ����ȡ��ʧ��" << std::endl;
            return;
        }
        
        // ��ȡ����
        image_reader->readNext();
        
        while (!radar_reader->isEnd()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(60));  // ��Ϊ5ms
            if (radar_reader->readNext()) {

                displayManager.updateDisplay(radar_reader->getData());
                
                while(std::dynamic_pointer_cast<ImageFileReader>(image_reader)->getData()->timestamp < std::dynamic_pointer_cast<RadarFileReader>(radar_reader)->getData()->timestamp){
                    image_reader->readNext();
                }

                displayManager.updateDisplay(image_reader->getData()); 
                displayManager.renderAll();
            }
        }
        
        // image_reader->readNext();
        // radar_reader->readNext();
        // while(true){
        //     std::this_thread::sleep_for(std::chrono::milliseconds(5));
        //     displayManager.updateDisplay(radar_reader->getData());
        //     displayManager.updateDisplay(image_reader->getData());
        //     displayManager.renderAll();
        // }
    }
    catch(const std::exception& e)
    {
        std::cerr << "testDisplayManager �쳣: " << e.what() << '\n';
    }
}


void testDisplaySingleRadarPoint(){
    std::cout << "��ʼ������ʾ������..." << std::endl;
    DisplayManager displayManager;

    // ��ӿ��ӻ���
    displayManager.addVisualizer(DisplayManager::DisplayType::POINT_CLOUD, std::make_shared<PointCloudVisualizer>());
    // ���ô��ڲ���
    displayManager.setLayout(DisplayManager::DisplayType::POINT_CLOUD, 100, 100, 800, 600);
}


void testRealTimeRadarDisplay(){
    try{
        SynchronizedCollector collector;
        std::cout << "�����ɼ����ɹ�" << std::endl;
        
        // ���������Դ���״
        auto radarSource = std::make_unique<RadarSource>("192.168.88.219", 12345, "radar");
        if (!radarSource->init()) {
            throw std::runtime_error("�״��ʼ��ʧ��");
        }
        collector.addSource(std::move(radarSource), true);
        std::cout << "����״�Դ�ɹ�" << std::endl;

        // ��Ӵ���ƵԴ
        // auto videoSource = std::make_unique<VideoSource>(
        //     "E:/dataset/How Tower Cranes Build Themselves1080p.mp4", "camera");
        // auto videoSource = std::make_unique<VideoSource>(
        //     "rtsp://127.0.0.1:8554/camera_test", "camera");
        
        auto videoSource_near = std::make_unique<VideoSource>("rtsp://192.168.88.219:554/live/chn1/stream_1", "camera_near");
        if (!videoSource_near->init()) {
            throw std::runtime_error("��ƵԴ1��ʼ��ʧ��");
        }
        collector.addSource(std::move(videoSource_near), false);
        
        // auto videoSource_far = std::make_unique<VideoSource>("rtsp://192.168.88.219:554/live/chn0/stream_1", "camera_far");
        // if (!videoSource_far->init()) {
        //     throw std::runtime_error("��ƵԴ2��ʼ��ʧ��");
        // }
        // collector.addSource(std::move(videoSource_far), false);
        std::cout << "�����ƵԴ�ɹ�" << std::endl;

        // ���ñ��棬(�״���)
        collector.setSaveConfig(true, true, RadarFileReader::Format::BIN);

        // �������ӻ���
        DisplayManager displayManager;
        displayManager.addVisualizer(DisplayManager::DisplayType::POINT_CLOUD, std::make_shared<PointCloudVisualizer>());
        displayManager.addVisualizer(DisplayManager::DisplayType::IMAGE, std::make_shared<ImageVisualizer>());

        // ���ô��ڲ���
        displayManager.setLayout(DisplayManager::DisplayType::POINT_CLOUD, 100, 100, 800, 600);
        displayManager.setLayout(DisplayManager::DisplayType::IMAGE, 200, 150, 800, 600);
        // �����߳�
        collector.start();

        // �ɼ�״̬��Ϣ��ӡ���
        int printInterval = 10;     // �����״�Ĳɼ�Ƶ��
        int count = 0;

        while(true){
            std::this_thread::sleep_for(std::chrono::milliseconds(5));  // ��Ϊ5ms
            if(count % printInterval == 0){
                collector.printStats();
            }
            count++;
            auto newRadarData = collector.getMainSourceData();
            displayManager.updateDisplay(std::move(newRadarData));
            // ��ȡ��Դ���� 
            auto subData = collector.getSubSourceData();
            for(const auto& data : subData){
                auto imageData = std::make_shared<ImageData>();
                imageData->frame = data.second;  // ��ȷ���� frame
                displayManager.updateDisplay(imageData);
            }
            // ��Ⱦ���п��ӻ���
            displayManager.renderAll();
        }

        collector.stop();
    }
    catch(const std::exception& e){
        std::cerr << "testDisplayManager �쳣: " << e.what() << '\n';
    }
}

struct BoundingBox {
    pcl::PointXYZL min_pt;
    pcl::PointXYZL max_pt;
};
std::map<std::string, BoundingBox> boundingBoxes; // �洢���������С�������


void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* viewer_void){
    pcl::visualization::PCLVisualizer::Ptr viewer = *static_cast<pcl::visualization::PCLVisualizer::Ptr*>(viewer_void);
    if(!event.getPointIndex() != -1){
        float x, y, z;
        event.getPoint(x, y, z);
        std::cout << "������: (" << x << ", " << y << ", " << z << ")" << std::endl;

        // ���Ұ������������
        for(const auto& [box_id, bbox] : boundingBoxes){
            if (x >= bbox.min_pt.x && x <= bbox.max_pt.x &&
                y >= bbox.min_pt.y && y <= bbox.max_pt.y &&
                z >= bbox.min_pt.z && z <= bbox.max_pt.z)
            {
                std::cout << "������ ID: " << box_id << std::endl;
                std::cout << "��: " << bbox.max_pt.x - bbox.min_pt.x << std::endl;
                std::cout << "��: " << bbox.max_pt.y - bbox.min_pt.y << std::endl;
                std::cout << "��: " << bbox.max_pt.z - bbox.min_pt.z << std::endl;
                std::cout << "����: (" << (bbox.min_pt.x + bbox.max_pt.x) / 2 << ", "
                          << (bbox.min_pt.y + bbox.max_pt.y) / 2 << ", "
                          << (bbox.min_pt.z + bbox.max_pt.z) / 2 << ")" << std::endl;
                // ��ѡ�е���������ɫ����Ϊ��ɫ
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, box_id); // ��ɫ (R=1.0, G=1.0, B=0.0)
                viewer->spinOnce(); // ˢ�¿��ӻ�����
                break;
            }
        }
    }
}

std::vector<cv::Mat> calib() {
    std::vector<cv::Mat> calib_result;
    // �״�㣨3D �㣩- ÿ��۲���Ҫ��vector<Point3f>     
    std::vector<cv::Point3f> radar_points_single = {
        {-5.24, 37.73, -0.11},
        {-3.34, 29.92, -0.90},
        {-2.53, 19.24, -0.96},
        {-4.74, 32.34, -0.48},
        {-3.63, 13.93, -0.09},
        {-0.83, 16.18, 0.39},
        {-3.73, 23.09, -0.64}
    };
    std::vector<std::vector<cv::Point3f>> radar_points(1, radar_points_single);

    // ��Ӧ��ͼ�����ص㣨2D �㣩- ÿ��۲���Ҫ��vector<Point2f>
    std::vector<cv::Point2f> image_points_single = {
        {834, 370},
        {889, 370},
        {836, 381},
        {824, 370},
        {606, 414},
        {990, 399},
        {786, 382}
    };
    std::vector<std::vector<cv::Point2f>> image_points(1, image_points_single);

    int image_width = 1920;
    int image_height = 1080;
    // ��������ڲξ���ĳ�ʼֵ��ʹ�ø�����ĳ�ʼ���ƣ�
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0,0) = 1000.0; // fx
    camera_matrix.at<double>(1,1) = 1000.0; // fy
    camera_matrix.at<double>(0,2) = image_width/2.0;  // cx
    camera_matrix.at<double>(1,2) = image_height/2.0; // cy

    // �������ϵ��
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    // ������ε���ת������ƽ������
    std::vector<cv::Mat> rvecs, tvecs;


    // ���ñ궨������־
    int flags = cv::CALIB_USE_INTRINSIC_GUESS | 
                cv::CALIB_FIX_PRINCIPAL_POINT;

    // ʹ�� calibrateCamera ���б궨
    double reprojection_error = cv::calibrateCamera(
        radar_points,      // 3D ��
        image_points,      // 2D ��
        cv::Size(image_width, image_height), // ͼ��ߴ�
        camera_matrix,     // ����ڲξ���
        dist_coeffs,       // �������ϵ��
        rvecs,            // �����ת����
        tvecs,            // ���ƽ������
        flags             // �궨������־
    );

    calib_result.push_back(camera_matrix);
    calib_result.push_back(dist_coeffs);
    calib_result.push_back(rvecs[0]);
    calib_result.push_back(tvecs[0]);

    // ������
    std::cout << "����ڲξ��� K:\n" << camera_matrix << std::endl;
    std::cout << "����ϵ��:\n" << dist_coeffs << std::endl;
    std::cout << "��ת���� rvec:\n" << rvecs[0] << std::endl;
    std::cout << "ƽ������ tvec:\n" << tvecs[0] << std::endl;
    std::cout << "��ͶӰ���: " << reprojection_error << std::endl;

    // ��֤��ͶӰ���
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(radar_points_single, rvecs[0], tvecs[0], 
                     camera_matrix, dist_coeffs, projected_points);
    
    // ����ÿ�������ͶӰ���
    for(size_t i = 0; i < projected_points.size(); i++) {
        double error = cv::norm(image_points_single[i] - projected_points[i]);
        std::cout << "Point " << i << " reprojection error: " 
                  << error << " pixels" << std::endl;
    }

    return calib_result;
}

void projectRadarPoints(const std::vector<RadarPoint>& radar_points, 
                       cv::Mat& image,
                       const cv::Mat& camera_matrix,
                       const cv::Mat& dist_coeffs,
                       const cv::Mat& rvec,
                       const cv::Mat& tvec) {
    // ת���״��ΪOpenCV��ʽ
    std::vector<cv::Point3f> object_points;
    std::vector<float> object_points_speed;
    for (const auto& point : radar_points) {
        object_points.push_back(cv::Point3f(point.x, point.y, point.z));
        object_points_speed.push_back(point.v_r);
    }

    // ͶӰ3D�㵽ͼ��ƽ��
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, image_points);

    // ��ͼ���ϻ���ͶӰ��
    for (int i = 0; i < image_points.size(); i++) {
        // �����Ƿ���ͼ��Χ��
        if (image_points[i].x >= 0 && image_points[i].x < image.cols && 
            image_points[i].y >= 0 && image_points[i].y < image.rows) {
            // ����Բ�㣬���Ը�����Ҫ������С����ɫ
            if(object_points_speed[i] < 0){
                cv::circle(image, image_points[i], 3, cv::Scalar(0, 0, 255), -1);
            }
            else if(object_points_speed[i] > 0){
                cv::circle(image, image_points[i], 3, cv::Scalar(255, 0, 0), -1);
            }
            // cv::circle(image, image_points[i], 3, cv::Scalar(0, 255, 0), -1);
        }
    }
}

void testDBScan(){
    std::vector<cv::Mat> calib_result = calib();
    cv::Mat camera_matrix = calib_result[0];
    cv::Mat dist_coeffs = calib_result[1];
    cv::Mat rvec = calib_result[2];
    cv::Mat tvec = calib_result[3];

    // ���ļ��ж�ȡ����
    auto radar_reader = DataReaderFactory::createReader(ReaderType::RADAR_FILE, "E:/Source/4DRadarTest/build/Debug/sync_data_20241231_155245/radar");
    if (!radar_reader->init()) {
        std::cerr << "��ʼ����ȡ��ʧ��" << std::endl;
        return;
    }

    auto image_reader = DataReaderFactory::createReader(ReaderType::IMAGE_FILE, "E:/Source/4DRadarTest/build/Debug/sync_data_20241231_155245/camera_near");
    if (!image_reader->init()) {
        std::cerr << "��ʼ����ȡ��ʧ��" << std::endl;
        return;
    }
    DisplayManager displayManager;
    // ��ӿ��ӻ���
    displayManager.addVisualizer(DisplayManager::DisplayType::IMAGE, std::make_shared<ImageVisualizer>());
    displayManager.setLayout(DisplayManager::DisplayType::IMAGE, 150, 400, 800, 600);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("DBSCAN with Bounding Boxes"));
    viewer->setBackgroundColor(0, 0, 0);
    // ע�����ѡ��ص�����
    viewer->registerPointPickingCallback(pointPickingEventOccurred, &viewer);

    pcl::visualization::PCLVisualizer::Ptr viewer_source(new pcl::visualization::PCLVisualizer("Source cloud"));
    viewer_source->setBackgroundColor(0, 0, 0);

    // int index = 420;
    // for(int i = 0; i < index; i++){
    //     radar_reader->readNext();
    //     image_reader->readNext();
    // }

    while (!radar_reader->isEnd()) {
        // ÿ�θ���ǰ�����������
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        viewer_source->removeAllPointClouds();
        viewer_source->removeAllShapes();

        radar_reader->readNext();
        image_reader->readNext();

        auto currentRadarData = std::dynamic_pointer_cast<RadarData>(radar_reader->getData());
        // ���������ʾ
        pcl::PointCloud<pcl::PointXYZL>::Ptr input_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZL>>();
        pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        for(const auto& point : currentRadarData->points){
            pcl::PointXYZI p_rcs;
            pcl::PointXYZL p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            input_cloud->push_back(p);
            p_rcs.x = point.x;
            p_rcs.y = point.y;
            p_rcs.z = point.z;
            p_rcs.intensity = point.v_r;
            source_cloud->push_back(p_rcs);
        }

        //DBSCAN����
        float eps = 0.5;//dbscan�뾶����
        int min_pts = 5;//dbscan������С��������
        int number_of_clusters = 0;//����������Ҫ�޸�

        // ʹ��pcl��������dbscan
        number_of_clusters = PCLTools::dbscan(input_cloud, eps, min_pts);
        // dbscan(input_cloud, eps, min_pts, number_of_clusters);

        viewer->addPointCloud(input_cloud, "cluster_cloud");

        // Ϊÿ���ؼ������ʾ��С��Χ��
        std::vector<std::vector<int>> cluster_indices(number_of_clusters);
        for (size_t i = 0; i < input_cloud->points.size(); ++i) {
            if (input_cloud->points[i].label >= 0) {
                cluster_indices[input_cloud->points[i].label - 1].push_back(i);
            }
        }

        boundingBoxes.clear(); // ���֮ǰ����������Ϣ
        // Ϊÿ���ش�����Χ��
        for (int i = 0; i < number_of_clusters; ++i) {
            // // ��ȡ��ǰʱ���
            // auto start = std::chrono::high_resolution_clock::now();
            if (cluster_indices[i].size() < 1) continue;

            // ��ȡ��ǰ�صĵ���
            pcl::PointCloud<pcl::PointXYZL>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZL>);
            for (const auto& idx : cluster_indices[i]) {
                cluster->points.push_back(input_cloud->points[idx]);
            }

            // �����Χ�е���С�������
            pcl::PointXYZL min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);
            

            // ���ư�Χ�е�12����
            std::string box_id = "box_" + std::to_string(i);
            viewer->addCube(min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z, 
                1.0, 0.0, 0.0, box_id);
                // �����������͸���ȣ�0.0Ϊ��ȫ͸����1.0Ϊ��ȫ��͸����
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, box_id);

            // �洢���������С�������
            boundingBoxes[box_id] = {min_pt, max_pt};

            // // ��ȡ����ʱ���
            // auto end = std::chrono::high_resolution_clock::now();
            // // ����ʱ��ת��Ϊ����
            // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            // std::cout << "ִ��ʱ��: " << duration.count() << " ����" << std::endl;
        }

        
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(source_cloud, "intensity");

        viewer_source->addPointCloud<pcl::PointXYZI>(source_cloud, intensity_distribution, "source cloud");

        viewer->spinOnce(100);
        viewer_source->spinOnce(100);

        auto currentImageData = std::dynamic_pointer_cast<ImageData>(image_reader->getData());
        cv::Mat display_image = currentImageData->frame.clone();
        // ͶӰ�״�㵽ͼ����
        projectRadarPoints(currentRadarData->points, 
                         display_image,
                         camera_matrix,
                         dist_coeffs,
                         rvec,
                         tvec);
        auto display_imagedata = std::make_shared<ImageData>();
        display_imagedata->frame = display_image;
        displayManager.updateDisplay(display_imagedata); 
        displayManager.renderAll();

    }
}


int main() {
    std::cout << "����ʼ����..." << std::endl;
    std::cout.flush();  // ǿ��ˢ�����������
    // ����OpenCV��־����
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);

    try {
        // testDisplayManager();
        // testRealTimeRadarDisplay();
        // testPcltoolsCluster();
        
        testDBScan();
        // calib();
    } catch (const std::exception& e) {
        std::cerr << "�����쳣: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}

