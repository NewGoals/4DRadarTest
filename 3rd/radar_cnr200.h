/*
 * Copyright (c) 2022-2022 湖南华诺星空电子技术有限公司. All rights reserved.
 */
#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <queue>
#include <atomic>
#include <thread>
#include <mutex>
#include <map>
#include <common/channel.hpp>
#include <common/node_data_type.h>
#include <common/node_config_type.h>
#include <hal/gpio.h>


class RadarCNR200 {
private:
    #pragma pack(push, 1)
/*    struct TargetFrameObj_general
    {
        unsigned int id;
        unsigned int type; //00 其他  02人  01 车  03树  15猫 16狗 17 马 18羊 19 牛
        float x_speed;
        float y_speed;
        float z_speed;
        float x_axes;          //x坐标
        float y_axes;          //y坐标
        float z_axes;          //z坐标
        float length;          //测量距离
        float azimuth_angle;   //方位角
        float elevation_angle; //俯仰角
        float SNR;             //信噪比
        float peakEnergy;      //峰值能量
        unsigned int currentFollowTarFlag;
        char tarDefenceFlag;    //reserved[0]为目标所在防区类型标记
        unsigned int timeRadar; //雷达目标出现的时间
        unsigned int radarFlag; //目标信息雷达标记
        char reserved[9];       //reserved[0]为目标所在防区类型标记
    } ;*/

    struct TargetFrameBasicMod {
        uint16_t mod_type;
        uint16_t mod_data_len;
        uint8_t mod_data_status;
        uint8_t target_cnt;
        uint8_t target_byte;
        uint8_t road_cnt;
        uint8_t reserved[8];
        uint8_t checksum;
    };
    static_assert(sizeof(TargetFrameBasicMod)==17, "TargetFrameBasicMod size error");
    struct TargetFrameObj {
        uint8_t num;
        uint16_t dist_long;
        int16_t dist_lat;
        int16_t v_long;
        uint8_t obj_type;
        uint8_t road_num;
        uint8_t reserved[4];
        int16_t v_lat;
        uint8_t reserved2[2];
    };
    static_assert(sizeof(TargetFrameObj)==17, "TargetFrameObj size error");
    struct TargetFrameObjMod {
        uint16_t mod_type;
        uint16_t mod_data_len;
        TargetFrameObj objs[0];
    };
    static_assert(sizeof(TargetFrameObjMod)==4, "TargetFrameObjMod size error");
    struct TargetFrame {
        uint32_t header;
        uint16_t data_len;
        TargetFrameBasicMod target_frame_basic;
        TargetFrameObjMod target_frame_obj;
    };

    //通用雷达结构体 100 150 200 500
    struct TargetFrame_general {
        uint16_t header;
        uint16_t addr;
        uint8_t cmd;
        uint16_t data_len;
        Cnr200_Mudslides cnr200_mudslides;
        RadarObj_general objs[0];
//        uint8_t check_sum;
    };
    struct _rTarget_Frame {
        uint8_t check_sum;
        TargetFrame_general target_frame_objs;
    };

    struct CmdFrame {
        uint32_t header;
        char command[64];
        uint32_t return_value;
        uint32_t flags;
        uint32_t prm_size;
        uint8_t params[0];
    };
    static_assert(sizeof(CmdFrame)==80, "CmdFrame size error");
    #pragma pack(pop)

    const char *CMD_CODE_BURN = "code_burn";
    const char *CMD_PARAM_WRITE = "param_write";
    const char *CMD_PARAM_READ = "param_read";
    const char *CMD_PARAM_GET = "param_get";
    const char *CMD_RADAR_RESET = "radar_reset";
    struct Command {
        std::string command;
        uint32_t return_value;
        uint32_t flags;
        std::vector<uint8_t> params;
    };

    enum ParseFrameState {PFS_START=0, PFS_TARGET_FRAME_DATA_LEN, PFS_TARGET_FRAME_DATA,
        PFS_CMD_FRAME_HEADER, PFS_CMD_FRAME_INFO, PFS_CMD_FRAME_PARAMS};
    enum ParamName {PN_ROAD_CNT=0, PN_ROAD_DIR, PN_ROAD_WIDTH1, PN_ROAD_WIDTH2, PN_ROAD_WIDTH3, PN_ROAD_WIDTH4,
        PN_ROAD_WIDTH5, PN_ROAD_WIDTH6, PN_ROAD_WIDTH7, PN_ROAD_WIDTH8, PN_ROAD_DIR1, PN_ROAD_DIR2,
        PN_ROAD_DIR3, PN_ROAD_DIR4, PN_ROAD_DIR5, PN_ROAD_DIR6, PN_ROAD_DIR7, PN_ROAD_DIR8,
        PN_HORIZON_SHIFT, PN_ANGLE_RECOUP, PN_DIST_LONG_RECOUP, PN_INSTALL_HEIGHT, PN_STOP_LINE_DIST,
        PN_RADAR_FREQ_ID, PN_SENSE, PN_BIKE_PERSION, PN_AUTO_DISTINCTION, PN_STOP_OBJ_TIMEOUT,
        PN_QUHUA_SWITCH, PN_QUHUA_ROAD_CNT, PN_QUHUA_DIR, PN_QUHUA_ISOLATE, PN_QUHUA_WIDTH1, PN_QUHUA_WIDTH2,
        PN_QUHUA_WIDTH3, PN_QUHUA_WIDTH4, PN_QUHUA_WIDTH5, PN_QUHUA_WIDTH6, PN_QUHUA_WIDTH7,
        PN_QUHUA_WIDTH8, PN_QUHUA_GREEN_W, PN_QUHUA_DIR1, PN_QUHUA_DIR2, PN_QUHUA_DIR3, PN_QUHUA_DIR4,
        PN_QUHUA_DIR5, PN_QUHUA_DIR6, PN_QUHUA_DIR7, PN_QUHUA_DIR8, PN_QUHUA_HORIZON_SHIFT,
        PN_QUHUA_UP_EDGE, PN_QUHUA_DOWN_EDGE, PN_FAR_STOP_LINE, PN_ISOLATE_POSITION, PN_ISOLATE_WIDTH,
        PN_LEFT_OUT_DIST1, PN_LEFT_OUT_WIDTH1, PN_LEFT_OUT_DIST2, PN_LEFT_OUT_WIDTH2, PN_MID_OUT_DIST1,
        PN_MID_OUT_WIDTH1, PN_MID_OUT_DIST2, PN_MID_OUT_WIDTH2, PN_RIGHT_OUT_DIST1, PN_RIGHT_OUT_WIDTH1,
        PN_RIGHT_OUT_DIST2, PN_RIGHT_OUT_WIDTH2, PN_CROSS_UP_EDGE, PN_CROSS_DOWN_EDGE,
        PN_FAR_CROSS_WIDTH, PN_WORK_MODE, PN_APPLY_FIELD, PN_LEAVE_MID_MIN_LEN, PN_LEAVE_BIG_MIN_LEN,
        PN_COME_MID_MIN_LEN, PN_COME_BIG_MIN_LEN
    };

    struct ParamInfo {
        uint32_t addr;
        const char *type_name;
        uint32_t size;
    };

public:
    static const int MAX_TRY_CNT = 3;
    static const int TIME_WAIT_START_MS = 3000;
    RadarCNR200();
    ~RadarCNR200();
    int32_t open();
    int32_t close();
    int32_t close_cut();
    int32_t get_obj_block(RadarData_general *p_data);
    int32_t get_version(std::string *p_version);
    int32_t set_apply_field(const std::string& apply_field);
    int32_t get_apply_field(std::string *p_apply_field);
    int32_t set_work_mode(bool debug_mode);
    int32_t set_road_cnt(uint32_t road_cnt);
    bool get_radar_status();
    int32_t config(Cnr200Config *p_conf);

    static void show_target_vec(std::vector<RadarObj_general> *p_obj_vec);

private:
    Gpio *p_power_ctrl{};
    std::atomic<bool> init_ok{};
    std::atomic<bool> config_ok{};
    std::atomic<bool> exit_flag{};
    std::thread *p_recv_thread{};
    std::map<std::string, Command> cmd_resp_pool{};   // key = frame_id
    std::mutex cmd_resp_pool_mtx{};
    void *p_uart{};
    // parse frame var
    ParseFrameState parse_frame_state;
    uint8_t parse_buf[256000]{};
    uint32_t parse_buf_cnt{};
    uint16_t tar_buf_Len=0;

    Channel<RadarData_general> obj_channel;
    Cnr200Config conf{};
    std::mutex conf_mtx{};
    std::thread *p_conf_thread{};
    std::atomic<uint32_t> obj_frame_acc{};
    std::atomic<uint32_t> cmd_resp_acc{};
    bool cnr200_status;

    void recv_proc();
    void deal_target_frame();
    void deal_target_frame_200radar();
    void deal_cmd_frame();
    int32_t deal_cmd_frame_check_info();

    int32_t send_cmd(Command *p_cmd);
    int32_t clear_response(Command *p_cmd);
    int32_t wait_response(Command *p_cmd, Command *p_resp, int32_t timeout_ms);
    int32_t config_proc();

    template<typename T> int32_t write_param(ParamName param_name, T val);
    template<typename T> int32_t read_param(ParamName param_name, T *p_val);

    static const std::map<ParamName, ParamInfo> param_map;
    static RadarObj frame_obj_to_radar_obj(const TargetFrameObj &frame_obj);
    static uint8_t target_frame_check(const void *data, uint32_t data_len);
    static uint8_t target_frame_check_general(const void *data, uint32_t data_len);
    static void swap_uint16(uint16_t *p_num);
    static void swap_uint32(uint32_t *p_num);
    static const int32_t WAIT_RESPONSE_TIME_MS = 3000;
    template<typename T>static std::string get_value_string(const T &val);
};


