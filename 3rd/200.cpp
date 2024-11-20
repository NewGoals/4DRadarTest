/*
 * Copyright (c) 2022-2022 湖南华诺星空电子技术有限公司. All rights reserved.
 */
#include <zlogger.h>
#include <cstring>
#include <common/common.h>
#include <common/misc_tool.h>
#include <hal/uart.h>
#include <boost/format.hpp>
#include "radar_cnr200.h"



ZLOGGER_HEADER_DEFINE("INPUT")

const std::map<RadarCNR200::ParamName, RadarCNR200::ParamInfo> RadarCNR200::param_map = {
    {PN_ROAD_CNT, {0x10, typeid(uint32_t).name(), 4}},
    {PN_ROAD_DIR, {0x14, typeid(uint32_t).name(), 4}},
    {PN_ROAD_WIDTH1, {0x18, typeid(float).name(), 4}},
    {PN_ROAD_WIDTH2, {0x1C, typeid(float).name(), 4}},
    {PN_ROAD_WIDTH3, {0x20, typeid(float).name(), 4}},
    {PN_ROAD_WIDTH4, {0x24, typeid(float).name(), 4}},
    {PN_ROAD_WIDTH5, {0x176, typeid(float).name(), 4}},
    {PN_ROAD_WIDTH6, {0x17A, typeid(float).name(), 4}},
    {PN_ROAD_WIDTH7, {0x17E, typeid(float).name(), 4}},
    {PN_ROAD_WIDTH8, {0x182, typeid(float).name(), 4}},
    {PN_ROAD_DIR1, {0x18A, typeid(Mr76sConfig::RoadDir).name(), 1}},
    {PN_ROAD_DIR2, {0x18B, typeid(Mr76sConfig::RoadDir).name(), 1}},
    {PN_ROAD_DIR3, {0x18C, typeid(Mr76sConfig::RoadDir).name(), 1}},
    {PN_ROAD_DIR4, {0x18D, typeid(Mr76sConfig::RoadDir).name(), 1}},
    {PN_ROAD_DIR5, {0x18E, typeid(Mr76sConfig::RoadDir).name(), 1}},
    {PN_ROAD_DIR6, {0x18F, typeid(Mr76sConfig::RoadDir).name(), 1}},
    {PN_ROAD_DIR7, {0x190, typeid(Mr76sConfig::RoadDir).name(), 1}},
    {PN_ROAD_DIR8, {0x191, typeid(Mr76sConfig::RoadDir).name(), 1}},
    {PN_HORIZON_SHIFT, {0x32, typeid(float).name(), 4}},
    {PN_ANGLE_RECOUP,     {0x38,  typeid(float).name(), 4}},
    {PN_DIST_LONG_RECOUP, {0x3C,  typeid(float).name(), 4}},
    {PN_INSTALL_HEIGHT,   {0x40,  typeid(float).name(), 4}},
    {PN_STOP_LINE_DIST,   {0x48,  typeid(float).name(), 4}},
    {PN_RADAR_FREQ_ID,    {0xBA,  typeid(uint8_t).name(), 1}},
    {PN_SENSE,            {0xD6,  typeid(float).name(), 4}},
    {PN_BIKE_PERSION,     {0x11E, typeid(Mr76sConfig::BikePersonSw).name(), 4}},
    {PN_AUTO_DISTINCTION, {0x122, typeid(int32_t).name(), 4}},
    {PN_STOP_OBJ_TIMEOUT, {0x126, typeid(uint32_t).name(), 4}},
    {PN_QUHUA_SWITCH,     {0x12A, typeid(Mr76sConfig::QuHuaSw).name(),  4}},
    {PN_QUHUA_ROAD_CNT,   {0x12E, typeid(uint32_t).name(), 4}},
    {PN_QUHUA_DIR,        {0x132, typeid(uint32_t).name(), 4}},
    {PN_QUHUA_ISOLATE,    {0x136, typeid(uint32_t).name(), 4}},
    {PN_QUHUA_WIDTH1,     {0x13A, typeid(float).name(), 4}},
    {PN_QUHUA_WIDTH2,     {0x13E, typeid(float).name(), 4}},
    {PN_QUHUA_WIDTH3,     {0x142, typeid(float).name(), 4}},
    {PN_QUHUA_WIDTH4,     {0x146, typeid(float).name(), 4}},
    {PN_QUHUA_WIDTH5,     {0x14A, typeid(float).name(), 4}},
    {PN_QUHUA_WIDTH6,     {0x14E, typeid(float).name(), 4}},
    {PN_QUHUA_WIDTH7,     {0x152, typeid(float).name(), 4}},
    {PN_QUHUA_WIDTH8,        {0x156, typeid(float).name(), 4}},
    {PN_QUHUA_GREEN_W,       {0x15A, typeid(float).name(), 4}},
    {PN_QUHUA_DIR1,          {0x15E, typeid(Mr76sConfig::RoadDir).name(), 1}},
    {PN_QUHUA_DIR2,          {0x15F, typeid(Mr76sConfig::RoadDir).name(), 1}},
    {PN_QUHUA_DIR3,          {0x160, typeid(Mr76sConfig::RoadDir).name(), 1}},
    {PN_QUHUA_DIR4,          {0x161, typeid(Mr76sConfig::RoadDir).name(), 1}},
    {PN_QUHUA_DIR5,          {0x162, typeid(Mr76sConfig::RoadDir).name(), 1}},
    {PN_QUHUA_DIR6,          {0x163, typeid(Mr76sConfig::RoadDir).name(), 1}},
    {PN_QUHUA_DIR7,          {0x164, typeid(Mr76sConfig::RoadDir).name(), 1}},
    {PN_QUHUA_DIR8,          {0x165, typeid(Mr76sConfig::RoadDir).name(), 1}},
    {PN_QUHUA_HORIZON_SHIFT, {0x166, typeid(float).name(), 4}},
    {PN_QUHUA_UP_EDGE,       {0x16A, typeid(float).name(), 4}},
    {PN_QUHUA_DOWN_EDGE,     {0x16E, typeid(float).name(), 4}},
    {PN_FAR_STOP_LINE,       {0x274, typeid(float).name(), 4}},
    {PN_ISOLATE_POSITION,    {0x172, typeid(uint32_t).name(), 4}},
    {PN_ISOLATE_WIDTH,       {0x186, typeid(float).name(), 4}},
    {PN_LEFT_OUT_DIST1,      {0x289, typeid(float).name(), 4}},
    {PN_LEFT_OUT_WIDTH1,     {0x28D, typeid(float).name(), 4}},
    {PN_LEFT_OUT_DIST2,      {0x291, typeid(float).name(), 4}},
    {PN_LEFT_OUT_WIDTH2,     {0x295, typeid(float).name(), 4}},
    {PN_MID_OUT_DIST1,       {0x299, typeid(float).name(), 4}},
    {PN_MID_OUT_WIDTH1, {0x29D, typeid(float).name(), 4}},
    {PN_MID_OUT_DIST2, {0x2A1, typeid(float).name(), 4}},
    {PN_MID_OUT_WIDTH2,    {0x2A5, typeid(float).name(), 4}},
    {PN_RIGHT_OUT_DIST1,   {0x2A9, typeid(float).name(), 4}},
    {PN_RIGHT_OUT_WIDTH1,  {0x2AD, typeid(float).name(), 4}},
    {PN_RIGHT_OUT_DIST2,   {0x2B1, typeid(float).name(), 4}},
    {PN_RIGHT_OUT_WIDTH2,  {0x2B5, typeid(float).name(), 4}},
    {PN_CROSS_UP_EDGE,     {0x2F9, typeid(float).name(), 4}},
    {PN_CROSS_DOWN_EDGE,   {0x2FD, typeid(float).name(), 4}},
    {PN_FAR_CROSS_WIDTH,   {0x301, typeid(float).name(), 4}},
    {PN_WORK_MODE,         {0xC2,  typeid(Mr76sConfig::WorkMode).name(), 4}},
    {PN_APPLY_FIELD,       {0xBE,  typeid(Mr76sConfig::ApplyField).name(), 4}},
    {PN_LEAVE_MID_MIN_LEN, {0x305, typeid(float).name(), 4}},
    {PN_LEAVE_BIG_MIN_LEN, {0x309, typeid(float).name(), 4}},
    {PN_COME_MID_MIN_LEN,  {0x30D, typeid(float).name(), 4}},
    {PN_COME_BIG_MIN_LEN,  {0x311, typeid(float).name(), 4}},
};


RadarCNR200::RadarCNR200():
    parse_frame_state(PFS_START)
{

}

RadarCNR200::~RadarCNR200() {
    DeleteUartObj(p_uart);
}


int32_t RadarCNR200::open() {
    if (init_ok) {
        ZWARN("WARN: RadarCNR200 %d is opened, close it before reopen\n");
        return 0;
    }
    if (!config_ok) {
        ZERROR("RadarCNR200 %d is not config_ok\n");
        return 0;
    }
    init_ok = false;
    cnr200_status = true;
    p_uart = NewUartObj((void*)conf.uart_dev.c_str(), (int32_t)this->conf.uart_dev.size());
    p_power_ctrl = new Gpio(conf.power_sw_gpio, 'O');

    if (0 != p_power_ctrl->open()) {
        ZERROR("FAIL: RadarCNR200 power_ctrl open\n");
        close();
        return -1;
    } else {
        ZINFO("GOOD: RadarCNR200 power_ctrl open\n");
    }

    if (0 != p_power_ctrl->set(1)) {
        ZERROR("FAIL: RadarCNR200 power_ctrl set 1\n");
        close();
        return -2;
    } else {
        ZINFO("GOOD: RadarCNR200 power_ctrl set 1\n");
    }

    if (0 != UartObjOpen(p_uart)) {
        ZERROR("FAIL: RadarCNR200 UartObjOpen %s\n", conf.uart_dev.c_str());
        close();
        return -3;
    } else {
        ZINFO("GOOD: RadarCNR200 UartObjOpen %s\n", conf.uart_dev.c_str());
    }

    if (0 != UartObjSet(p_uart, conf.uart_baudrate, 8, 'n', 1, 0, 20)) {
        ZERROR("FAIL: RadarCNR200 UartObjSet %s\n", conf.uart_dev.c_str());
        close();
        return -4;
    }

    exit_flag = false;
    p_recv_thread = new std::thread(&RadarCNR200::recv_proc, std::ref(*this));
    p_conf_thread = new std::thread(&RadarCNR200::config_proc, std::ref(*this));
    init_ok = true;
    return 0;
}


int32_t RadarCNR200::close_cut() {
    init_ok = false;
    exit_flag = true;

    if (p_recv_thread)
    {
        p_recv_thread->join();
        delete p_recv_thread;
        p_recv_thread = nullptr;
    }
    if (p_conf_thread)
    {
        p_conf_thread->join();
        delete p_conf_thread;
        p_conf_thread = nullptr;
    }
    if (p_uart)
    {
        UartObjClose(p_uart);
        DeleteUartObj(p_uart);
        p_uart = nullptr;
    }
    if (p_power_ctrl)
    {
        p_power_ctrl->set(0);
        delete p_power_ctrl;
        p_power_ctrl = nullptr;
    }
    return 0;
}

int32_t RadarCNR200::close() {
    init_ok = false;
    obj_channel.close();
    exit_flag = true;
    if (p_recv_thread) {
        p_recv_thread->join();
        delete p_recv_thread;
        p_recv_thread = nullptr;
    }
    if (p_conf_thread) {
        p_conf_thread->join();
        delete p_conf_thread;
        p_conf_thread = nullptr;
    }
    if (p_uart) {
        UartObjClose(p_uart);
        DeleteUartObj(p_uart);
        p_uart = nullptr;
    }
    if (p_power_ctrl) {
        p_power_ctrl->set(0);
        delete p_power_ctrl;
        p_power_ctrl = nullptr;
    }
    return 0;
}

void RadarCNR200::recv_proc() {
    set_this_thread_name("radar_recv");
    uint8_t read_buf[4096] = {};

    while (!exit_flag) {
        int32_t ret_uart_recv = UartObjRead(p_uart, read_buf, sizeof(read_buf));
        if (ret_uart_recv <= 0)
        {
            cnr200_status = false;
            ZWARN("WARN: UartObjRead %s, ret_uart_recv %d\n", conf.uart_dev.c_str(), ret_uart_recv);
            continue;
        }
        else {
            cnr200_status = true;
/*            ZDEBUG("\n11111 uart read_buf len = %d \n",ret_uart_recv);
            for (int i=0; i<ret_uart_recv; i++) {
                printf("0x%02x ",read_buf[i]);
            }
            printf("\r\n");*/
        }

        for (int i=0; i<ret_uart_recv; i++)
        {
            uint8_t cur_byte = read_buf[i];
            switch (parse_frame_state)
            {
                case PFS_START:
                {
                    parse_buf[parse_buf_cnt++] = cur_byte;
                    if (parse_buf_cnt == 2)
                    {
                        if (0xA5 == parse_buf[0] && 0x5A == parse_buf[1])
                        {
                            parse_frame_state = PFS_CMD_FRAME_HEADER;
                        }
                    }
                } break;

                case PFS_TARGET_FRAME_DATA_LEN:
                {
                    auto *p_target_frame = (TargetFrame*)parse_buf;
                    parse_buf[parse_buf_cnt++] = cur_byte;
                    if (parse_buf_cnt >= 6) {
                        // swap_uint16(&p_target_frame->data_len);
                        p_target_frame->data_len = (parse_buf[6] << 8) | (parse_buf[5]);
                        if (p_target_frame->data_len + 7u <= sizeof(parse_buf))
                        {
                            parse_frame_state = PFS_TARGET_FRAME_DATA;
                            //ZDEBUG("\np_target_frame->data_len %d\n", p_target_frame->data_len);
                        }
                        else
                        {
                            ZERROR("FAIL: PFS_TARGET_FRAME_DATA_LEN data_len too big %d\n", p_target_frame->data_len);
                            parse_frame_state = PFS_START;
                            parse_buf[0] = 0;
                            parse_buf[1] = 0;
                            parse_buf_cnt = 0;
                        }
                    }
                } break;

                case PFS_TARGET_FRAME_DATA: {

                    auto *p_target_frame = (TargetFrame_general*)parse_buf;
                    parse_buf[parse_buf_cnt++] = cur_byte;
                    if(parse_buf_cnt>6)
                    {
                        if (parse_buf_cnt < sizeof(parse_buf))
                        {
                            tar_buf_Len = 7u + p_target_frame->data_len + 1;
                            if (parse_buf_cnt >= 7u + p_target_frame->data_len+1)
                            {
                                deal_target_frame();
                                parse_frame_state = PFS_START;
                                parse_buf[0] = 0;
                                parse_buf[1] = 0;
                                parse_buf_cnt = 0;
                                tar_buf_Len = 0;
                            }
                        }
                        else
                        {
                            ZERROR("FAIL: PFS_TARGET_FRAME_DATA recv too many data, parse_buf_cnt %d\n", parse_buf_cnt);
                            parse_frame_state = PFS_START;
                            parse_buf[0] = 0;
                            parse_buf[1] = 0;
                            parse_buf_cnt = 0;
                        }
                    }
                } break;

                case PFS_CMD_FRAME_HEADER:
                {
                    parse_buf[parse_buf_cnt++] = cur_byte;
                    if (parse_buf_cnt >= 4) {
                        //ZDEBUG("parse_buf[2] = 0x%02x , parse_buf[3] = 0x%02x\r\n",parse_buf[2],parse_buf[3]);
                        if (0x90 == parse_buf[2])
                        {
                            parse_frame_state = PFS_CMD_FRAME_INFO;
                        }
                        else
                        {
                            parse_frame_state = PFS_START;
                            parse_buf[0] = 0;
                            parse_buf[1] = 0;
                            parse_buf_cnt = 0;
                        }
                    }
                } break;

                case PFS_CMD_FRAME_INFO: {

                    parse_buf[parse_buf_cnt++] = cur_byte;
                    if (parse_buf_cnt >= 5)
                    {
                        if(parse_buf[4] == 0xA8) //目标信息
                        {
                            parse_frame_state = PFS_TARGET_FRAME_DATA;
                        }
                        else
                        {
                            //执行其他的指令信息
                        }
                    }
                } break;

                case PFS_CMD_FRAME_PARAMS: {
                    auto *p_cmd_frame = (CmdFrame*)parse_buf;
                    if (parse_buf_cnt < sizeof(parse_buf)) {
                        parse_buf[parse_buf_cnt++] = cur_byte;
                        if (parse_buf_cnt >= sizeof(CmdFrame) + p_cmd_frame->prm_size) {
                            deal_cmd_frame();
                            parse_frame_state = PFS_START;
                            parse_buf[0] = 0;
                            parse_buf[1] = 0;
                            parse_buf_cnt = 0;
                        }
                    } else {
                        ZERROR("FAIL: PFS_CMD_FRAME_PARAMS recv too many data, parse_buf_cnt %d\n", parse_buf_cnt);
                        parse_frame_state = PFS_START;
                        parse_buf[0] = 0;
                        parse_buf[1] = 0;
                        parse_buf_cnt = 0;
                    }
                } break;

                default: {
                    ZERROR("FAIL: unknown state %d\n", parse_frame_state);
                } break;
            }
        }

        if ((parse_buf_cnt < tar_buf_Len) && (tar_buf_Len != 0))
        {
            //下一帧开始补齐
        }
        else
        {
            parse_frame_state = PFS_START;
            parse_buf_cnt = 0;
            tar_buf_Len = 0;
        }

    }
}

void RadarCNR200::deal_target_frame()
{
    auto *p_target_frame = (TargetFrame_general *) parse_buf;
    RadarData_general radar_data;

    radar_data.cnr200_mudslides = p_target_frame->cnr200_mudslides;

    if(radar_data.cnr200_mudslides.debrisFlowWidth == 0 && radar_data.cnr200_mudslides.debrisFlowHeight == 0 && radar_data.cnr200_mudslides.flowVelocity == 0
        && radar_data.cnr200_mudslides.centerDistance == 0 &&radar_data.cnr200_mudslides.centerAngle == 0 && radar_data.cnr200_mudslides.targetNum == 0)
    {
        //ZERROR("FAIL: deal_target_frame data is invalid\n");
        return;
    }

    if (radar_data.cnr200_mudslides.targetNum * 68 + 48 <= p_target_frame->data_len)
    {
        for (int32_t i = 0; i < radar_data.cnr200_mudslides.targetNum; i++)
        {
            //radar_data.cnr200_mudslides.radarobjs.push_back(radar_obj);
/*            ZDEBUG("ZDEBUG,p_target_frame->objs[obj_index].id = %d\r\n", p_target_frame->objs[i].id);
            ZDEBUG("ZDEBUG,p_target_frame->objs[obj_index].type = %d\r\n", p_target_frame->objs[i].type);
            ZDEBUG("ZDEBUG,p_target_frame->objs[obj_index].length = %.4f\r\n", p_target_frame->objs[i].length);
            ZDEBUG("ZDEBUG,p_target_frame->objs[obj_index].azimuth_angle = %.4f\r\n", p_target_frame->objs[i].azimuth_angle);
            ZDEBUG("ZDEBUG,p_target_frame->objs[obj_index].elevation_angle = %.4f\r\n", p_target_frame->objs[i].elevation_angle);*/
            radar_data.radarobjs.push_back(p_target_frame->objs[i]);
        }
    }
    else
    {
        return;
    }
    obj_channel.push_force(radar_data);
}

uint8_t RadarCNR200::target_frame_check(const void *data, uint32_t data_len) {
    uint8_t sum = 0;
    for (uint32_t i=0; i<data_len; i++) {
        sum += ((uint8_t*)data)[i];
    }
    return sum;
}

uint8_t RadarCNR200::target_frame_check_general(const void *data, uint32_t data_len) {
    uint8_t sum = 0;
    for (uint32_t i=2; i<data_len+7+1-1; i++) {
        sum += ((uint8_t*)data)[i];
    }
    return sum;
}

void RadarCNR200::swap_uint16(uint16_t *p_num) {
    uint8_t tmp = ((uint8_t*)p_num)[0];
    ((uint8_t*)p_num)[0] = ((uint8_t*)p_num)[1];
    ((uint8_t*)p_num)[1] = tmp;
}

void RadarCNR200::swap_uint32(uint32_t *p_num) {
    uint8_t tmp = ((uint8_t*)p_num)[0];
    ((uint8_t*)p_num)[0] = ((uint8_t*)p_num)[3];
    ((uint8_t*)p_num)[3] = tmp;

    tmp = ((uint8_t*)p_num)[1];
    ((uint8_t*)p_num)[1] = ((uint8_t*)p_num)[2];
    ((uint8_t*)p_num)[2] = tmp;
}

int32_t RadarCNR200::deal_cmd_frame_check_info() {
    auto *p_cmd_frame = (CmdFrame*)parse_buf;
    //HZDEBUG(parse_buf, parse_buf_cnt);

    // check cmd and prmsize
    if (0 == strncmp(p_cmd_frame->command, CMD_CODE_BURN, sizeof(p_cmd_frame->command))) {
        if (0 != p_cmd_frame->prm_size) {
            ZERROR("FAIL: deal_cmd_frame_check_info, %s prm_size[%d] != 0\n", p_cmd_frame->command, p_cmd_frame->prm_size);
            return -1;
        }
    } else if (0 == strncmp(p_cmd_frame->command, CMD_PARAM_WRITE, sizeof(p_cmd_frame->command))) {
        if (0 != p_cmd_frame->prm_size) {
            ZERROR("FAIL: deal_cmd_frame_check_info, %s prm_size[%d] != 0\n", p_cmd_frame->command, p_cmd_frame->prm_size);
            return -2;
        }
    } else if (0 == strncmp(p_cmd_frame->command, CMD_PARAM_READ, sizeof(p_cmd_frame->command))) {
        if (p_cmd_frame->prm_size != 1 && p_cmd_frame->prm_size != 4) {
            ZERROR("FAIL: deal_cmd_frame_check_info, %s prm_size[%d] < 1\n", p_cmd_frame->command, p_cmd_frame->prm_size);
            return -3;
        }
    } else if (0 == strncmp(p_cmd_frame->command, CMD_PARAM_GET, sizeof(p_cmd_frame->command))) {
        if (p_cmd_frame->prm_size < 1 || p_cmd_frame->prm_size > 10) {
            ZERROR("FAIL: deal_cmd_frame_check_info, %s prm_size[%d] <1 or >10\n", p_cmd_frame->command, p_cmd_frame->prm_size);
            return -4;
        }
    } else {
        p_cmd_frame->command[sizeof(p_cmd_frame->command)-1] = '\0';
        ZERROR("FAIL: deal_cmd_frame_check_info, unknown cmd %s\n", p_cmd_frame->command);
        return -5;
    }
    return 0;
}

void RadarCNR200::deal_cmd_frame() {
    auto *p_cmd_frame = (CmdFrame*)parse_buf;
    //HZDEBUG(parse_buf, parse_buf_cnt);
    Command cmd = {};
    cmd.command = p_cmd_frame->command;
    cmd.flags = p_cmd_frame->flags;
    cmd.return_value = p_cmd_frame->return_value;
    for (uint32_t i=0; i<p_cmd_frame->prm_size; i++) {
        cmd.params.push_back(p_cmd_frame->params[i]);
    }
    cmd_resp_pool_mtx.lock();
    cmd_resp_pool[cmd.command] = cmd;
    cmd_resp_pool_mtx.unlock();
    cmd_resp_acc++;
}


RadarObj RadarCNR200::frame_obj_to_radar_obj(const TargetFrameObj &frame_obj) {
    RadarObj obj = {};
    obj.id = frame_obj.num;
    obj.obj_type = frame_obj.obj_type;
    obj.road_num = frame_obj.road_num;
    obj.dist_long = (float)frame_obj.dist_long / 10;
    obj.dist_lat = (float)frame_obj.dist_lat / 10;
    obj.v_long = (float)frame_obj.v_long / 10;
    obj.v_lat = (float)frame_obj.v_lat / 10;
    return obj;
}


int32_t RadarCNR200::get_obj_block(RadarData_general *p_data)
{
    return obj_channel.pop_wait(p_data);
}


void RadarCNR200::show_target_vec(std::vector<RadarObj_general> *p_obj_vec) {
    ZINFO("target count: %ld\n", p_obj_vec->size());
    for (auto &t : *p_obj_vec) {
        ZINFO("ID %d, type %d, x_speed %.2f, length %.2f, azimuth_angle %.2f, elevation_angle %.2f\n",
              t.id, t.type, t.x_speed, t.length, t.azimuth_angle, t.elevation_angle);
    }
}

int32_t RadarCNR200::send_cmd(Command *p_cmd) {
    CmdFrame cmd_frame = {};
    cmd_frame.header = 0x1234ABCD;
    strncpy(cmd_frame.command, p_cmd->command.c_str(), sizeof(cmd_frame.command));
    if (p_cmd->command == std::string(CMD_CODE_BURN)) {
        // todo return_value is crc of params
        cmd_frame.return_value = 0;
    } else {
        cmd_frame.return_value = 0;
    }
    cmd_frame.flags = 0;
    cmd_frame.prm_size = p_cmd->params.size();

    clear_response(p_cmd);  // clear response before uart write
    auto ret = UartObjWrite(p_uart, &cmd_frame, sizeof(cmd_frame));
    if (ret != sizeof(cmd_frame)) {
        ZERROR("FAIL: RadarCNR200 UartObjWrite, ret %d\n", ret);
        return -1;
    }
    ret = UartObjWrite(p_uart, p_cmd->params.data(), (int32_t)p_cmd->params.size());
    if (ret != (int32_t)p_cmd->params.size()) {
        ZERROR("FAIL: RadarCNR200 UartObjWrite, ret %d\n", ret);
        return -2;
    }
    return 0;
}

int32_t RadarCNR200::clear_response(Command *p_cmd) {
    cmd_resp_pool_mtx.lock();
    auto iter = cmd_resp_pool.find(p_cmd->command);
    if (iter != cmd_resp_pool.end()) {
        cmd_resp_pool.erase(iter);
    }
    cmd_resp_pool_mtx.unlock();
    return 0;
}

int32_t RadarCNR200::wait_response(Command *p_cmd, Command *p_resp, int32_t timeout_ms) {
    int32_t timer_ms = 0;
    const int POLL_INTERVAL_MS =50;
    while (timer_ms < timeout_ms && !exit_flag) {
        cmd_resp_pool_mtx.lock();
        auto iter = cmd_resp_pool.find(p_cmd->command);
        if (iter != cmd_resp_pool.end()) {
            *p_resp = iter->second;
            cmd_resp_pool.erase(iter);
            cmd_resp_pool_mtx.unlock();
            return 0;
        } else {
            cmd_resp_pool_mtx.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(POLL_INTERVAL_MS));
            timer_ms += POLL_INTERVAL_MS;
        }
    }
    return -1;
}


template<typename T> int32_t RadarCNR200::write_param(ParamName param_name, T val) {
    auto iter = param_map.find(param_name);
    if (iter == param_map.end()) {
        ZERROR("FAIL: RadarCNR200 write_param, param_name %d not exist\n", param_name);
        return -1;
    }
    ParamInfo param_info = iter->second;
    if (0 != strcmp(param_info.type_name, typeid(T).name())) {
        ZERROR("FAIL: RadarCNR200 write_param, type_name %s != %s\n", param_info.type_name, typeid(T).name());
        return -2;
    }
    if (sizeof(T) != param_info.size) {
        ZERROR("FAIL: RadarCNR200 write_param, sizeof(%s):%ld != param_info.size:%d\n",
               typeid(T).name(), sizeof(T), param_info.size);
        return -2;
    }
    Command cmd = {};
    cmd.command = CMD_PARAM_WRITE;
    cmd.return_value = 0;
    cmd.flags = 0;
    cmd.params.insert(cmd.params.end(), (uint8_t*)&param_info.addr, (uint8_t*)&(&param_info.addr)[1]);
    uint32_t content_size = sizeof(T);
    cmd.params.insert(cmd.params.end(), (uint8_t*)&content_size, (uint8_t*)&(&content_size)[1]);
    cmd.params.insert(cmd.params.end(), (uint8_t*)&val, (uint8_t*)&(&val)[1]);
    bool resp_ok = false;
    for (int32_t try_cnt=0; try_cnt<MAX_TRY_CNT; try_cnt++) {
        auto ret = send_cmd(&cmd);
        if (0 != ret) {
            ZERROR("FAIL: RadarCNR200 write_param send_cmd, ret %d\n", ret);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        Command resp = {};
        ret = wait_response(&cmd, &resp, WAIT_RESPONSE_TIME_MS);
        if (0 != ret) {
            ZERROR("FAIL: RadarCNR200 write_param wait_response, ret %d\n", ret);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        if (resp.return_value != 0) {
            ZERROR("FAIL: RadarCNR200 write_param return_value %d\n", resp.return_value);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        resp_ok = true;
        break;
    }
    if (resp_ok) {
        return 0;
    } else {
        return -1;
    }
}


template<typename T> int32_t RadarCNR200::read_param(ParamName param_name, T *p_val) {
    auto iter = param_map.find(param_name);
    if (iter == param_map.end()) {
        ZERROR("FAIL: RadarCNR200 read_param, param_name %d not exist\n", param_name);
        return -1;
    }
    ParamInfo param_info = iter->second;
    if (0 != strcmp(param_info.type_name, typeid(T).name())) {
        ZERROR("FAIL: RadarCNR200 read_param, type_name %s != %s\n", param_info.type_name, typeid(T).name());
        return -2;
    }
    Command cmd = {};
    cmd.command = CMD_PARAM_READ;
    cmd.return_value = 0;
    cmd.flags = 0;
    cmd.params.insert(cmd.params.end(), (uint8_t*)&param_info.addr, (uint8_t*)&(&param_info.addr)[1]);
    uint32_t content_size = sizeof(T);
    cmd.params.insert(cmd.params.end(), (uint8_t*)&content_size, (uint8_t*)&(&content_size)[1]);
    auto ret = send_cmd(&cmd);
    if (0 != ret) {
        ZERROR("FAIL: RadarCNR200 read_param send_cmd, ret %d\n", ret);
        return -3;
    }
    Command resp = {};
    ret = wait_response(&cmd, &resp, WAIT_RESPONSE_TIME_MS);
    if (0 != ret) {
        ZERROR("FAIL: RadarCNR200 read_param wait_response, ret %d\n", ret);
        return -4;
    }
    if (resp.return_value != 0) {
        ZERROR("FAIL: RadarCNR200 read_param return_value %d\n", resp.return_value);
        return -5;
    }
    if (resp.params.size() != sizeof(T)) {
        ZERROR("FAIL: RadarCNR200 read_param params.size %ld != %ld\n", resp.params.size(), sizeof(T));
        return -5;
    }
    *p_val = *(T*)resp.params.data();
    return 0;
}


int32_t RadarCNR200::get_version(std::string *p_version) {
    Command cmd = {};
    cmd.command = CMD_PARAM_GET;
    cmd.return_value = 0;
    cmd.flags = 0;
    cmd.params.clear();
    auto ret = send_cmd(&cmd);
    if (0 != ret) {
        ZERROR("FAIL: RadarCNR200 get_version send_cmd, ret %d\n", ret);
        return -3;
    }
    Command resp = {};
    ret = wait_response(&cmd, &resp, WAIT_RESPONSE_TIME_MS);
    if (0 != ret) {
        ZERROR("FAIL: RadarCNR200 get_version wait_response, ret %d\n", ret);
        return -4;
    }
    if (resp.return_value != 0) {
        ZERROR("FAIL: RadarCNR200 get_version return_value %d\n", resp.return_value);
        return -5;
    }
    *p_version = std::string(resp.params.rbegin(), resp.params.rend());
    *p_version += '\0';
    return 0;
}


bool RadarCNR200::get_radar_status()
{
    return cnr200_status;
}


int32_t RadarCNR200::set_apply_field(const std::string& apply_field) {
    if (apply_field == "园区测速") {
        return write_param(PN_APPLY_FIELD, Mr76sConfig::AF_PARK_SPEED);
    } if (apply_field == "路段场景") {
        return write_param(PN_APPLY_FIELD, Mr76sConfig::AF_ROAD_SEG);
    } else {
        ZERROR("FAIL: RadarCNR200 set_apply_field unknown apply field %s\n", apply_field.c_str());
        return -1;
    }
}


int32_t RadarCNR200::get_apply_field(std::string *p_apply_field) {
    Mr76sConfig::ApplyField apply_field_code;
    auto ret = read_param(PN_APPLY_FIELD, &apply_field_code);
    if (0 != ret) {
        ZERROR("FAIL: RadarCNR200 get_apply_field, ret %d\n", ret);
        return -1;
    }
    if (Mr76sConfig::AF_PARK_SPEED == apply_field_code) {
        *p_apply_field = "园区测速";
        return 0;
    } if (Mr76sConfig::AF_ROAD_SEG == apply_field_code) {
        *p_apply_field = "路段场景";
        return 0;
    } else {
        ZERROR("FAIL: RadarCNR200 get_apply_field unknown apply field code 0x%x\n", apply_field_code);
        return -2;
    }
}


int32_t RadarCNR200::set_work_mode(bool debug_mode) {
    if (debug_mode) {
        return write_param(PN_WORK_MODE, Mr76sConfig::WM_DEBUG);
    } else {
        return write_param(PN_WORK_MODE, Mr76sConfig::WM_NORMAL);
    }
}


int32_t RadarCNR200::set_road_cnt(uint32_t road_cnt) {
    return write_param(PN_ROAD_CNT, road_cnt);
}

#define RadarCNR200WriteParam(name, value) { \
    if (write_param(name, value)) { \
        ZERROR("FAIL: RadarCNR200::config "#name" to %s\n", get_value_string(value).c_str()); \
        return -1; \
    } else { \
        ZINFO("GOOD: RadarCNR200::config "#name" to %s\n", get_value_string(value).c_str()); \
    }\
}

#define RadarCNR200WriteParamArr(name, value) { \
    for (size_t i=0; i<ARR_SIZE(value); i++) { \
        if (write_param(ParamName(name+i), value[i])) { \
            ZERROR("FAIL: RadarCNR200::config "#name"+%ld to %s\n", i, get_value_string(value[i]).c_str()); \
            return -1; \
        } else { \
            ZINFO("GOOD: RadarCNR200::config "#name"+%ld to %s\n", i, get_value_string(value[i]).c_str()); \
        } \
    } \
}

int32_t RadarCNR200::config(Cnr200Config *p_conf) {
    // todo check config
    conf_mtx.lock();
    conf = *p_conf;
    conf_mtx.unlock();
    config_ok = true;
    return 0;
}

int32_t RadarCNR200::config_proc() {
    set_this_thread_name("radar_config");
    // wait radar start
    int32_t time_wait_ms = 0;
    while (obj_frame_acc <= 0 && time_wait_ms < TIME_WAIT_START_MS) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        time_wait_ms += 100;
    }
    if (time_wait_ms >= TIME_WAIT_START_MS) {
        // 线程中不能执行close操作，因为close操作会等待线程结束并join线程，在线程中join本线程会报错：
        // std::system_error Resource deadlock avoided
        ZERROR("FAIL: wait RadarCNR200 start, wait %dms\n", time_wait_ms);
        return -5;
    } else {
        ZINFO("GOOD: wait RadarCNR200 start, wait %dms\n", time_wait_ms);
    }

    while (!exit_flag) {
        // todo recv config request and do config
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}

template<typename T>
std::string RadarCNR200::get_value_string(const T &val)
{
    if (typeid(T) == typeid(uint32_t))
    {
        return (boost::format("0x%08x") % val).str();
    }
    else if (typeid(T) == typeid(uint8_t))
    {
        char str[16] = {0};
        sprintf(str, "0x%02x", (uint8_t) val);
        return str;
    }
    else if (typeid(T) == typeid(float))
    {
        return (boost::format("%.6f") % val).str();
    }
    else if (typeid(T) == typeid(Mr76sConfig::RoadDir))
    {
        return (boost::format("0x%02x") % val).str();
    }
    else if (typeid(T) == typeid(Mr76sConfig::WorkMode))
    {
        return (boost::format("0x%08x") % val).str();
    }
    else if (typeid(T) == typeid(Mr76sConfig::ApplyField))
    {
        return (boost::format("0x%08x") % val).str();
    }
    else if (typeid(T) == typeid(Mr76sConfig::BikePersonSw))
    {
        return (boost::format("0x%08x") % val).str();
    }
    else if (typeid(T) == typeid(Mr76sConfig::QuHuaSw))
    {
        return (boost::format("0x%08x") % val).str();
    }
    else
    {
        return "unknown";
    }
}
