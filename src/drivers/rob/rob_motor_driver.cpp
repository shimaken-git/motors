// RobMotorDriver.cpp
#include "rob_motor_driver.hpp"

ROB_Limit_Param limit_param[ROB_Num_Of_Model] = {
    {12.5, 33, 14, 500, 5},   // RS00_48V   operation control mode
    {12.5, 33, 60, 5000, 100},  // RS03_48V   operation control mode
    {12.5, 20, 36, 5000, 100},  // RS06_48V   operation control mode
};

RobMotorDriver::RobMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                             ROB_Motor_Model motor_model, double motor_zero_offset)
    : MotorDriver(), can_(MotorsSocketCAN::get(can_interface)), motor_model_(motor_model) {
    if (interface_type != "can") {
        throw std::runtime_error("ROB driver only support CAN interface");
    }
    motor_id_ = motor_id;
    // master_id_ = motor_id_ + master_id_offset;
    limit_param_ = limit_param[motor_model_];
    can_interface_ = can_interface;
    motor_zero_offset_ = motor_zero_offset;
    CanCbkFunc can_callback = std::bind(&RobMotorDriver::can_rx_cbk, this, std::placeholders::_1);
    can_->add_can_callback(can_callback, master_id_);
}

RobMotorDriver::~RobMotorDriver() { can_->remove_can_callback(master_id_); }

void RobMotorDriver::lock_motor() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_ | CAN_EFF_FLAG | ROB_CMD::ROB_ENABLE;  // change according to the mode   extended frame compatible
    tx_frame.can_dlc = 0x08;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void RobMotorDriver::unlock_motor() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_ | CAN_EFF_FLAG | ROB_CMD::ROB_DISABLE;  // change according to the mode   extended frame compatible
    tx_frame.can_dlc = 0x08;

    // tx_frame.data[0] = 0x01;   // モーターにエラーがある場合、エラーをクリアする。
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

uint8_t RobMotorDriver::init_motor() {
    // send disable command to enter read mode
    RobMotorDriver::unlock_motor();
    Timer::sleep_for(normal_sleep_time);
    // set_motor_control_mode(MIT);
    Timer::sleep_for(normal_sleep_time);
    // send enable command to enter contorl mode
    RobMotorDriver::lock_motor();
    Timer::sleep_for(normal_sleep_time);
    RobMotorDriver::refresh_motor_status();
    Timer::sleep_for(normal_sleep_time);
    return error_id_ >> 16;   // ROBErrorFlag -> ROBError
}

void RobMotorDriver::deinit_motor() {
    RobMotorDriver::unlock_motor();
    Timer::sleep_for(normal_sleep_time);
}

bool RobMotorDriver::write_motor_flash() { return true; }

bool RobMotorDriver::set_motor_zero() {
    // send set zero command
    RobMotorDriver::set_motor_zero_rob();
    Timer::sleep_for(setup_sleep_time);
    RobMotorDriver::refresh_motor_status();
    Timer::sleep_for(setup_sleep_time);  // wait for motor to set zero
    logger_->info("motor_id: {0}\tposition: {1}\t", motor_id_, get_motor_pos());
    RobMotorDriver::unlock_motor();
    if (get_motor_pos() > judgment_accuracy_threshold || get_motor_pos() < -judgment_accuracy_threshold) {
        logger_->warn("set zero error");
        return false;
    } else {
        logger_->info("set zero success");
        return true;
    }
    // disable motor
}

void RobMotorDriver::can_rx_cbk(const can_frame& rx_frame) {
    {
        response_count_ = 0;
    }
    uint32_t extra_data;    // extended frame compatible
    uint16_t host_id_t = 0;
    uint16_t device_id_t = 0;
    uint16_t pos_int = 0;
    uint16_t spd_int = 0;
    uint16_t t_int = 0;
    pos_int = rx_frame.data[0] << 8 | rx_frame.data[1];
    spd_int = rx_frame.data[2] << 8 | rx_frame.data[3];
    t_int = rx_frame.data[4] << 8 | rx_frame.data[5];
    extra_data = rx_frame.can_id;
    host_id_t = extra_data & 0xff;
    device_id_t = (extra_data & 0x0000ff00) >> 8;
    uint16_t err = (extra_data & 0x00ff0000) >> 16;
    if (err) {  // error flag is valid
        error_id_ = err;
            if (logger_) {
                logger_->error("can_interface: {0}\tmotor_id: {1}\terror_id: 0x{2:x}", can_interface_, motor_id_, (uint32_t)error_id_);
            }
        }
    motor_pos_ =
        range_map(pos_int, uint16_t(0), bitmax<uint16_t>(16), -limit_param_.PosMax, limit_param_.PosMax) + motor_zero_offset_;
    motor_spd_ =
        range_map(spd_int, uint16_t(0), bitmax<uint16_t>(16), -limit_param_.SpdMax, limit_param_.SpdMax);
    motor_current_ =
        range_map(t_int, uint16_t(0), bitmax<uint16_t>(16), -limit_param_.TauMax, limit_param_.TauMax);
    motor_temperature_ = (rx_frame.data[6] << 8 | rx_frame.data[7]) * 0.1;
}

void RobMotorDriver::get_motor_param(uint8_t param_cmd) {
    can_frame tx_frame;
    tx_frame.can_id = 0x7FF | CAN_EFF_FLAG;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = motor_id_ & 0xFF;
    tx_frame.data[1] = motor_id_ >> 8;
    tx_frame.data[2] = 0x33;
    tx_frame.data[3] = param_cmd;

    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFF;
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void RobMotorDriver::motor_pos_cmd(float pos, float spd, bool ignore_limit) {
    std::cout << " RobMotorDriver::motor_pos_cmd() Not implemented" << std::endl;
    // if (motor_control_mode_ != POS) {
    //     set_motor_control_mode(POS);
    //     return;
    // }
    // can_frame tx_frame;
    // tx_frame.can_id = (0x100 + motor_id_) | CAN_EFF_FLAG;
    // tx_frame.can_dlc = 0x08;
    // uint8_t *pbuf, *vbuf;

    // pos -= motor_zero_offset_;
    // spd = limit(spd, -limit_param_.SpdMax, limit_param_.SpdMax);
    // pos = limit(pos, -limit_param_.PosMax, limit_param_.PosMax);

    // pbuf = (uint8_t*)&pos;
    // vbuf = (uint8_t*)&spd;

    // tx_frame.data[0] = *pbuf;
    // tx_frame.data[1] = *(pbuf + 1);
    // tx_frame.data[2] = *(pbuf + 2);
    // tx_frame.data[3] = *(pbuf + 3);
    // tx_frame.data[4] = *vbuf;
    // tx_frame.data[5] = *(vbuf + 1);
    // tx_frame.data[6] = *(vbuf + 2);
    // tx_frame.data[7] = *(vbuf + 3);

    // can_->transmit(tx_frame);
    // {
    //     response_count_++;
    // }
}

void RobMotorDriver::motor_spd_cmd(float spd) {
    std::cout << " RobMotorDriver::motor_spd_cmd() Not implemented" << std::endl;
    // if (motor_control_mode_ != SPD) {
    //     set_motor_control_mode(SPD);
    //     return;
    // }
    // can_frame tx_frame;
    // tx_frame.can_id = (0x200 + motor_id_) | CAN_EFF_FLAG;
    // tx_frame.can_dlc = 0x04;

    // spd = limit(spd, -limit_param_.SpdMax, limit_param_.SpdMax);
    // union32_t rv_type_convert;
    // rv_type_convert.f = spd;
    // tx_frame.data[0] = rv_type_convert.buf[0];
    // tx_frame.data[1] = rv_type_convert.buf[1];
    // tx_frame.data[2] = rv_type_convert.buf[2];
    // tx_frame.data[3] = rv_type_convert.buf[3];

    // can_->transmit(tx_frame);
    // {
    //     response_count_++;
    // }
}

// Transmit MIT-mDme control(hybrid) package. Called in canTask.
void RobMotorDriver::motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
    // if (motor_control_mode_ != MIT) {
    //     set_motor_control_mode(MIT);
    //     return;
    // }
    uint16_t p, v, kp, kd, t;
    can_frame tx_frame;

    f_p -= motor_zero_offset_;
    f_p = limit(f_p, -limit_param_.PosMax, limit_param_.PosMax);
    f_v = limit(f_v, -limit_param_.SpdMax, limit_param_.SpdMax);
    f_kp = limit(f_kp, 0.0f, limit_param_.OKpMax);
    f_kd = limit(f_kd, 0.0f, limit_param_.OKdMax);
    f_t = limit(f_t, -limit_param_.TauMax, limit_param_.TauMax);

    p = range_map(f_p, -limit_param_.PosMax, limit_param_.PosMax, uint16_t(0), bitmax<uint16_t>(16));
    v = range_map(f_v, -limit_param_.SpdMax, limit_param_.SpdMax, uint16_t(0), bitmax<uint16_t>(16));
    kp = range_map(f_kp, 0.0f, limit_param_.OKpMax, uint16_t(0), bitmax<uint16_t>(16));
    kd = range_map(f_kd, 0.0f, limit_param_.OKdMax, uint16_t(0), bitmax<uint16_t>(16));
    t = range_map(f_t, -limit_param_.TauMax, limit_param_.TauMax, uint16_t(0), bitmax<uint16_t>(16));

    tx_frame.can_id = motor_id_ | CAN_EFF_FLAG | t | 0x1000000;   // extended frame compatible
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = p >> 8;
    tx_frame.data[1] = p & 0xFF;
    tx_frame.data[2] = v >> 8;
    tx_frame.data[3] = v & 0xFF;
    tx_frame.data[4] = kp >> 8;
    tx_frame.data[5] = kp & 0xFF;
    tx_frame.data[6] = kd >> 8;
    tx_frame.data[7] = kd & 0xFF;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void RobMotorDriver::set_motor_control_mode(uint8_t motor_control_mode) {
    // write_register_rob(10, motor_control_mode);
    motor_control_mode_ = motor_control_mode;
}

void RobMotorDriver::set_motor_zero_rob() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_ | CAN_EFF_FLAG | ROB_CMD::ROB_ZEROPOS;  // change according to the mode  extended frame compatible
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = 0x01;
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void RobMotorDriver::clear_motor_error_rob() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_ | CAN_EFF_FLAG;  // change according to the mode  extended frame compatible
    tx_frame.can_dlc = 0x08;
}

void RobMotorDriver::write_register_rob(uint16_t rid, float value) {
    // param_cmd_flag_[rid] = false;
    can_frame tx_frame;
    tx_frame.can_id = motor_id_ | CAN_EFF_FLAG | ROB_CMD::ROB_WRITEPRM;
    tx_frame.can_dlc = 0x08;

    uint8_t* vbuf;
    vbuf = (uint8_t*)&value;

    tx_frame.data[0] = rid >> 8;
    tx_frame.data[1] = rid & 0xFF;
    tx_frame.data[2] = 0x00;
    tx_frame.data[3] = 0x00;

    tx_frame.data[4] = *vbuf;
    tx_frame.data[5] = *(vbuf + 1);
    tx_frame.data[6] = *(vbuf + 2);
    tx_frame.data[7] = *(vbuf + 3);
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void RobMotorDriver::write_register_rob(uint16_t rid, int32_t value) {
    // param_cmd_flag_[rid] = false;
    can_frame tx_frame;
    tx_frame.can_id = motor_id_ | CAN_EFF_FLAG | ROB_CMD::ROB_WRITEPRM;
    tx_frame.can_dlc = 0x08;

    uint8_t* vbuf;
    vbuf = (uint8_t*)&value;

    tx_frame.data[0] = rid >> 8;
    tx_frame.data[1] = rid & 0xFF;
    tx_frame.data[2] = 0x00;
    tx_frame.data[3] = 0x00;

    tx_frame.data[4] = *vbuf;
    tx_frame.data[5] = *(vbuf + 1);
    tx_frame.data[6] = *(vbuf + 2);
    tx_frame.data[7] = *(vbuf + 3);
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void RobMotorDriver::save_register_rob() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_ | CAN_EFF_FLAG | ROB_CMD::ROB_SAVEPRM;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = 0x01;
    tx_frame.data[1] = 0x02;
    tx_frame.data[2] = 0x03;
    tx_frame.data[3] = 0x04;

    tx_frame.data[4] = 0x05;
    tx_frame.data[5] = 0x06;
    tx_frame.data[6] = 0x07;
    tx_frame.data[7] = 0x08;
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void RobMotorDriver::refresh_motor_status() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_ | CAN_EFF_FLAG;
    tx_frame.can_dlc = 0x08;

    // tx_frame.data[0] = motor_id_ & 0xFF;
    // tx_frame.data[1] = motor_id_ >> 8;
    // tx_frame.data[2] = 0xCC;
    // tx_frame.data[3] = 0x00;

    // tx_frame.data[4] = 0x00;
    // tx_frame.data[5] = 0x00;
    // tx_frame.data[6] = 0x00;
    // tx_frame.data[7] = 0x00;
    // can_->transmit(tx_frame);
    // {
    //     response_count_++;
    // }
}

void RobMotorDriver::clear_motor_error() {
    clear_motor_error_rob();
}