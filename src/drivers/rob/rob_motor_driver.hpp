#pragma once

#include <atomic>
#include <string>

#include "motor_driver.hpp"
#include "protocol/can/socket_can.hpp"
enum ROBError {
    ROBE_UNDERVOLT = 0x01,
    ROBE_OVERCURR  = 0x02,
    ROBE_OVERTEMP  = 0x04,
    ROBE_MAGCODING = 0x08,
    ROBE_UNCALIB1  = 0x10,
    ROBE_UNCALIB2  = 0x20,
};

enum ROBErrorFlag {
    ROBEF_UNDERVOLT = 0x00010000,
    ROBEF_OVERCURR  = 0x00020000,
    ROBEF_OVERTEMP  = 0x00040000,
    ROBEF_MAGCODING = 0x00080000,
    ROBEF_UNCALIB1  = 0x00100000,
    ROBEF_UNCALIB2  = 0x00200000,
};

enum ROB_Motor_Model { 
    RS00_48V = 0, 
    RS03_48V = 1, 
    RS06_48V = 2,
    ROB_Num_Of_Model,
};

enum ROB_REG {
    MECHANICAL_OFFSET       = 0x2005,
    MEASURED_POSITION       = 0x3016,
    MEASURED_VELOCITY       = 0x3017,
    MEASURED_TORQUE         = 0x302C,
    MODE                    = 0x7005,
    IQ_TARGET               = 0x7006,
    VELOCITY_TARGET         = 0x700A,
    TORQUE_LIMIT            = 0x700B,
    CURRENT_KP              = 0x7010,
    CURRENT_KI              = 0x7011,
    CURRENT_FILTER_GAIN     = 0x7014,
    POSITION_TARGET         = 0x7016,
    VELOCITY_LIMIT          = 0x7017,
    CURRENT_LIMIT           = 0x7018,
    MECHANICAL_POSITION     = 0x7019,
    IQ_FILTERED             = 0x701A,
    MECHANICAL_VELOCITY     = 0x701B,
    VBUS                    = 0x701C,
    POSITION_KP             = 0x701E,
    VELOCITY_KP             = 0x701F,
    VELOCITY_KI             = 0x7020,
    VELOCITY_FILTER_GAIN    = 0x7021,
    VEL_ACCELERATION_TARGET = 0x7022,
    PP_VELOCITY_MAX         = 0x7024,
    PP_ACCELERATION_TARGET  = 0x7025,
    EPSCAN_TIME             = 0x7026,
    CAN_TIMEOUT             = 0x7028,
    ZERO_STATE              = 0x7029,
};

enum ROB_CMD {
    ROB_DEVICEID   = 0x00000000,
    ROB_CONTROL = 0x01000000,
    ROB_STATUS  = 0x02000000,
    ROB_ENABLE  = 0x03000000,
    ROB_DISABLE = 0x04000000,
    ROB_ZEROPOS = 0x06000000,
    ROB_SETID   = 0x07000000,
    ROB_READPRM = 0x11000000,
    ROB_WRITEPRM = 0x12000000,
    ROB_FAULTREP = 0x15000000,
    ROB_SAVEPRM  = 0x16000000,
    ROB_BAUDRATE = 0x17000000,
    ROB_ACTIVEREP = 0x18000000,
    ROB_SETPROTOCOL = 0x19000000,
};

typedef struct {
    float PosMax;       ///< Maximum position limit (rad)
    float SpdMax;       ///< Maximum velocity limit (rad/s)
    float TauMax;       ///< Maximum torque limit (N·m)
    float OKpMax;       ///< Maximum outer-loop proportional gain
    float OKdMax;       ///< Maximum outer-loop derivative gain
} ROB_Limit_Param;

class RobMotorDriver : public MotorDriver {
   public:
    RobMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                  ROB_Motor_Model motor_model, double motor_zero_offset = 0.0);
    ~RobMotorDriver();

    virtual void lock_motor() override;
    virtual void unlock_motor() override;
    virtual uint8_t init_motor() override;
    virtual void deinit_motor() override;
    virtual bool set_motor_zero() override;
    virtual bool write_motor_flash() override;

    virtual void get_motor_param(uint8_t param_cmd) override;
    virtual void motor_pos_cmd(float pos, float spd, bool ignore_limit) override;
    virtual void motor_spd_cmd(float spd) override;
    virtual void motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) override;
    virtual void reset_motor_id() override {};
    virtual void set_motor_control_mode(uint8_t motor_control_mode) override;
    virtual int get_response_count() const {
        return response_count_;
    }
    virtual void refresh_motor_status() override;
    virtual void clear_motor_error() override;

   private:
    uint16_t master_id_;
    std::atomic<int> response_count_{0};
    bool param_cmd_flag_[30] = {false};
    ROB_Motor_Model motor_model_;
    ROB_Limit_Param limit_param_;
    std::atomic<uint8_t> mos_temperature_{0};
    std::string can_interface_;
    void set_motor_zero_rob();
    void clear_motor_error_rob();
    void write_register_rob(uint16_t rid, float value);
    void write_register_rob(uint16_t rid, int32_t value);
    void save_register_rob();
    virtual void can_rx_cbk(const can_frame& rx_frame);
    std::shared_ptr<MotorsSocketCAN> can_;
};
