//
// Created by lingwei on 3/25/24.
//

#ifndef SIRIUS_SOFT_RT_USB_INTERFACE_H
#define SIRIUS_SOFT_RT_USB_INTERFACE_H
#include <cstdint>
#include "libusb-1.0/libusb.h"
#include <chrono>
// #include "common/include/SimUtilities/SpineBoard.h"
#include <mutex>
#include <cmath>

const uint8_t NUMBER_CHIPS = 2;
const uint16_t usb_motors_in_length = 156;
const uint16_t usb_motors_out_length = 252;
const uint16_t usb_motors_in_check_length = usb_motors_in_length/4 - 1;
const uint16_t usb_motors_out_check_length = usb_motors_out_length/4 - 1;
//todo: Check the size of remote controllers

#define K_KNEE_OFFSET_POS 0.0f
#define K_HIP_OFFSET_POS (0.0f)
#define K_ABAD_OFFSET_POS (0.0f)

const float max_torque[3] = {24.f, 24.f, 26.f};
const float wimp_torque[3] = {6.f, 6.f, 6.f};
const float disabled_torque[3] = {0.f, 0.f, 0.f};
// only used for actual robot
const float abad_side_sign[4] = {1.f, 1.f, 1.f, 1.f};
const float hip_side_sign[4] = {1.f, 1.f, 1.f, 1.f};
const float knee_side_sign[4] = {18.0/26.0f, 18.0/26.0f, 18.0/26.0f, 18.0/26.0f};
// only used for actual robot
const float abad_offset[4] = {K_ABAD_OFFSET_POS, -K_ABAD_OFFSET_POS, -K_ABAD_OFFSET_POS, K_ABAD_OFFSET_POS}; //

const float hip_offset[4] = {K_HIP_OFFSET_POS, K_HIP_OFFSET_POS, -K_HIP_OFFSET_POS, K_HIP_OFFSET_POS};
const float knee_offset[4] = {K_KNEE_OFFSET_POS, K_KNEE_OFFSET_POS,
                              K_KNEE_OFFSET_POS, K_KNEE_OFFSET_POS};

typedef struct {
    float q_des_abad[4];
    float q_des_hip[4];
    float q_des_knee[4];
    float qd_des_abad[4];
    float qd_des_hip[4];
    float qd_des_knee[4];
    float kp_abad[4];
    float kp_hip[4];
    float kp_knee[4];
    float kd_abad[4];
    float kd_hip[4];
    float kd_knee[4];
    float tau_abad_ff[4];
    float tau_hip_ff[4];
    float tau_knee_ff[4];
    uint32_t flags[4];
} spi_command_t;

typedef struct {
    float q_abad[4];
    float q_hip[4];
    float q_knee[4];
    float qd_abad[4];
    float qd_hip[4];
    float qd_knee[4];
    float tau_abad[4];
    float tau_hip[4];
    float tau_knee[4];
    uint32_t flags[4];
} spi_data_t;

typedef struct Leg_Cmd{
    float p_abad_cmd[2];
    float p_hip_cmd[2];
    float p_knee_cmd[2];
    float v_abad_cmd[2];
    float v_hip_cmd[2];
    float v_knee_cmd[2];
    float kp_abad_cmd[2];
    float kp_hip_cmd[2];
    float kp_knee_cmd[2];
    float kd_abad_cmd[2];
    float kd_hip_cmd[2];
    float kd_knee_cmd[2];
    float t_abad_cmd[2];
    float t_hip_cmd[2];
    float t_knee_cmd[2];
    uint32_t leg_flag[1];
} Leg_Cmd_T;

typedef struct Leg_Data{
    float p_abad_data[2];
    float p_hip_data[2];
    float p_knee_data[2];
    float v_abad_data[2];
    float v_hip_data[2];
    float v_knee_data[2];
    float t_abad_data[2];
    float t_hip_data[2];
    float t_knee_data[2];
    uint32_t leg_flag[1];
} Leg_Data_T;

typedef struct USB_Cmd{
    Leg_Cmd_T leg_cmd[2];
    uint32_t checksum;
} USB_Cmd_T;

typedef union USB_CMD{
    USB_Cmd_T usb_cmd;
    uint8_t   usb_cmd_buff[usb_motors_out_length];
} USB_Cmd_U;

typedef struct USB_Data{
    Leg_Data leg_data[2];
    uint32_t checksum;
} USB_Data_T;

typedef union USB_DATA{
    USB_Data_T usb_data;
    uint8_t    usb_data_buff[usb_motors_in_length];
} USB_Data_U;

class Sirius_USB2CAN_Board{
public:
    // Sirius_USB2CAN_Board()= default;
    Sirius_USB2CAN_Board(uint16_t vendor_id, uint16_t product_id, uint8_t _motors_epin, uint8_t _motors_epout);
    ~Sirius_USB2CAN_Board();
    // data union of this class is a temp buff, data checkok, memcpy to controll databuff.
    void USB2CAN_SetBuffer(spi_command_t* _control_cmd, spi_data_t* _controller_data);
    void USB2CAN_Start_Transfer_Ans();
    void motor_epin_callback(struct libusb_transfer* _transfer);
    void motor_epout_callback(struct libusb_transfer* _transfer);
    libusb_context*          ctx{};
    void lock_in_mutex();
    void lock_out_mutex();
    void unlock_out_mutex();
    void unlock_in_mutex();
    void destroy_thread();
    bool                     out_zero_flag{};
private:

    // for controller data protocals
    uint16_t                 usb_vendor_id{};
    uint16_t                 usb_product_id{};

    libusb_transfer*         transfer_in_motors{};
    libusb_transfer*         transfer_out_motors{};

    std::mutex*              usb_in_mutex{};
    std::mutex*              usb_out_mutex{};

    uint8_t                  motors_endpoint_in{};
    uint8_t                  motors_endpoint_out{};

    uint8_t                  out_zero_count{};
    USB_Data_U*              usb_data_u{};
    USB_Cmd_U*               usb_cmd_u{};
    spi_command_t *          control_cmd{};
    spi_data_t*              control_data{};
    libusb_device_handle*    device_handle{};
    std::chrono::steady_clock::time_point time_last_in;
    std::chrono::steady_clock::time_point time_now_in;
    std::chrono::steady_clock::time_point time_last_out;
    std::chrono::steady_clock::time_point time_now_out;
    void Deal_Usb_In_Data();
    void Deal_Usb_Out_Cmd();
};

void usb_motors_in_cbf_wrapper(struct libusb_transfer* _transfer);
void usb_motors_out_cbf_wrapper(struct libusb_transfer* _transfer);
void fresh_usb_cmd_from_controller();
#endif //SIRIUS_SOFT_RT_USB_INTERFACE_H
