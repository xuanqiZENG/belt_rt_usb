//
// Created by lingwei on 3/25/24.
//
#include "legged_sirius_hw/rt_usb_interface.h"
#include "iostream"
#include <cstdlib>
// #include "spi_command_t.hpp"
#include <thread>
#include <cstdio>
#include <cstring>
int actual_length_out = 0;
int actual_length_in = 0;

static uint32_t data_checksum(const uint32_t* data_to_check, uint32_t check_length)
{
    uint32_t t = 0;
    for(uint32_t i = 0; i < check_length; i++)
    {
        t = t ^ data_to_check[i];
    }

    return t;
}

Sirius_USB2CAN_Board::Sirius_USB2CAN_Board(uint16_t vendor_id, uint16_t product_id, uint8_t _motors_epin,
                                           uint8_t _motors_epout):
        usb_vendor_id(vendor_id), usb_product_id(product_id),
        motors_endpoint_in(_motors_epin), motors_endpoint_out(_motors_epout){
    usb_cmd_u = new USB_Cmd_U();
    usb_data_u = new USB_Data_U ();
    control_cmd = new spi_command_t();
    control_data = new spi_data_t();
    usb_in_mutex = new std::mutex();
    usb_out_mutex = new std::mutex();

    time_last_out = std::chrono::steady_clock::now();
    time_now_out = std::chrono::steady_clock::now();

    time_last_in = std::chrono::steady_clock::now();
    time_now_in = std::chrono::steady_clock::now();
    out_zero_flag = 0;
    out_zero_count = 0;
    libusb_init(&ctx);
    transfer_out_motors = libusb_alloc_transfer(0);
    transfer_in_motors = libusb_alloc_transfer(0);
    device_handle = libusb_open_device_with_vid_pid(ctx, usb_vendor_id, usb_product_id);
    if(libusb_kernel_driver_active(device_handle, 0x00))
    {
        int success = libusb_detach_kernel_driver(device_handle, 0x00);
        if(success != 0)
        {
            std::cerr << "Detach Driver Failed!" << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    int claim_interface = libusb_claim_interface(device_handle, 0x00);
    if(claim_interface != 0)
    {
        std::cerr << "Claim Driver Failed!" << std::endl;
        exit(EXIT_FAILURE);
    }
    else{
        std::cout << "Claim USB Device OK!\n";
    }
}

Sirius_USB2CAN_Board::~Sirius_USB2CAN_Board() {

}

void Sirius_USB2CAN_Board::destroy_thread() {
    std::cout << "i am out1\n";
    delete  usb_cmd_u;
    delete  usb_data_u;
    delete  control_cmd;
    delete  control_data;
    delete  usb_in_mutex;
    delete  usb_out_mutex;
std::cout << "i am out2\n";
    libusb_free_transfer(transfer_in_motors);
    libusb_free_transfer(transfer_out_motors);
    libusb_release_interface(device_handle, 0);
    libusb_close(device_handle);
    libusb_exit(ctx);
    std::cout << "i am out3\n";
}



void Sirius_USB2CAN_Board::USB2CAN_SetBuffer(spi_command_t* _control_cmd, spi_data_t* _controller_data) {
    control_cmd = _control_cmd;
    control_data = _controller_data;
}

void Sirius_USB2CAN_Board::motor_epin_callback(struct libusb_transfer *_transfer) {
    if(_transfer->status != LIBUSB_TRANSFER_COMPLETED)
    {
        std::cout << "Motor Ep81 IN Error! Transfer again!\n";

    }
    else if(_transfer->status == LIBUSB_TRANSFER_COMPLETED)
    {
        this->lock_in_mutex();
        this->Deal_Usb_In_Data();
        this->unlock_in_mutex();
//        time_now_in = std::chrono::steady_clock::now();
//        std::chrono::duration<double,std::micro> time_used = std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(time_now_in - time_last_in);
//        std::cout << "[Time Interval In]: " << time_used.count() << " us\n";
//        time_last_in = time_now_in;
        libusb_submit_transfer(_transfer);
    }
    //std::cout <<_transfer->status <<"\n";
}

void Sirius_USB2CAN_Board::motor_epout_callback(struct libusb_transfer *_transfer) {
    if(_transfer->status != LIBUSB_TRANSFER_COMPLETED)
    {
        std::cout << "Motor Ep01 OUT Error! Transfer again!\n";

    }
    else if(_transfer->status == LIBUSB_TRANSFER_COMPLETED)
    {
//        fresh_usb_cmd_from_controller();
        this->lock_out_mutex();
        this->Deal_Usb_Out_Cmd();
        this->unlock_out_mutex();
//        time_now_out = std::chrono::steady_clock::now();
//        std::chrono::duration<double,std::micro> time_used = std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(time_now_out - time_last_out);
//        std::cout << "[Time Interval Out]: " << time_used.count() << " us\n";
//        time_last_out = time_now_out;

        libusb_submit_transfer(_transfer);
    }
}

float last_one = 1000;

void Sirius_USB2CAN_Board::Deal_Usb_In_Data(){
    uint32_t t = data_checksum((uint32_t*)usb_data_u, usb_motors_in_check_length);
    volatile uint8_t leg_id = 0;
    volatile uint8_t data_index = 0;
    if(usb_data_u->usb_data.checksum == t)
    {
        for(uint8_t i = 0; i < 2 * NUMBER_CHIPS; i++) {
            leg_id = i / 2;
            data_index = i % 2;
            control_data->q_abad[i] = (usb_data_u->usb_data.leg_data[leg_id].p_abad_data[data_index] - abad_offset[i]) * abad_side_sign[i];
            control_data->q_hip[i] = (usb_data_u->usb_data.leg_data[leg_id].p_hip_data[data_index] - hip_offset[i]) * hip_side_sign[i];
            control_data->q_knee[i] = (usb_data_u->usb_data.leg_data[leg_id].p_knee_data[data_index] - knee_offset[i]) * knee_side_sign[i];
            control_data->qd_abad[i] = usb_data_u->usb_data.leg_data[leg_id].v_abad_data[data_index] * abad_side_sign[i];
            control_data->qd_hip[i] = usb_data_u->usb_data.leg_data[leg_id].v_hip_data[data_index] * hip_side_sign[i];
            control_data->qd_knee[i] = usb_data_u->usb_data.leg_data[leg_id].v_knee_data[data_index] * knee_side_sign[i];
            control_data->tau_abad[i] = usb_data_u->usb_data.leg_data[leg_id].t_abad_data[data_index] * abad_side_sign[i];
            control_data->tau_hip[i] = usb_data_u->usb_data.leg_data[leg_id].t_hip_data[data_index] * hip_side_sign[i];
            control_data->tau_knee[i] = usb_data_u->usb_data.leg_data[leg_id].t_knee_data[data_index] / knee_side_sign[i];
        }
        control_data->flags[0] = usb_data_u->usb_data.leg_data[0].leg_flag[0];
        control_data->flags[1] = usb_data_u->usb_data.leg_data[1].leg_flag[0];
//       if(std::fabs(usb_data_u->usb_data.leg_data[0].p_knee_data[0])<0.001)
//       {
//           std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!/n";
//       }
//        if(last_one == control_data->q_abad[0])
//          std::cout << "[Transmit ERRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRrrrrn";
    //    std::cout << "\033[0m" << control_data->q_abad[0] << " " << control_data->q_abad[1]
                //  << " " << control_data->q_hip[0]<< " " << control_data->q_hip[1] <<  std::endl << "\033[37m";
//        last_one = control_data->q_abad[0];
    }
    else
    {
        std::cout << "usb data checksum error\n";
    }
//    std::cout << "[Joints Input: ]" <<control_data->q_abad[0] << " | " << control_data->q_hip[0] << " | " << control_data->q_knee[0] <<"\n";
//if(control_data->q_abad[0] != -0.1f)
//{
//    std::cout << "[Joints Input: ]" << "\033[31m" <<control_data->q_abad[0] << " | " << control_data->q_hip[0] << " | " << control_data->q_knee[0]  << " | "
//    << usb_data_u->usb_data.leg_data[0].p_abad_data[0] << " | " << usb_data_u->usb_data.leg_data[0].p_hip_data[0] << "\033[0m";
//}
}

void Sirius_USB2CAN_Board::Deal_Usb_Out_Cmd(){
    volatile uint8_t leg_id;
    volatile uint8_t cmd_index;
    for(uint8_t i = 0; i < 2 * NUMBER_CHIPS; i++) {
        leg_id = i / 2;
        cmd_index = i % 2;
        usb_cmd_u->usb_cmd.leg_cmd[leg_id].p_abad_cmd[cmd_index] = (control_cmd ->q_des_abad[i] * abad_side_sign[i]) + abad_offset[i];
        usb_cmd_u->usb_cmd.leg_cmd[leg_id].p_hip_cmd[cmd_index]  = (control_cmd->q_des_hip[i] * hip_side_sign[i]) + hip_offset[i];
        usb_cmd_u->usb_cmd.leg_cmd[leg_id].p_knee_cmd[cmd_index]  = (control_cmd->q_des_knee[i] / knee_side_sign[i]) + knee_offset[i];

        usb_cmd_u->usb_cmd.leg_cmd[leg_id].v_abad_cmd[cmd_index]  = control_cmd->qd_des_abad[i] * abad_side_sign[i];
        usb_cmd_u->usb_cmd.leg_cmd[leg_id].v_hip_cmd[cmd_index]  = control_cmd->qd_des_hip[i] * hip_side_sign[i];
        usb_cmd_u->usb_cmd.leg_cmd[leg_id].v_knee_cmd[cmd_index]  = control_cmd->qd_des_knee[i] / knee_side_sign[i];

        usb_cmd_u->usb_cmd.leg_cmd[leg_id].kp_abad_cmd[cmd_index]  = control_cmd->kp_abad[i];
        usb_cmd_u->usb_cmd.leg_cmd[leg_id].kp_hip_cmd[cmd_index]  = control_cmd->kp_hip[i];
        usb_cmd_u->usb_cmd.leg_cmd[leg_id].kp_knee_cmd[cmd_index]  = control_cmd->kp_knee[i];

        usb_cmd_u->usb_cmd.leg_cmd[leg_id].kd_abad_cmd[cmd_index]  = control_cmd->kd_abad[i];
        usb_cmd_u->usb_cmd.leg_cmd[leg_id].kd_hip_cmd[cmd_index]  = control_cmd->kd_hip[i];
        usb_cmd_u->usb_cmd.leg_cmd[leg_id].kd_knee_cmd[cmd_index]  = control_cmd->kd_knee[i];

        usb_cmd_u->usb_cmd.leg_cmd[leg_id].t_abad_cmd[cmd_index]  = control_cmd->tau_abad_ff[i] * abad_side_sign[i];
        usb_cmd_u->usb_cmd.leg_cmd[leg_id].t_hip_cmd[cmd_index]  = control_cmd->tau_hip_ff[i] * hip_side_sign[i];
        usb_cmd_u->usb_cmd.leg_cmd[leg_id].t_knee_cmd[cmd_index]  = control_cmd->tau_knee_ff[i] * knee_side_sign[i];
    }
    usb_cmd_u->usb_cmd.leg_cmd[0].leg_flag[0]  = control_cmd->flags[0];
    usb_cmd_u->usb_cmd.leg_cmd[1].leg_flag[0]  = control_cmd->flags[1];
    usb_cmd_u->usb_cmd.checksum = data_checksum((uint32_t*)usb_cmd_u, usb_motors_out_check_length);
//    std::cout << "[Joints Output: ] " << usb_cmd_u->usb_cmd.leg_cmd[0].p_abad_cmd[0] <<" | " <<  usb_cmd_u->usb_cmd.leg_cmd[0].p_hip_cmd[0] << " | " <<
//    usb_cmd_u->usb_cmd.leg_cmd[0].p_knee_cmd[0] << "\n";
    out_zero_count++;
    if (out_zero_count>20)
    {
        out_zero_count=0;
        //out_zero_flag = true;
    }

}

void Sirius_USB2CAN_Board::USB2CAN_Start_Transfer_Ans() {
    libusb_fill_interrupt_transfer(transfer_out_motors,device_handle, motors_endpoint_out,
                                   usb_cmd_u->usb_cmd_buff, usb_motors_out_length, usb_motors_out_cbf_wrapper, this, 0);
    libusb_fill_interrupt_transfer(transfer_in_motors, device_handle, motors_endpoint_in,usb_data_u->usb_data_buff,
                                   usb_motors_in_length, usb_motors_in_cbf_wrapper, this, 0);
    libusb_submit_transfer(transfer_in_motors);
    libusb_submit_transfer(transfer_out_motors);
    if((!transfer_out_motors->status) & (!transfer_in_motors->status))
    {
        std::cout << "[Good] All endpoints start transfering!\n";
    }
    else
    {
        std::cout << "[Bad] Some endpoints not work\n";
        exit(EXIT_FAILURE);
    }
}

void Sirius_USB2CAN_Board::lock_in_mutex(){
    usb_in_mutex->lock();
}

void Sirius_USB2CAN_Board::lock_out_mutex() {
    usb_out_mutex->lock();
}

void Sirius_USB2CAN_Board::unlock_out_mutex() {
    usb_out_mutex->unlock();
}

void Sirius_USB2CAN_Board::unlock_in_mutex() {
    usb_in_mutex->unlock();
}

void usb_motors_in_cbf_wrapper(struct libusb_transfer* _transfer)
{
    auto *temp = reinterpret_cast<Sirius_USB2CAN_Board*>(_transfer->user_data);
    temp->motor_epin_callback(_transfer);
}

void usb_motors_out_cbf_wrapper(struct libusb_transfer* _transfer)
{
    auto *temp = reinterpret_cast<Sirius_USB2CAN_Board*>(_transfer->user_data);
    temp->motor_epout_callback(_transfer);
}
