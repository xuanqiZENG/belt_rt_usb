/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by Shengzhi Wang on 2023/12/22. (email: wahrheitwsz@gmail.com).
//

#include <legged_hw/LeggedHWLoop.h>
#include "legged_sirius_hw/SiriusHW.h"

// const uint16_t sirius_usb2can_vendor_id = 0x1111;
// const uint16_t sirius_usb2can_product_id = 0x2222;
// const uint8_t  motors_ep_in = 0x81;
// const uint8_t  motors_ep_out = 0x01;
// spi_command_t * usb_cmd_from_controller;
// spi_data_t * usb_data_to_controller;
// uint8_t first_run = 1;
// static int counter = 0;

int main(int argc, char** argv) {
  ros::init(argc, argv, "legged_sirius_hw");
  ros::NodeHandle nh;
  ros::NodeHandle robotHwNh("~");

  // Run the hardware interface node
  // -------------------------------

  // We run the ROS loop in a separate thread as external calls, such
  // as service callbacks loading controllers, can block the (main) control loop

  ros::AsyncSpinner spinner(4);
  spinner.start();
std::cout << "Test 1: \n";

  // usb_cmd_from_controller = new spi_command_t();
  // usb_data_to_controller = new spi_data_t();
  int complete = 0;
  // struct timeval timestru{};
  // timestru.tv_sec = 0;
  // timestru.tv_usec = 1000;

    // Sirius_USB2CAN_Board spUSB2CAN(sirius_usb2can_vendor_id,//
    //                               sirius_usb2can_product_id,
    //                               motors_ep_in,
    //                               motors_ep_out);
    // spUSB2CAN.USB2CAN_SetBuffer(usb_cmd_from_controller, usb_data_to_controller);
    // spUSB2CAN.USB2CAN_Start_Transfer_Ans();
  try {
    // Create the hardware interface specific to your robot
    
    std::shared_ptr<legged::SiriusHW> SiriusHw = std::make_shared<legged::SiriusHW>();
std::cout << "Test 2: \n";
    // SiriusArmHw->spUSB2CAN(sirius_usb2can_vendor_id,//
    //                               sirius_usb2can_product_id,
    //                               motors_ep_in,
    //                               motors_ep_out);
    // Initialize the hardware interface:
    // 1. retrieve configuration from rosparam
    // 2. initialize the hardware and interface it with ros_control
    // 3. initialize the thread for reading and writing data to arm
    SiriusHw->init(nh, robotHwNh);
    SiriusHw->usb_cmd_from_controller = new spi_command_t();
    SiriusHw->usb_data_to_controller = new spi_data_t();
    SiriusHw->usb2can_thread_ = std::thread([&]() {
      std::cout << "Test 3: \n";
      SiriusHw->spUSB2CAN.USB2CAN_SetBuffer(SiriusHw->usb_cmd_from_controller, SiriusHw->usb_data_to_controller);
      std::cout << "Test 4: \n";
      SiriusHw->spUSB2CAN.USB2CAN_Start_Transfer_Ans();
      std::cout << "Test 5: \n";
      struct timeval timestr{};
      timestr.tv_sec = 0;
      timestr.tv_usec = 1000;
      int complete = 0;
      SiriusHw->usb2canRunning_ = true;
      while (SiriusHw->usb2canRunning_ && ros::ok() && ros::master::check()) {
        libusb_handle_events_timeout_completed(SiriusHw->spUSB2CAN.ctx, &timestr, &complete);
        if (SiriusHw->spUSB2CAN.out_zero_flag)
        {
          std::cout << "Test break: \n";
          break;
        }
      }
    });
    
    // Start the control loop
    legged::LeggedHWLoop controlLoop(nh, SiriusHw);

    // Wait until shutdown signal received
    // char out_signal;
    // std::cin >> out_signal;
    // ros::shutdown();
    // delete SiriusArmHw->usb_cmd_from_controller;
    // delete SiriusArmHw->usb_data_to_controller;
    ros::waitForShutdown();
    SiriusHw.reset();
  } catch (const ros::Exception& e) {
    ROS_FATAL_STREAM("Error in the hardware interface:\n"
                     << "\t" << e.what());
    return 1;
  }

  return 0;
}

// void fresh_usb_cmd_from_controller()
// {
//     counter++;
//     float temp_angle = 0.001f * (float)counter;
//     for(int i = 0; i < (3 * NUMBER_CHIPS); i++)
//     {
//         //SiriusArmHw->usb_cmd_from_controller->q_des_abad[i] = 1.5;
//         // SiriusArmHw->usb_cmd_from_controller->q_des_hip[i] = 2;
//         // SiriusArmHw->usb_cmd_from_controller->q_des_knee[i] = 3;
//         SiriusArmHw->usb_cmd_from_controller->qd_des_abad[i] = 3.f* sinf(temp_angle);
// //        SiriusArmHw->usb_cmd_from_controller->tau_abad_ff[i] = -90.f;
//         //SiriusArmHw->usb_cmd_from_controller->kd_abad[i] = 1.f;
// //        SiriusArmHw->usb_cmd_from_controller->qd_des_hip[i]= 4;
// //        SiriusArmHw->usb_cmd_from_controller->kd_hip[i] = 1.f;
// //        SiriusArmHw->usb_cmd_from_controller->qd_des_knee[i]= 5;
//         // SiriusArmHw->usb_cmd_from_controller->kp_abad[i]= SiriusArmHw->usb_cmd_from_controller->kp_hip[i] = SiriusArmHw->usb_cmd_from_controller->kp_knee[i] = 10;
//         SiriusArmHw->usb_cmd_from_controller->kd_abad[i]= SiriusArmHw->usb_cmd_from_controller->kd_hip[i] = SiriusArmHw->usb_cmd_from_controller->kd_knee[i] = 2;
//         //SiriusArmHw->usb_cmd_from_controller->tau_abad_ff[i]= sinf(temp_angle) + 8;
// //        SiriusArmHw->usb_cmd_from_controller->tau_hip_ff[i]= 9;
// //        SiriusArmHw->usb_cmd_from_controller->tau_knee_ff[i]= 10;
//         // 00000 100 01 Position / VEL / Mit
//         if(first_run)
//         {
//             SiriusArmHw->usb_cmd_from_controller->flags[0] = SiriusArmHw->usb_cmd_from_controller->flags[1]
//                     = SiriusArmHw->usb_cmd_from_controller->flags[2] = (0x01 << 1); //disable
//             //SiriusArmHw->usb_cmd_from_controller->flags[i] = 0;
//         } else {
//             //SiriusArmHw->usb_cmd_from_controller->flags[i] = 1;
//             SiriusArmHw->usb_cmd_from_controller->flags[0] = SiriusArmHw->usb_cmd_from_controller->flags[1]
//             = SiriusArmHw->usb_cmd_from_controller->flags[2]= (0x01 << 2) | 0x01;
//         }
//     }
//     if(counter == 10) {
//         first_run = 0;
//     }
// }


