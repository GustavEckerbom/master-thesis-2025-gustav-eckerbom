/*
 * SPDX-FileCopyrightText: Copyright 2023 Arm Limited and/or its
 * affiliates <open-source-office@arm.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 #include "camera_capture.hpp"
 #include <cstring>
 #include <cstdbool>
 #include <array>
 #include <functional>

 // If multicore defined the application will be using hwsemaphores to synchronize the frames, 
 // Some application on the MCU must unlock the semaphores to make sure the camera always has "available" memory to write to.
 #define MULTICORE 1

 namespace arm {
    namespace app {
        arm_camera_status_t camera_status = {
            .camera_on = false,
            .configure_next_frame = false,
            .frame_complete = false,
            .camera_error = false,
            .dropped_frame = false
        };
    }
}
 
#if defined(__cplusplus)
extern "C" {

#endif /* __cplusplus */
#include "Driver_CPI.h"
#include "RTE_Components.h"
#include CMSIS_device_header
#include "Driver_Common.h"
#include "Driver_GPIO.h"
#include "log_macros.h"
#include "board.h"
#include "system_utils.h"
#include "camera_capture.hpp" 
#include "image_buffer.h"
#include "cpi.h"
#include "hwsem_wrapper.h"
#include "utimer_wrapper.h"
#include "Core_select.h"
#include "event_preprocessing.h"
#include "pinconf.h"
#include "board_gpio_handles.h"

#define END_OF_FRAME_GPIO_PORT 12
#define END_OF_FRAME_GPIO_PIN 5 // Used to toggle pin at end of frame to see if the frame was captured by CPI

extern ARM_DRIVER_CPI Driver_LPCPI;

uint32_t frame_counter = 0;
uint8_t number_of_frames = 0;

#define cpi ((CPI_Type *)LPCPI_BASE)

static void camera_event_cb(uint32_t event) {
    if (event & ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED){       
        BOARD_IOHEADER_GPIOdrv->SetValue(BOARD_GPIO_PINHEAD_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);

        // Reset the CPI
        // cpi->CAM_CTRL |= CAM_CTRL_SW_RESET;
        cpi->CAM_CTRL = CAM_CTRL_SW_RESET;
        
        //  clear reset bit
        //  cpi->CAM_CTRL = 0;

        // Start CPI
        cpi->CAM_CTRL = (CAM_CTRL_START | CAM_CTRL_FIFO_CLK_SEL);
        
        BOARD_IOHEADER_GPIOdrv->SetValue(BOARD_GPIO_PINHEAD_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
    }

    if (event & ARM_CPI_EVENT_CAMERA_FRAME_VSYNC_DETECTED) {
        
        // --- New frame arriving right now ---
        new_frame_available = true;

        number_of_frames++;
               
        // Are we currently capturing the last frame in the ring buffer?
        if (frame_index == NUM_BUFFERS - 1){
            // Set CPI stop to turn off after frame has finished, keep the FIFO clock on
            cpi->CAM_CTRL = CAM_CTRL_FIFO_CLK_SEL;
        }
    }

    // --- Handle CPI errors (outside VSYNC) ---
    if (event & ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN) {
        arm::app::camera_status.camera_error = true;
        // END_OF_FRAMEdrv->SetValue(END_OF_FRAME_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        printf("Cam_err INP FIFO \r\n");
    }
    if (event & ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN) {
        arm::app::camera_status.camera_error = true;
        // END_OF_FRAMEdrv->SetValue(END_OF_FRAME_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        printf("Cam_err OUT FIFO\r\n");
    }
    if (event & ARM_CPI_EVENT_MIPI_CSI2_ERROR) {
        arm::app::camera_status.camera_error = true;
        // END_OF_FRAMEdrv->SetValue(END_OF_FRAME_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        printf("Camera_error CSI\r\n");
    }
}

 #if defined(__cplusplus)
 }
 #endif /* __cplusplus */
 
 __attribute__((noreturn)) static void CameraErrorLoop(const char* errorStr)
 {
    printf_err("%s\n", errorStr);

    while(true) {
        BOARD_LED_BOT_GPIO_GPIOdrv->SetValue(BOARD_LED_BOT_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
        // 3s delay
        for (int i = 0; i < 30; i++) {
            sys_busy_loop_us(100*1000); 
        }

        BOARD_LED_BOT_GPIO_GPIOdrv->SetValue(BOARD_LED_BOT_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
        // 3s delay
        for (int i = 0; i < 30; i++) {
            sys_busy_loop_us(100*1000);
        }
    }
}

void arm::app::LockNextFrame() {
    /* --- Reset new frame avail flag --- */
    new_frame_available = false;

    /* --- Save last frame idex --- */
    last_frame_index = frame_index;

    /* --- Now Last Frame index is being written to --- */
    frame_buffers[last_frame_index].status = BUFFER_WRITING;

    // Keep track so we know when to reset the CPI
    frame_index = (frame_index + 1) % NUM_BUFFERS;
       
    /* --- NNext frame will be put in frame_index, mark as queued --- */
    frame_buffers[frame_index].status = BUFFER_QUEUED;    
}
 
int arm::app::CameraCaptureInit()
{   
     // Toggle the IO pin a few times
    for (int i = 0; i < 6; i++) {
        sys_busy_loop_us(100*10);
    }

    // Toggle the IO pin a few times
    for (int i = 0; i < 6; i++) {
        BOARD_IOHEADER_GPIOdrv->SetValue(BOARD_GPIO_PINHEAD_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        sys_busy_loop_us(100*10);
    }

    if (0 != Driver_LPCPI.Initialize(camera_event_cb)) {

        // Toggle the IO pin a few times
        for (int i = 0; i < 8; i++) {
            sys_busy_loop_us(100*10);
        }

        // Toggle the IO pin a few times
        for (int i = 0; i < 8; i++) {
            BOARD_IOHEADER_GPIOdrv->SetValue(BOARD_GPIO_PINHEAD_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
            sys_busy_loop_us(100*10);
        }
        printf("Camera initialisation failed.\n\r");
    }
    
    if (0 != Driver_LPCPI.PowerControl(ARM_POWER_FULL)) {
        // Toggle the IO pin a few times
        for (int i = 0; i < 10; i++) {
            sys_busy_loop_us(100*10);
        }

        // Toggle the IO pin a few times
        for (int i = 0; i < 10; i++) {
            BOARD_IOHEADER_GPIOdrv->SetValue(BOARD_GPIO_PINHEAD_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
            sys_busy_loop_us(100*10);
        }

    printf("Camera power up failed.\n\r");
    }
    
    if (0 != Driver_LPCPI.Control(CPI_CAMERA_SENSOR_CONFIGURE, 0)) {

         // Toggle the IO pin a few times
        for (int i = 0; i < 12; i++) {
            sys_busy_loop_us(100*10);
        }

        // Toggle the IO pin a few times
        for (int i = 0; i < 12; i++) {
            BOARD_IOHEADER_GPIOdrv->SetValue(BOARD_GPIO_PINHEAD_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
            sys_busy_loop_us(100*10);
        }

    printf("Camera configuration failed.\n\r");
    }

#ifdef CPI_CONFIGURE
    if (0 != Driver_LPCPI.Control(CPI_CONFIGURE, 0)) {
    printf("Camera CPI_CONFIGURE failed.\n\r");
    }
#endif
    // Check this in details
    if (0 != Driver_LPCPI.Control(CPI_EVENTS_CONFIGURE,
                                                    ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN |
                                                    ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN |
                                                    ARM_CPI_EVENT_ERR_HARDWARE |
                                                    ARM_CPI_EVENT_MIPI_CSI2_ERROR |
                                                    ARM_CPI_EVENT_CAMERA_FRAME_VSYNC_DETECTED |
                                                    ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED
                                                    )) {
                                                         // Toggle the IO pin a few times
        for (int i = 0; i < 14; i++) {
            sys_busy_loop_us(100*10);
        }

        // Toggle the IO pin a few times
        for (int i = 0; i < 14; i++) {
            BOARD_IOHEADER_GPIOdrv->SetValue(BOARD_GPIO_PINHEAD_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
            sys_busy_loop_us(100*10);
        }
        printf("Camera CPI_EVENTS_CONFIGURE failed.\n\r");
    }

    info("Camera initialised.\n\r");
    return 0;
}
 
 static inline void CameraStatusReset()
 {
     NVIC_DisableIRQ((IRQn_Type) CAM_IRQ_IRQn);
     arm::app::camera_status.frame_complete = false;
     arm::app::camera_status.camera_error = false;
     NVIC_EnableIRQ((IRQn_Type) CAM_IRQ_IRQn);
 }
 
int arm::app::CameraCaptureStart(uint8_t* rawImage)
{
    CameraStatusReset();
    /* NOTE: This is a blocking call at the moment; doesn't need to be.
    *       It slows down the whole pipeline considerably. */
    Driver_LPCPI.CaptureFrame(rawImage);
    arm::app::camera_status.camera_on = true;
    return 0;
}
 
int arm::app::CameraCaptureStartStream(uint8_t* rawImage)
{    
    CameraStatusReset();
    Driver_LPCPI.CaptureVideo(rawImage);
    arm::app::camera_status.camera_on = true;
    return 0;
}

void arm::app::CameraCaptureWaitForFrame()
{
    while (arm::app::camera_status.frame_complete != true) {
    }

    if (arm::app::camera_status.camera_error) {
    printf_err("Camera error detected!\n\r");
    }
}

int arm::app::CameraCaptureStopStreaming()
{
    Driver_LPCPI.Stop();
    arm::app::camera_status.camera_on = false;
    return 0;
}