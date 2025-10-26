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

#define END_OF_FRAME_GPIO_PORT 12
#define END_OF_FRAME_GPIO_PIN 5

// REV3
// #define BOARD_VSYNC_INTERRUPT_GPIO_PORT 11   
// #define BOARD_VSYNC_INTERRUPT_GPIO_PIN 1

//REV2
//#define BOARD_VSYNC_INTERRUPT_GPIO_PORT 3   
// #define BOARD_VSYNC_INTERRUPT_GPIO_PIN 6

/* --- Driver pointers --- */
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_PXRSTN_GPIO_PORT);
extern ARM_DRIVER_GPIO Driver_GPIO1;
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_LED_BOT_GPIO_PORT);
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_PSU_EN_2V5_GPIO_PORT);
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_PXRSTN_GPIO_PORT);
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_GPIO_PINHEAD_GPIO_PORT);
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(END_OF_FRAME_GPIO_PORT);
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_VSYNC_INTERRUPT_GPIO_PORT);

ARM_DRIVER_GPIO *BOARD_CAM_PXRSTN_GPIO_GPIOdrv = &ARM_Driver_GPIO_(BOARD_CAM_PXRSTN_GPIO_PORT);
ARM_DRIVER_GPIO *BOARD_LED_BOT_GPIO_GPIOdrv = &ARM_Driver_GPIO_(BOARD_LED_BOT_GPIO_PORT);
ARM_DRIVER_GPIO *BOARD_CAM_PXRSTN_GPIOdrv = &ARM_Driver_GPIO_(BOARD_CAM_PXRSTN_GPIO_PORT);
ARM_DRIVER_GPIO *BOARD_PINHEAD_GPIOdrv = &ARM_Driver_GPIO_(BOARD_GPIO_PINHEAD_GPIO_PORT);
ARM_DRIVER_GPIO *END_OF_FRAMEdrv = &ARM_Driver_GPIO_(END_OF_FRAME_GPIO_PORT);
ARM_DRIVER_GPIO *VSYNC_GPIO_drv = &ARM_Driver_GPIO_(BOARD_VSYNC_INTERRUPT_GPIO_PORT);

extern ARM_DRIVER_CPI Driver_LPCPI;

uint32_t frame_counter = 0;
uint8_t number_of_frames = 0;

#define cpi ((CPI_Type *)LPCPI_BASE)

static void camera_event_vsync_cb(uint32_t event){
    if (event & ARM_GPIO_IRQ_EVENT_EXTERNAL) {
        // Was the frame that just finished the last frame in the ring buffer?
        if (frame_index == NUM_BUFFERS - 1){
            //END_OF_FRAMEdrv->SetValue(END_OF_FRAME_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);

            // Set CPI stop to turn off after frame has finished, keep the FIFO clock on software reset and
            // cpi->CAM_CTRL = CAM_CTRL_FIFO_CLK_SEL;
            // Reset the CPI
            cpi->CAM_CTRL = CAM_CTRL_SW_RESET;
            // Start CPI
            cpi->CAM_CTRL = (CAM_CTRL_START | CAM_CTRL_FIFO_CLK_SEL);
            // END_OF_FRAMEdrv->SetValue(END_OF_FRAME_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        }

        BOARD_PINHEAD_GPIOdrv->SetValue(BOARD_GPIO_PINHEAD_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        /* --- New frame finished --- */
        new_frame_available = true;

        if (frame_index == (NUM_BUFFERS - 1)){
            update_frame_deltatime();
        }

        number_of_frames++;     // Increment frame counter
    }

}

static void camera_event_cb(uint32_t event) {
/*
    if (event & ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED){       
        // END_OF_FRAMEdrv->SetValue(END_OF_FRAME_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        // Disable streaming from cam sensor
        // BOARD_CAM_PXRSTN_GPIOdrv->SetValue(BOARD_CAM_PXRSTN_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);

        // Start the new capture
        // Prepare for soft reset
        // cpi->CAM_CTRL = 0;

        // Reset the CPI
        // cpi->CAM_CTRL |= CAM_CTRL_SW_RESET;
        cpi->CAM_CTRL = CAM_CTRL_SW_RESET;
        
        //  clear reset bit
        // cpi->CAM_CTRL = 0;

        // Start CPI
        cpi->CAM_CTRL = (CAM_CTRL_START | CAM_CTRL_FIFO_CLK_SEL);
        
        // END_OF_FRAMEdrv->SetValue(END_OF_FRAME_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        // Enable camera sensor streaming
        // BOARD_CAM_PXRSTN_GPIOdrv->SetValue(BOARD_CAM_PXRSTN_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
    }
*/
/*
    if (event & ARM_CPI_EVENT_CAMERA_FRAME_VSYNC_DETECTED) {
        
        // BOARD_PINHEAD_GPIOdrv->SetValue(BOARD_GPIO_PINHEAD_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);

        // --- New frame arrived ---
        new_frame_available = true;

        if (frame_index == (NUM_BUFFERS - 1)){
            update_frame_deltatime();
        }

        number_of_frames++;
               
        // are we currently capturing the last frame in the ring buffer?
        if (frame_index == NUM_BUFFERS - 1){
            // Set CPI stop to turn off after frame has finished, keep the FIFO clock on
            cpi->CAM_CTRL = CAM_CTRL_FIFO_CLK_SEL;
        }
    }
*/

    if (event & ARM_CPI_EVENT_CAMERA_FRAME_VSYNC_DETECTED){
        BOARD_PINHEAD_GPIOdrv->SetValue(BOARD_GPIO_PINHEAD_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        arm::app::camera_status.configure_next_frame = true;
    }

    // --- Handle CPI errors (outside VSYNC) ---
    if (event & ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN) {
        arm::app::camera_status.camera_error = true;
        END_OF_FRAMEdrv->SetValue(END_OF_FRAME_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        printf("Cam_err INP FIFO \r\n");
    }
    if (event & ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN) {
        arm::app::camera_status.camera_error = true;
        END_OF_FRAMEdrv->SetValue(END_OF_FRAME_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        printf("Cam_err OUT FIFO\r\n");
    }
    if (event & ARM_CPI_EVENT_MIPI_CSI2_ERROR) {
        arm::app::camera_status.camera_error = true;
        END_OF_FRAMEdrv->SetValue(END_OF_FRAME_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
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

    /* --- Mark the buffer the CPI is *currently* writing to --- */
    // frame_buffers[frame_index].status = BUFFER_WRITING;

    /* --- Mark the previous buffer ready and release semaphore --- */
    //if (last_frame_index >= 0) {
    frame_buffers[last_frame_index].status = BUFFER_READY;

    if (!(hwsem_unlock(last_frame_index) == ARM_DRIVER_OK)) {
        printf("Failed to unlock last frame buffer %d\n\r", last_frame_index);
    }
    //}
       
    last_frame_index = frame_index;

    frame_index = (frame_index + 1) % NUM_BUFFERS;

    if (hwsem_cam_custom_lock(frame_index) == ARM_DRIVER_OK) {
        frame_buffers[frame_index].status = BUFFER_WRITING; // Next frame will be written to this buffer
    } 
    
    else {
        printf("Buffer overrun, Turning off camera %d\n\r", frame_index);
        arm::app::CameraCaptureStopStreaming();
    }
    arm::app::camera_status.frame_complete = true;
}
 
int arm::app::CameraCaptureInit()
{   
    
    if (0 != Driver_LPCPI.Initialize(camera_event_cb)) {
        printf("Camera initialisation failed.\n\r");
    }
    
    if (0 != Driver_LPCPI.PowerControl(ARM_POWER_FULL)) {
    printf("Camera power up failed.\n\r");
    }
    
    if (0 != Driver_LPCPI.Control(CPI_CAMERA_SENSOR_CONFIGURE, 0)) {
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
                                                    ARM_CPI_EVENT_MIPI_CSI2_ERROR // |
                                                    // ARM_CPI_EVENT_CAMERA_FRAME_VSYNC_DETECTED | 
                                                    // ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED 
                                                    )) {
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

    uint32_t config_button =    PADCTRL_READ_ENABLE |
                                PADCTRL_SCHMITT_TRIGGER_ENABLE;

    uint32_t irq_cfg =  ARM_GPIO_IRQ_POLARITY_HIGH |
                        ARM_GPIO_IRQ_SENSITIVE_EDGE;

    VSYNC_GPIO_drv->Initialize(BOARD_VSYNC_INTERRUPT_GPIO_PIN, camera_event_vsync_cb);
    VSYNC_GPIO_drv->PowerControl(BOARD_VSYNC_INTERRUPT_GPIO_PIN, ARM_POWER_FULL);
    VSYNC_GPIO_drv->SetDirection(BOARD_VSYNC_INTERRUPT_GPIO_PIN, GPIO_PIN_DIRECTION_INPUT);

    // Set pinmux to GPIO mode with no pull-up/down
    pinconf_set(PORT_(BOARD_VSYNC_INTERRUPT_GPIO_PORT),
                BOARD_VSYNC_INTERRUPT_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_0,
                config_button);

    // Make sure debounce disabled
    uint32_t debounce_disable = 0;
    VSYNC_GPIO_drv->Control(BOARD_VSYNC_INTERRUPT_GPIO_PIN, ARM_GPIO_CONFIG_DEBOUNCE, &debounce_disable);


    // Enable GPIO interrupt on rising edge
    VSYNC_GPIO_drv->Control(BOARD_VSYNC_INTERRUPT_GPIO_PIN,
                            ARM_GPIO_ENABLE_INTERRUPT,
                            &irq_cfg);
        /*
    uint32_t config_button =
        PADCTRL_READ_ENABLE |
        PADCTRL_SCHMITT_TRIGGER_ENABLE| PADCTRL_DRIVER_DISABLED_PULL_UP | PADCTRL_DRIVER_DISABLED_PULL_DOWN;

    uint32_t arg = ARM_GPIO_IRQ_POLARITY_HIGH | ARM_GPIO_IRQ_SENSITIVE_EDGE;

    VSYNC_GPIO_drv->Initialize(BOARD_VSYNC_INTERRUPT_GPIO_PIN, NULL);
    VSYNC_GPIO_drv->PowerControl(BOARD_VSYNC_INTERRUPT_GPIO_PIN, ARM_POWER_FULL);
    VSYNC_GPIO_drv->SetDirection(BOARD_VSYNC_INTERRUPT_GPIO_PIN, GPIO_PIN_DIRECTION_INPUT);
    pinconf_set(PORT_(BOARD_VSYNC_INTERRUPT_GPIO_PORT), BOARD_VSYNC_INTERRUPT_GPIO_PIN, PINMUX_ALTERNATE_FUNCTION_0, config_button);
    VSYNC_GPIO_drv->Initialize(BOARD_VSYNC_INTERRUPT_GPIO_PIN, camera_event_vsync_cb);
    VSYNC_GPIO_drv->Control(BOARD_VSYNC_INTERRUPT_GPIO_PIN, ARM_GPIO_ENABLE_INTERRUPT, &arg);
    */
    
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