/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution, and modification of this code are permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement.
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */


extern "C" {
#include "DEBUG.h"
#include <stdio.h>
#include "system_utils.h"
#include "Driver_GPIO.h"
#include "board.h"
#include "uart_tracelib.h"
#include "fault_handler.h"
#include "pinconf.h"
#include <inttypes.h>
// #include "pm.h"
#include "se_services_port.h"
#include "power_control_wrapper.h"
#include "image_buffer.h"
#include "event_preprocessing.h"
#include "RTE_Components.h"
#include "Utimer_wrapper.h"
#include "Core_select.h"
#include "hwsem_wrapper.h"
#include "genx320.h"
}

#include CMSIS_device_header
#include "camera_capture.hpp"
//#define DEVKIT_GEN2
/*
/* Get GPIO driver for port 6 */
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_LED_TOP_GPIO_PORT);

// Camera module Power supply enable pins
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_PSU_EN_1V1_GPIO_PORT);
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_PSU_EN_1V8_GPIO_PORT);
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAM_PSU_EN_2V5_GPIO_PORT);
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_GPIO_PINHEAD_GPIO_PORT);


ARM_DRIVER_GPIO *gpioDrv = &ARM_Driver_GPIO_(BOARD_LED_TOP_GPIO_PORT);

// IO header
ARM_DRIVER_GPIO *BOARD_IOHEADER_GPIOdrv = &ARM_Driver_GPIO_(BOARD_GPIO_PINHEAD_GPIO_PORT);

bool capture_new_frame = true;
bool stream_data_over_usb = false;
bool new_frame_ready = false;
bool serial_ready = false;
static uint32_t accumulated_event_count = 0;
static uint8_t processed_frame_count = 0;

static void uart_callback(uint32_t event) {}


void wait_seconds(int seconds)
{
    for (int i = 0; i < seconds*100; i++) {
        sys_busy_loop_us(100*100);
    }
}

void wait_milliseconds(int millisecs)
{
    for (int i = 0; i < millisecs; i++) {
        sys_busy_loop_us(1000);
    }
}

void blink_led()
{
    for (int i = 0; i < 10; i++) {
        gpioDrv->SetValue(BOARD_LED_TOP_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
        wait_milliseconds(100);
        gpioDrv->SetValue(BOARD_LED_TOP_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
        wait_milliseconds(100);
    }
}


void TOGGLE_IO_HEADER()
{
    for (int i = 0; i < 10; i++) {
        BOARD_IOHEADER_GPIOdrv->SetValue(BOARD_GPIO_PINHEAD_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
        wait_milliseconds(100);
        BOARD_IOHEADER_GPIOdrv->SetValue(BOARD_GPIO_PINHEAD_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
        wait_milliseconds(100);
    }
}


int main(void)
{
    for (int i = 0; i < 9; i++) {
        hwsem_init_channel(i);
    }

    // Lock semaphore to proceed with init sequence
    hwsem_lock(8);

    int32_t         counter = 0;
    int32_t         ret = 0;
    uint32_t        error_code = SERVICES_REQ_SUCCESS;
    uint32_t        service_error_code;
    off_profile_t   offp = {0};
    run_profile_t   runp = {0};
    
    BOARD_pinconf();
    
    BOARD_Pinmux_Init();
    
    sys_busy_loop_init();

    fault_dump_enable(true);
    
    tracelib_init(NULL, uart_callback);
    
    utimer_init_freerun(UTIMER_CHANNEL);

    //blink_led();

    //TOGGLE_IO_HEADER();

    // Toggle the IO pin a few times
    for (int i = 0; i < 100; i++) {
        BOARD_IOHEADER_GPIOdrv->SetValue(BOARD_GPIO_PINHEAD_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        sys_busy_loop_us(100*10);
    }

    printf("HWSEM 8 locked by HE core and configuration of board finished\r\n");

    #if defined(DEVKIT_GEN2)
        BOARD_UART_Init();
    #endif
    
    init_frame_buffers();
    
    //dump_frame_buffers();

    utimer_set_reference(UTIMER_CHANNEL);

    sys_busy_loop_us(2000);
    
    uint32_t busy_time_us = utimer_get_delta_time_us(UTIMER_CHANNEL);
    
    printf("This print should be 2000 us if the timer is correctly configured: %lu us\n\r", busy_time_us);

    printf("App starts\r\n");

    // dump_frame_buffers();
    
    if (0 != arm::app::CameraCaptureInit()) {
        printf("Failed to initalise camera\n");
        return 2;
    }
    printf("Camera initialised\n\r");
    
    // Signal HP core it's safe to start and try to enumerate the UBS port
    if (ARM_DRIVER_OK != hwsem_unlock(8)) {
        printf("Failed to unlock hwsem at init\n\r");
    }
    
    // Wait for it to be aquired by the HP core
    while(!(hwsem_is_locked(8))) {
        printf("Waiting for the HP core to lock init sem\r\n");
    }
    
    // Blocking lock until the HP core has initialized the USB periperhal
    if (ARM_DRIVER_OK != hwsem_lock(8)) {
        printf("Failed to lock hwsem at init\n\r");
    }

    printf("USB ready and init sem locked by the HE core\n\r");

    // Lock hwsem for the first frame
    if (ARM_DRIVER_OK != hwsem_lock(0)) {
        printf("Failed to lock hwsem for first frame\n\r");
    }
    // Indicate the first frame is will be written to soon
    else {
        frame_buffers[0].status = BUFFER_QUEUED;
    }
    
    printf("Locked hwsem for first frame\n\r");

    // Start the camera stream
    if (capture_new_frame) {
        if (0 != arm::app::CameraCaptureStartStream(frame_buffers[0].data)) {
            printf("Failed to start camera\n\r");
            return 2;
        }
    }

    printf("Enter main loop\n\r");
               
    while(1) {

        // if try lock failed and hwsem 7 has been released since the last try
        if (hwsem_available(7)){
            // Update the deadtime between frames
            SCB_InvalidateDCache_by_Addr((uint32_t *)&dead_time_between_frames_HE_core, sizeof(dead_time_between_frames_HE_core));
            __DSB();  // Ensure memory read happens after cache invalidation
            psee_sensor_set_deadtime(dead_time_between_frames_HE_core);
            hwsem_unlock(7);
            printf("Deadtime set to %lu us\r\n", dead_time_between_frames_HE_core);
        }

        if (counter == 1000)
        {
            // Print to show the HE core is not in deadlock
            printf("in main loop.\r\n");
        }

        if (number_of_frames > 0) {
            //printf("Frame captured\n\r");
            number_of_frames--;
        }

        if (new_frame_available){
            arm::app::LockNextFrame();
        }

        if((hwsem_trylock(7) == ARM_DRIVER_OK)) {
            // Update the deadtime between frames
            SCB_InvalidateDCache_by_Addr((uint32_t *)&dead_time_between_frames_HE_core, sizeof(dead_time_between_frames_HE_core));
            __DSB();  // Ensure memory read happens after cache invalidation
            psee_sensor_set_deadtime(dead_time_between_frames_HE_core);
            hwsem_unlock(7);
            //printf("Deadtime set to %lu us\r\n", dead_time_between_frames_HE_core);
        }
        counter++;
    }
}