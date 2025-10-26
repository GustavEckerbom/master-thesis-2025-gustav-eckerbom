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
#include "services_lib_api.h"
#include "services_lib_interface.h"
#include "mhu_driver.h"
#include "board_gpio_handles.h"
}

#include CMSIS_device_header
#include "camera_capture.hpp"
#define SE_MHU0_RECV_BASE 0x40040000
#define SE_MHU0_SEND_BASE 0x40050000
#define MHU_M55_SE_MHU0   0
#define MAX_TIMEOUT       0x01000000


bool capture_new_frame = true;
bool stream_data_over_usb = false;
bool new_frame_ready = false;
bool serial_ready = false;

// user pushbutton flag 
static bool pushflag = false;

static uint32_t accumulated_event_count = 0;
static uint8_t processed_frame_count = 0;

static mhu_driver_in_t mhu_in;
static mhu_driver_out_t mhu_out;

static uint32_t sender_base[1]   = { SE_MHU0_SEND_BASE };
static uint32_t receiver_base[1] = { SE_MHU0_RECV_BASE };

static void init_mhu(void)
{
    mhu_in.sender_base_address_list   = sender_base;
    mhu_in.receiver_base_address_list = receiver_base;
    mhu_in.mhu_count                  = 1;
    mhu_in.send_msg_acked_callback    = SERVICES_send_msg_acked_callback;
    mhu_in.rx_msg_callback            = SERVICES_rx_msg_callback;
    mhu_in.debug_print                = SERVICES_print;

    MHU_driver_initialize(&mhu_in, &mhu_out);
}

void divide_ACLK(uint32_t divider)
{
    init_mhu();
    SERVICES_Setup(mhu_out.send_message, MAX_TIMEOUT);

    uint32_t handle = SERVICES_register_channel(MHU_M55_SE_MHU0, 0);
    SERVICES_wait_ms(10);

    uint32_t error;
    uint32_t error_code;

    // Set ACLK divider (affects AXI/AHB/APB indirectly)
    if (SERVICES_clocks_set_divider(handle, DIVIDER_ACLK, divider, &error_code) != SERVICE_SUCCESS ||
        error_code != SERVICE_SUCCESS) {
        SERVICES_print("[ERROR] Failed to set ACLK divider. err=0x%08X\n", error_code);
    } else {
        SERVICES_print("[INFO] ACLK divider set to 4\n");
    }
}

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
        BOARD_LED_TOP_GPIO_GPIOdrv->SetValue(BOARD_LED_TOP_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
        wait_milliseconds(100);
        BOARD_LED_TOP_GPIO_GPIOdrv->SetValue(BOARD_LED_TOP_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
        wait_milliseconds(100);
    }
}

void blink_led_fast()
{
    for (int i = 0; i < 10; i++) {
        BOARD_LED_TOP_GPIO_GPIOdrv->SetValue(BOARD_LED_TOP_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
        wait_milliseconds(40);
        BOARD_LED_TOP_GPIO_GPIOdrv->SetValue(BOARD_LED_TOP_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
        wait_milliseconds(40);
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

static void user_push_cb(uint32_t event){
    if (event & ARM_GPIO_IRQ_EVENT_EXTERNAL) {
        pushflag = true;
    }
}

int main(void)
{
    int32_t         ret = 0;
    // ret = config_run_power_domain();    
 
    uint32_t config_button =    PADCTRL_READ_ENABLE |
                                PADCTRL_SCHMITT_TRIGGER_ENABLE;

    uint32_t irq_cfg =  ARM_GPIO_IRQ_POLARITY_HIGH |
                        ARM_GPIO_IRQ_SENSITIVE_EDGE;
                        
    BOARD_USER_BUTTON_GPIO_GPIOdrv->Initialize(BOARD_USER_PUSH_GPIO_PIN, user_push_cb);
    BOARD_USER_BUTTON_GPIO_GPIOdrv->PowerControl(BOARD_USER_PUSH_GPIO_PIN, ARM_POWER_FULL);
    BOARD_USER_BUTTON_GPIO_GPIOdrv->SetDirection(BOARD_USER_PUSH_GPIO_PIN, GPIO_PIN_DIRECTION_INPUT);

    // Set pinmux to GPIO mode with no pull-up/down
    pinconf_set(PORT_(BOARD_USER_PUSH_GPIO_PORT),
                BOARD_USER_PUSH_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_0,
                config_button);

    // Make sure debounce disabled
    uint32_t debounce_disable = 0;
    
    BOARD_USER_BUTTON_GPIO_GPIOdrv->Control(BOARD_USER_PUSH_GPIO_PIN, ARM_GPIO_CONFIG_DEBOUNCE, &debounce_disable);

    // Enable GPIO interrupt on rising edge
    BOARD_USER_BUTTON_GPIO_GPIOdrv->Control(BOARD_USER_PUSH_GPIO_PIN,
                            ARM_GPIO_ENABLE_INTERRUPT,
                            &irq_cfg);
    
                            
while (pushflag == false) {
        // Wait for user push button to be pressed
    }

    pushflag = false;

    //divide_ACLK(31);
    
    ret = config_run_power_domain();      

    while (pushflag == false) {
        // Wait for user push button to be pressed
    }

    pushflag = false;

    for (int i = 0; i < 9; i++) {
        hwsem_init_channel(i);
    }

    int32_t         counter = 0;
    uint32_t        error_code = SERVICES_REQ_SUCCESS;
    uint32_t        service_error_code;
    off_profile_t   offp = {0};
    run_profile_t   runp = {0};
    
    BOARD_pinconf();
    
    BOARD_Pinmux_Init();
    
    sys_busy_loop_init();

    // fault_dump_enable(true);
    
    // tracelib_init(NULL, uart_callback);

    // Toggle the IO pin a few times
    for (int i = 0; i < 100; i++) {
        BOARD_IOHEADER_GPIOdrv->SetValue(BOARD_GPIO_PINHEAD_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        sys_busy_loop_us(100*10);
    }

    init_frame_buffers();
    
    // printf("App starts\r\n");

    if (0 != arm::app::CameraCaptureInit()) {
        //printf("Failed to initalise camera\n");
        return 2;
    }

    //printf("Camera initialised\n\r");

    frame_buffers[0].status = BUFFER_WRITING;

    // Start the camera stream
    if (capture_new_frame) {
        if (0 != arm::app::CameraCaptureStartStream(frame_buffers[0].data)) {
            //printf("Failed to start camera\n\r");
            return 2;
        }
    }
        
        if(!ret){
            //printf("Run config set successfully\r\n");
        }

    // Shut down the other cores and power domains

    // Printf("Enter main loop\n\r");
    
    while(1) {  

        if (counter == 1000)
        {
            // Print to show the HE core is not in deadlock or something
            //printf("in main loop.\r\n");
        }

        if (number_of_frames > 0) {
            ////printf("Frame captured\n\r");
            number_of_frames--;
        }

        if (new_frame_available){
            arm::app::LockNextFrame();
            // BOARD_IOHEADER_GPIOdrv->SetValue(BOARD_GPIO_PINHEAD_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        }

        if (arm::app::camera_status.camera_error) {
            //printf("Camera error occurred, stopping stream\n\r");
            arm::app::CameraCaptureStopStreaming();
            blink_led_fast();
            arm::app::camera_status.camera_on = false;
            arm::app::camera_status.camera_error = false;
            arm::app::camera_status.frame_complete = false;
            continue;
        }

        counter++;
        
    }
}