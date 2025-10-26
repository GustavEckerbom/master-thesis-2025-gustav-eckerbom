#pragma GCC push_options
#pragma GCC optimize ("O3")

#include <stdbool.h>
#include "demo.h"
#include "Driver_common.h"
#include "image_buffer.h"
#include "tinyusb_wrapper.h"
#include "hwsem_wrapper.h"
#include "event_preprocessing.h"
#include "board.h"
#include "Driver_GPIO.h"

bool capture_new_frame      = true;
bool camera_running         = true;
bool stream_data_over_usb   = false;
bool serial_ready           = false;
static uint32_t accumulated_event_count = 0;
static uint8_t processed_frame_count = 0;

extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_LED_TOP_GPIO_PORT);
ARM_DRIVER_GPIO *gpioDrv = &ARM_Driver_GPIO_(BOARD_LED_TOP_GPIO_PORT);

int main(void) {

    uint32_t counter = 0;
    bool init_sem_failed = false;

    // init the semaphores for the camera buffer
    for (int i = 0; i < 9; i++) {
        hwsem_init_channel(i);
    }

    // Wait until the HE core has aquired the "initialization" semaphore
    while (!(hwsem_is_locked(8))){
    }

    // Lock the semaphore and start usb enumeration as soon as it's free
    if (ARM_DRIVER_OK != hwsem_lock(8)) {
        init_sem_failed = true;
    }
    // Gen decay tables
    generate_decay_tables();

    sys_busy_loop_init();

    // init frame buffers, surface and lookup table
    tusb_init();

    // Wait for USB to be connected
    while (!tud_cdc_connected()) {
        tud_task();
    }
    // tud_cdc_write_str("Initializeing Framebuffs\r\n");
    // tud_task();
    init_frame_buffers();
    // tud_cdc_write_str("Initializeing Surface\r\n");
    // tud_task();
    init_surface();

    for (int i = 0; i < 100; i++){ // Delay transmission boot by 1s to allow the python script to initialize in peace
        tud_task();
        sys_busy_loop_us(1000*10); // wait 10ms
    }

    memset(arm_app_rawImage_1, 0, CAMERA_BUFFER_SIZE);
    memset(arm_app_rawImage_2, 0, CAMERA_BUFFER_SIZE);
    memset(arm_app_rawImage_3, 0, CAMERA_BUFFER_SIZE);
    memset(arm_app_rawImage_4, 0, CAMERA_BUFFER_SIZE);

    if (init_sem_failed) {
        tud_cdc_write_str("Failed to lock hwsem at init\r\n");
        tud_task();
    } else {
        // tud_cdc_write_str("Locked hwsem at init\r\n");
        tud_task();
    }

    // Wait for HE core to lock in the first frame sem
    int32_t cnt = 999999;

    if (hwsem_unlock(8)!= ARM_DRIVER_OK) {
        tud_cdc_write_str("Failed to unlock hwsem at init\r\n");
        tud_task();
    }

      if (hwsem_lock(7)!= ARM_DRIVER_OK) {
        tud_cdc_write_str("Failed to lock hwsem for the deadtime \r\n");
        tud_task();
    }

    // tud_cdc_write_str("Will go into wait for hwsem\r\n");
    tud_task();

    while (!(hwsem_is_locked(0))) {
        // cnt++;
        // if (cnt == 100000){
        //     usb_blocking_printf("Waiting for HE core to lock sem %d \r\n", 0);
        //    cnt = 0;
        //}
        tud_task();
    }
    // tud_cdc_write_str("HWSEOM 0 now locked by HE core\r\n");
    // tud_task();

    while (get_demo_mode() == MODE_NONE) {
        handle_input_stream(); // or poll_usb_control_commands()
        tud_task();
    }
    
    while(1)  {

        // Wait for USB to be connected
        while (!tud_cdc_connected()) {
            tud_task();
        }

        if (camera_running) {
            process_ready_frames();         
        }
        
        tud_task();
        
        // Poll and check for index change in the buffer index
        poll_usb_control_commands();
    }

    return 0;
}


