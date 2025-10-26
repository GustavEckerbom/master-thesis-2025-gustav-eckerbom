
#pragma GCC push_options
#pragma GCC optimize ("O3")

#include "hwsem_wrapper.h"
#include "Driver_HWSEM.h"
#include "Core_select.h"
#include <stdio.h>
#include "tinyusb_wrapper.h"
#include "image_buffer.h"


#define HWSEM_CB_EVENT (1U << 0)

static volatile uint32_t event_flags[MAX_HWSEM_CHANNELS] = {0};

// Declare driver handles for each channel
extern ARM_DRIVER_HWSEM DRIVER_HWSEM0;
extern ARM_DRIVER_HWSEM DRIVER_HWSEM1;
extern ARM_DRIVER_HWSEM DRIVER_HWSEM2;
extern ARM_DRIVER_HWSEM DRIVER_HWSEM3;
extern ARM_DRIVER_HWSEM DRIVER_HWSEM4;
extern ARM_DRIVER_HWSEM DRIVER_HWSEM5;
extern ARM_DRIVER_HWSEM DRIVER_HWSEM6;
extern ARM_DRIVER_HWSEM DRIVER_HWSEM7;
extern ARM_DRIVER_HWSEM DRIVER_HWSEM8; // For init sequence 


// Lookup table of driver instances per channel
static ARM_DRIVER_HWSEM *const hwsem_drivers[MAX_HWSEM_CHANNELS] = {
    &DRIVER_HWSEM0,
    &DRIVER_HWSEM1,
    &DRIVER_HWSEM2,
    &DRIVER_HWSEM3,
    &DRIVER_HWSEM4,
    &DRIVER_HWSEM5,
    &DRIVER_HWSEM6,
    &DRIVER_HWSEM7,
    &DRIVER_HWSEM8, // For init sequence
};

bool hwsem_available(uint8_t sem_num) {
    if (sem_num >= MAX_HWSEM_CHANNELS) return false;
    // Store the flag 
    bool ret = (event_flags[sem_num] & HWSEM_CB_EVENT) != 0;
    // Clear the flag
    event_flags[sem_num] &= ~HWSEM_CB_EVENT;
    // Return the flag
    return ret;
}

// Shared callback for all channels
static void hwsem_cb_event(int32_t event, uint8_t sem_id) {
    if (sem_id < MAX_HWSEM_CHANNELS && (event & HWSEM_AVAILABLE_CB_EVENT)) {
        event_flags[sem_id] |= HWSEM_CB_EVENT;
    }
}

int32_t hwsem_init_channel(uint8_t sem_num) {
    if (sem_num >= MAX_HWSEM_CHANNELS) return -1;
    event_flags[sem_num] = 0;  
    return hwsem_drivers[sem_num]->Initialize(hwsem_cb_event);
}


int32_t hwsem_deinit(uint8_t sem_num) {
    if (sem_num >= MAX_HWSEM_CHANNELS) return -1;
    return hwsem_drivers[sem_num]->Uninitialize();
}

int32_t hwsem_trylock(uint8_t sem_num) {
    if (sem_num >= MAX_HWSEM_CHANNELS) return -1;
    return hwsem_drivers[sem_num]->TryLock();
}

int32_t hwsem_cam_custom_lock(uint8_t sem_num){
    if (sem_num >= MAX_HWSEM_CHANNELS) {
        return -1;
    }

    int32_t ret = hwsem_drivers[sem_num]->TryLock();

    if (ret == ARM_DRIVER_ERROR) {
        // Outright failure
        #ifdef HP_CORE
            usb_blocking_printf("Failed to lock hwsem %d\r\n", sem_num);
            // Poll and check for index change in the buffer index
            poll_usb_control_commands();
            tud_task();
        #elif defined(HE_CORE)
            printf("Failed to lock hwsem %d\r\n", sem_num);
        #endif
            return -1;
    }

    // Spin until lock or error
    while (ret == ARM_DRIVER_ERROR_BUSY) {
        uint32_t counter = 0;
        // wait for the callback
        while (!(event_flags[sem_num] & HWSEM_CB_EVENT)){
            
            #ifdef HP_CORE
                tud_task();
            #endif
            #ifdef HE_CORE
            // Check if new frame arrived
            if (new_frame_available){
                printf("Failed to lock hwsem before new frame was available %d\r\n", sem_num);
                return -1;
            }
            #endif
        }

        event_flags[sem_num] &= ~HWSEM_CB_EVENT;
        
        ret = hwsem_drivers[sem_num]->TryLock();
        
        #ifdef HP_CORE
            tud_task();
            counter++;
            if (counter == 1000) {
                usb_blocking_printf("Waiting for hwsem %d\r\n", sem_num);
                tud_task();
            }
        #endif
    }

    return (ret == ARM_DRIVER_OK) ? 0 : -1;
}

int32_t hwsem_lock(uint8_t sem_num) {
    if (sem_num >= MAX_HWSEM_CHANNELS) {
        return -1;
    }

    int32_t ret = hwsem_drivers[sem_num]->TryLock();

    if (ret == ARM_DRIVER_ERROR) {
        // outright failure
    
    #ifdef HP_CORE
        usb_blocking_printf("Failed to lock hwsem %d\r\n", sem_num);
        tud_task();

    #elif defined(HE_CORE)
        printf("Failed to lock hwsem %d\r\n", sem_num);
    #endif
        return -1;
    }

    // spin until lock or error
    while (ret == ARM_DRIVER_ERROR_BUSY) {
        uint32_t counter = 0;
        // wait for the callback
        while (!(event_flags[sem_num] & HWSEM_CB_EVENT)){
            #ifdef HP_CORE
                tud_task();
            #endif
        }

        event_flags[sem_num] &= ~HWSEM_CB_EVENT;
        ret = hwsem_drivers[sem_num]->TryLock();
        #ifdef HP_CORE
            tud_task();
            counter++;
            if (counter == 1000) {
                usb_blocking_printf("Waiting for hwsem %d\r\n", sem_num);
                tud_task();
            }
        #endif
    }

    return (ret == ARM_DRIVER_OK) ? 0 : -1;
}

int32_t hwsem_unlock(uint8_t sem_num) {
    if (sem_num >= MAX_HWSEM_CHANNELS) return -1;
    return (hwsem_drivers[sem_num]->Unlock() == ARM_DRIVER_OK) ? 0 : -1;
}

int32_t hwsem_get_count(uint8_t sem_num) {
    if (sem_num >= MAX_HWSEM_CHANNELS) {
        return -1;
    }
    /* returns either a non-negative count, or ARM_DRIVER_ERROR */
    return hwsem_drivers[sem_num]->GetCount();
}

bool hwsem_is_locked(uint8_t sem_num) {
    int32_t count = hwsem_get_count(sem_num);
    if (count < 0) {
        /* treat invalid channel as “not locked” */
        return false;
    }
    return (count > 0);
}