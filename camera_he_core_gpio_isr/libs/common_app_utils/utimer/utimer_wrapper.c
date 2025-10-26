/**************************************************************************//**
 * @file     utimer_wrapper.c
 * @author   Gustav Eckerbom
 * @email    gustavec@kth.se
 * @version  V1.0.0
 * @date     31-January-2025
 * @brief    Wrapper to initialize and use the UTIMER driver.
 *           This file provides a wrapper to initialize and use the UTIMER driver.
 *           The CMSIS driver is used to initialize and use the UTIMER driver.
 * @bug      None
 * @Note     None
 ******************************************************************************/

#pragma GCC push_options
#pragma GCC optimize ("O3")

#include "utimer_wrapper.h"
#include "Driver_UTIMER.h"
#include "RTE_Components.h"
#include <stdio.h>
#include "Core_select.h"

#include CMSIS_device_header

bool utimer_initialized; // Flag to indicate if UTIMER is initialized
uint32_t last_ticks = 0; // Last tick count for delay calculation
extern ARM_DRIVER_UTIMER DRIVER_UTIMER0;
ARM_DRIVER_UTIMER *ptrUTimer = &DRIVER_UTIMER0;

volatile uint32_t cb_status_utim  =  0;

/* Callback function for UTIMER event */
static void utimer_cb_fun(uint8_t event) {
    if (event == ARM_UTIMER_EVENT_OVER_FLOW) {
        cb_status_utim++;
    }
}

/**
 * @brief Initializes the UTIMER channel.
 * @param channel UTIMER channel (e.g., 0 for UTIMER0)
 * @return 0 on success, -1 on failure
*/
int utimer_init(uint8_t channel) 
{
    int32_t ret;

    ret = ptrUTimer->Initialize(channel, utimer_cb_fun);
    if (ret != ARM_DRIVER_OK) return -1;

    ret = ptrUTimer->PowerControl(channel, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) return -1;

    ret = ptrUTimer->ConfigCounter (channel, ARM_UTIMER_MODE_BASIC, ARM_UTIMER_COUNTER_UP);
    if (ret != ARM_DRIVER_OK) {
       if (ret != ARM_DRIVER_OK) return -1;
    }
    return 0;
}

/**
 * @brief Provides a delay in milliseconds using the UTIMER.
 * @param channel UTIMER channel to use
 * @param ms Delay duration in milliseconds
 * @return 0 on success, -1 on failure
 */
int utimer_delay_ms(uint8_t channel, uint32_t ms) {
    int32_t ret;
    uint32_t count = MS_TO_TICKS(ms);

    ret = ptrUTimer->SetCount(channel, ARM_UTIMER_CNTR, 0);
    if (ret != ARM_DRIVER_OK) return -1;

    ret = ptrUTimer->SetCount(channel, ARM_UTIMER_CNTR_PTR, count);
    if (ret != ARM_DRIVER_OK) return -1;

    ret = ptrUTimer->Start(channel);
    if (ret != ARM_DRIVER_OK) return -1;

    while (cb_status_utim == 0);
    cb_status_utim = 0;

    ret = ptrUTimer->Stop(channel, ARM_UTIMER_COUNTER_CLEAR);
    return (ret == ARM_DRIVER_OK) ? 0 : -1;
}

/**
 * @brief Provides a delay in microseconds using a continuously running UTIMER.
 * @param channel UTIMER channel to use.
 * @param us Delay duration in microseconds.
 * @return 0 on success, -1 on failure.
 */
int utimer_delay_us(uint8_t channel, uint32_t us) {
    uint32_t tickstart, tickend;

    // Ensure the timer is running before using it
    if (!ptrUTimer->GetCount) {
        return -1;  // Timer is not initialized
    }

    // Get the current counter value
    tickstart = ptrUTimer->GetCount(channel, ARM_UTIMER_CNTR);

    // Calculate the target end time (handle wrap-around)
    tickend = tickstart + US_TO_TICKS(us);

    // Wait until the timer reaches the desired delay duration
    while ((int32_t)(ptrUTimer->GetCount(channel, ARM_UTIMER_CNTR) - tickstart) < US_TO_TICKS(us));

    return 0;  // Success
}

/**
 * @brief Provides a delay in microseconds using the UTIMER.
 * @param channel UTIMER channel to use
 * @param us Delay duration in microseconds
 * @return 0 on success, -1 on failure
 */
/*
int utimer_delay_us(uint8_t channel, uint32_t us) {
    int32_t ret;
    uint32_t count = US_TO_TICKS(us);

    ret = ptrUTimer->SetCount(channel, ARM_UTIMER_CNTR, 0);
    if (ret != ARM_DRIVER_OK) return -1;

    ret = ptrUTimer->SetCount(channel, ARM_UTIMER_CNTR_PTR, count);
    if (ret != ARM_DRIVER_OK) return -1;

    ret = ptrUTimer->Start(channel);
    if (ret != ARM_DRIVER_OK) return -1;

    while (cb_status_utim == 0);
    cb_status_utim = 0;

    ret = ptrUTimer->Stop(channel, ARM_UTIMER_COUNTER_CLEAR);
    return (ret == ARM_DRIVER_OK) ? 0 : -1;
}
*/

/**
 * @brief Deinitializes the UTIMER.
 * @param channel UTIMER channel to deinitialize
 * @return 0 on success, -1 on failure
 */
int utimer_deinit(uint8_t channel) {
    int32_t ret;

    ret = ptrUTimer->PowerControl(channel, ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) return -1;

    ret = ptrUTimer->Uninitialize(channel);
    return (ret == ARM_DRIVER_OK) ? 0 : -1;
}

// Callback function pointer
static void (*user_callback)(void) = NULL;

static void utimer_callback_handler(uint8_t event) {
    if (event == ARM_UTIMER_EVENT_OVER_FLOW && user_callback) {
        user_callback();  // Call user-defined callback function
    }
}

/**
 * @brief Configures UTIMER to count continuously up to a given tick value.
 *        Triggers an interrupt when it reaches the tick count.
 * @param channel UTIMER channel to use
 * @param ticks Tick count at which callback should be triggered
 * @param callback Function to call when tick count is reached
 * @return 0 on success, -1 on failure
 */
int utimer_config_continuous(uint8_t channel, uint32_t ticks, void (*callback)(void)) {
    int32_t ret;

    // Store user callback function
    user_callback = callback;

    // Initialize UTIMER
    ret = ptrUTimer->Initialize(channel, utimer_callback_handler);
    if (ret != ARM_DRIVER_OK) return -1;

    // Power up UTIMER
    ret = ptrUTimer->PowerControl(channel, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) return -1;

    // Configure UTIMER in continuous counting mode
    ret = ptrUTimer->ConfigCounter(channel, ARM_UTIMER_MODE_BASIC, ARM_UTIMER_COUNTER_UP);
    if (ret != ARM_DRIVER_OK) return -1;

    // Set tick count at which interrupt should trigger
    ret = ptrUTimer->SetCount(channel, ARM_UTIMER_CNTR_PTR, ticks);
    if (ret != ARM_DRIVER_OK) return -1;

    // Start the UTIMER
    ret = ptrUTimer->Start(channel);
    return (ret == ARM_DRIVER_OK) ? 0 : -1;
}



/**
 * @brief Save the current UTIMER tick value as the reference starting point.
 * @param channel UTIMER channel to use
 */
void utimer_set_reference(uint8_t channel) {
    last_ticks = ptrUTimer->GetCount(channel, ARM_UTIMER_CNTR);
}


/**
* @brief Get the elapsed time in milliseconds since the last call to this function. 
* @param channel UTIMER channel to use
* @return Elapsed time in milliseconds
*/
uint32_t utimer_get_delta_time_ms(uint8_t channel) {
    uint32_t current_tick = ptrUTimer->GetCount(channel, ARM_UTIMER_CNTR);
    uint32_t delta_ticks = current_tick - last_ticks;

    return delta_ticks / (UTIMER_CLK_FREQ / 1000);  // Convert ticks to ms
}

/**
* @brief Get the elapsed time in milliseconds since the last call to this function. 
* @param channel UTIMER channel to use
* @return Elapsed time in us
*/
uint32_t utimer_get_delta_time_us(uint8_t channel) {
    uint32_t current_tick = ptrUTimer->GetCount(channel, ARM_UTIMER_CNTR);
    uint32_t delta_ticks = current_tick - last_ticks;

    return delta_ticks / (UTIMER_CLK_FREQ / (1000*1000));  // Convert ticks to us
}

/* New: reset timer to 0 */
int utimer_reset(uint8_t channel) {
    return ptrUTimer->SetCount(channel, ARM_UTIMER_CNTR, 0);
}

/* New: start timer manually */
int utimer_start(uint8_t channel) {
    return ptrUTimer->Start(channel);
}

/* New: stop timer manually */
int utimer_stop(uint8_t channel) {
    return ptrUTimer->Stop(channel, ARM_UTIMER_COUNTER_CLEAR);
}

int utimer_init_freerun(uint8_t channel)
{
    int32_t ret;
    uint8_t utimer_channel;
    
    // Map input to actual UTIMER channel macro
    switch (channel) {
        case 0: utimer_channel = ARM_UTIMER_CHANNEL0; break;
        case 1: utimer_channel = ARM_UTIMER_CHANNEL1; break;
        case 2: utimer_channel = ARM_UTIMER_CHANNEL2; break;
        case 3: utimer_channel = ARM_UTIMER_CHANNEL3; break;
        case 4: utimer_channel = ARM_UTIMER_CHANNEL4; break;
        case 5: utimer_channel = ARM_UTIMER_CHANNEL5; break;
        case 6: utimer_channel = ARM_UTIMER_CHANNEL6; break;
        case 7: utimer_channel = ARM_UTIMER_CHANNEL7; break;
        default:
            printf("Invalid UTIMER channel: %d\n", channel);
            return -1;
    }

    // Step 1: Initialize
    ret = ptrUTimer->Initialize(channel, utimer_cb_fun);  // No callback
    if (ret != ARM_DRIVER_OK) {
        printf("UTIMER channel %d initialization failed!\r\n", channel);
        return -1;
    }

    // Step 2: Power on
    ret = ptrUTimer->PowerControl(channel, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("UTIMER channel %d power control failed!\r\n", channel);
        return -1;
    }

    // Step 3: Configure counter in basic up-counting mode
    ret = ptrUTimer->ConfigCounter(channel, ARM_UTIMER_MODE_BASIC, ARM_UTIMER_COUNTER_UP);
    if (ret != ARM_DRIVER_OK) {
        printf("UTIMER channel %d mode config failed!\r\n", channel);
        return -1;
    }

    // Step 4: Set counter start at 0
    ret = ptrUTimer->SetCount(channel, ARM_UTIMER_CNTR, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("UTIMER channel %d set start count failed!\r\n", channel);
        return -1;
    }

    // Step 5: Set wrap value (overflow) to max 32-bit to run "forever"
    ret = ptrUTimer->SetCount(channel, ARM_UTIMER_CNTR_PTR, 0xFFFFFFFF);
    if (ret != ARM_DRIVER_OK) {
        printf("UTIMER channel %d set overflow failed!\r\n", channel);
        return -1;
    }

    // Step 6: Start the timer
    ret = ptrUTimer->Start(channel);
    if (ret != ARM_DRIVER_OK) {
        printf("UTIMER channel %d start failed!\r\n", channel);
        return -1;
    }

    printf("UTIMER channel %d configured in free-running basic mode.\r\n", channel);
    return 0;
}