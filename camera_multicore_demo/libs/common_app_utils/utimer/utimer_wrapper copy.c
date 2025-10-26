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

#include "utimer_wrapper.h"
#include "Driver_UTIMER.h"
#include "RTE_Components.h"
#include CMSIS_device_header

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