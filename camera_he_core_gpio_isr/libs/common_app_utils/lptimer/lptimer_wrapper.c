
/**************************************************************************//**
 * @file     lptimer_wrapper.c
 * @author   Gustav Eckerbom
 * @email    gustavec@kth.se
 * @version  V1.0.0
 * @date     31-January-2025
 * @brief    Wrapper to initialize and use the LPTIMER driver.
 *           This file provides a wrapper to initialize and use the LPTIMER driver.
 *           The CMSIS driver is used to initialize and use the LPTIMER driver.
 * @bug      None
 * @Note     None
 ******************************************************************************/
#pragma GCC push_options
#pragma GCC optimize ("O3")

#include "Driver_LPTIMER.h"
#include "RTE_Components.h"
#include CMSIS_device_header


extern ARM_DRIVER_LPTIMER DRIVER_LPTIMER0;
ARM_DRIVER_LPTIMER *ptrDrv = &DRIVER_LPTIMER0;

#define LPTIMER_CLK_FREQ   32768  // Default 32.768kHz clock
#define US_TO_TICKS(us)    ((us * LPTIMER_CLK_FREQ) / 1000000)
#define MS_TO_TICKS(ms)    ((ms * LPTIMER_CLK_FREQ) / 1000)

volatile uint32_t cb_status_lptim  =  0;
static void lptimer_cb_fun (uint8_t event)
{
    if (event == ARM_LPTIMER_EVENT_UNDERFLOW)
    {
        cb_status_lptim++;
    }
}

/**
 * @brief Initializes the LPTIMER channel.
 * @param channel LPTIMER channel (e.g., 0 for LPTIMER0)
 * @return 0 on success, -1 on failure
 */
int lptimer_init(uint8_t channel) {
    int32_t ret;

    ret = ptrDrv->Initialize(channel, lptimer_cb_fun);
    if (ret != ARM_DRIVER_OK) return -1;

    ret = ptrDrv->PowerControl(channel, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) return -1;

    return 0;
}

/**
 * @brief Provides a delay in milliseconds using the LPTIMER.
 * @param channel LPTIMER channel to use
 * @param ms Delay duration in milliseconds
 * @return 0 on success, -1 on failure
 */
int lptimer_delay_ms(uint8_t channel, uint32_t ms) {
    int32_t ret;
    uint32_t count = MS_TO_TICKS(ms);

    ret = ptrDrv->Control(channel, ARM_LPTIMER_SET_COUNT1, &count);
    if (ret != ARM_DRIVER_OK) return -1;

    ret = ptrDrv->Start(channel);
    if (ret != ARM_DRIVER_OK) return -1;

    // Wait for timer to expire
    while (cb_status_lptim == 0);
    cb_status_lptim = 0;

    ret = ptrDrv->Stop(channel);
    return (ret == ARM_DRIVER_OK) ? 0 : -1;
}


/** 
 * @brief Provides a delay in microseconds using the LPTIMER.
 * @param channel LPTIMER channel to use
 * @param us Delay duration in microseconds
 * @return 0 on success, -1 on failure
 */
int lptimer_delay_us(uint8_t channel, uint32_t us) {
    int32_t ret;
    uint32_t count = US_TO_TICKS(us);

    ret = ptrDrv->Control(channel, ARM_LPTIMER_SET_COUNT1, &count);
    if (ret != ARM_DRIVER_OK) return -1;

    ret = ptrDrv->Start(channel);
    if (ret != ARM_DRIVER_OK) return -1;

    // Wait for timer to expire
    while (cb_status_lptim == 0);
    cb_status_lptim = 0;

    ret = ptrDrv->Stop(channel);
    return (ret == ARM_DRIVER_OK) ? 0 : -1;
}


/**
 * @brief Deinitializes the LPTIMER.
 * @param channel LPTIMER channel to deinitialize
 * @return 0 on success, -1 on failure
 */
int lptimer_deinit(uint8_t channel) {
    int32_t ret;

    ret = ptrDrv->PowerControl(channel, ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) return -1;

    ret = ptrDrv->Uninitialize(channel);
    return (ret == ARM_DRIVER_OK) ? 0 : -1;
}