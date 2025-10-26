/**************************************************************************//**
 * @file     utimer_wrapper.h
 * @author   Gustav Eckerbom
 * @email    gustavec@kth.se
 * @version  V1.0.0
 * @date     31-January-2025
 * @brief    Header file for the UTIMER wrapper.
 *           Provides functions for initializing and coonfiguring and 
 *           deinitializing the UTIMER.
 * @bug      None
 * @Note     None
 ******************************************************************************/

#ifndef UTIMER_WRAPPER_H
#define UTIMER_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "Driver_UTIMER.h"

#define UTIMER_CHANNEL_0  0             // Default UTIMER channel
#define UTIMER_CLK_FREQ   400000000     // UTIMER runs at 400 MHz (2.5 ns resolution)

#define US_TO_TICKS(us)    ((us * (UTIMER_CLK_FREQ / 1000000)))
#define MS_TO_TICKS(ms)    ((ms * (UTIMER_CLK_FREQ / 1000)))

extern uint32_t last_ticks;

extern ARM_DRIVER_UTIMER *ptrUTimer;


extern bool utimer_initialized; // Flag to indicate if UTIMER is initialized
/**
 * @brief Initializes the UTIMER channel.
 * @param channel UTIMER channel (e.g., 0 for UTIMER0)
 * @return 0 on success, -1 on failure
 */
int utimer_init(uint8_t channel);

/**
 * @brief Provides a delay in milliseconds using the UTIMER.
 * @param channel UTIMER channel to use
 * @param ms Delay duration in milliseconds
 * @return 0 on success, -1 on failure
 */
int utimer_delay_ms(uint8_t channel, uint32_t ms);

/**
 * @brief Provides a delay in microseconds using the UTIMER.
 * @param channel UTIMER channel to use
 * @param us Delay duration in microseconds
 * @return 0 on success, -1 on failure
 */
int utimer_delay_us(uint8_t channel, uint32_t us);

/**
 * @brief Deinitializes the UTIMER.
 * @param channel UTIMER channel to deinitialize
 * @return 0 on success, -1 on failure
 */
int utimer_deinit(uint8_t channel);

int utimer_config_continuous(uint8_t channel, uint32_t ticks, void (*callback)(void));

uint32_t utimer_get_delta_time_ms(uint8_t channel);

int utimer_reset(uint8_t channel);

int utimer_start(uint8_t channel);

int utimer_stop(uint8_t channel);

/**
 * @brief Fully configures UTIMER0 in free-running mode at given frequency.
 *        Initializes, powers, sets divider, configures up-counting, and starts the timer.
 * @return 0 on success, -1 on failure
 */
int utimer_init_freerun(uint8_t channel);

void utimer_set_reference(uint8_t channel);

uint32_t utimer_get_delta_time_us(uint8_t channel);



#ifdef __cplusplus
}
#endif

#endif /* UTIMER_WRAPPER_H */
