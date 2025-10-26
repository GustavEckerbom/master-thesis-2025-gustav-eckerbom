/**************************************************************************//**
 * @file     lptimer_wrapper.h
 * @author   Gustav Eckerbom
 * @email    gustavec@kth.se
 * @version  V1.0.0
 * @date     31-January-2025
 * @brief    Header file for the LPTIMER wrapper.
 *           Provides function prototypes for initializing, delaying, and 
 *           deinitializing the LPTIMER.
 * @bug      None
 * @Note     None
 ******************************************************************************/

#ifndef LPTIMER_WRAPPER_H
#define LPTIMER_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define LPTIMER_CHANNEL_0  0  // Default LPTIMER channel

/**
 * @brief Initializes the LPTIMER channel.
 * @param channel LPTIMER channel (e.g., 0 for LPTIMER0)
 * @return 0 on success, -1 on failure
 */
int lptimer_init(uint8_t channel);

/**
 * @brief Provides a delay in milliseconds using the LPTIMER.
 * @param channel LPTIMER channel to use
 * @param ms Delay duration in milliseconds
 * @return 0 on success, -1 on failure
 */
int lptimer_delay_ms(uint8_t channel, uint32_t ms);

/**
 * @brief Provides a delay in microseconds using the LPTIMER.
 * @param channel LPTIMER channel to use
 * @param us Delay duration in microseconds
 * @return 0 on success, -1 on failure
 */
int lptimer_delay_us(uint8_t channel, uint32_t us);

/**
 * @brief Deinitializes the LPTIMER.
 * @param channel LPTIMER channel to deinitialize
 * @return 0 on success, -1 on failure
 */
int lptimer_deinit(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* LPTIMER_WRAPPER_H */
