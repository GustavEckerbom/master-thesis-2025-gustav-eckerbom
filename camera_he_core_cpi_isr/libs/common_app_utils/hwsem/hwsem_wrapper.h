#ifndef HWSEM_WRAPPER_H
#define HWSEM_WRAPPER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of supported hardware semaphore channels.
 * Adjust this value if your platform supports more or fewer channels.
 */
#define MAX_HWSEM_CHANNELS 9


bool hwsem_available(uint8_t sem_num);

/**
 * @brief Initialize all available HW semaphore channels.
 *
 * This function initializes each hardware semaphore driver instance
 * with a shared callback for handling lock availability events.
 *
 * @return 0 on success, -1 if any channel failed to initialize.
 */
int32_t hwsem_init_channel(uint8_t sem_num);

/**
 * @brief Uninitialize a specific hardware semaphore channel.
 *
 * @param sem_num The semaphore channel to deinitialize (0 to MAX_HWSEM_CHANNELS - 1).
 * @return 0 on success, -1 on invalid channel or failure.
 */
int32_t hwsem_deinit(uint8_t sem_num);

/**
 * @brief Attempt to lock a semaphore channel without blocking.
 *
 * @param sem_num The semaphore channel to attempt to lock.
 * @return ARM_DRIVER_OK (0) if lock was acquired,
 *         ARM_DRIVER_ERROR_BUSY (-1) if already locked,
 *         or -1 for invalid channel.
 */
int32_t hwsem_trylock(uint8_t sem_num);

/**
 * @brief Attempt to lock a semaphore channel, blocking until available.
 *
 * This function loops until the semaphore becomes available,
 * using the callback to wait for notification.
 *
 * @param sem_num The semaphore channel to lock.
 * @return 0 on success, -1 on error or invalid channel.
 */
int32_t hwsem_lock(uint8_t sem_num);

/**
 * @brief Unlock a previously locked semaphore channel.
 *
 * @param sem_num The semaphore channel to unlock.
 * @return 0 on success, -1 on error or invalid channel.
 */
int32_t hwsem_unlock(uint8_t sem_num);


/**
 * @brief  Get the current lock-count of a hardware semaphore.
 * @param  sem_num  [0..MAX_HWSEM_CHANNELS-1]
 * @return ≥0 the count (0 = unlocked); ARM_DRIVER_ERROR (-1) if invalid channel.
 */
int32_t hwsem_get_count(uint8_t sem_num);

/**
 * @brief  Simple test: is this semaphore currently held (count>0)?
 * @param  sem_num  [0..MAX_HWSEM_CHANNELS-1]
 * @return true if held, false if unlocked or invalid channel.
 */
bool    hwsem_is_locked(uint8_t sem_num);


int32_t hwsem_cam_custom_lock(uint8_t sem_num);


#ifdef __cplusplus
}
#endif

#endif // HWSEM_WRAPPER_H
