#ifndef DEMO_H
#define DEMO_H

#include <stdint.h>

/**
 * @brief Supported demo run modes.
 */
typedef enum {
    MODE_NONE = 0,
    MODE_VISUALIZE_EVENTS,  /**< Raw event display mode */
    MODE_COUNT_EVENTS,      /**< Count CD_HIGH/CD_LOW events */
    MODE_VISUALIZE_EROS_K3, /**< EROS surface visualization with 3x3 kernel */
    MODE_VISUALIZE_EROS_K5, /**< EROS surface visualization with 5x5 kernel */
    MODE_TIMESTAMPS,        /**< Timestamp display mode */    
    MODE_SPG                /**< SPG mode, not implemented yet */   
} DemoMode;

/**
 * @brief Set the current demo mode.
 * 
 * This also configures which frame send function to use after processing.
 * 
 * @param mode Desired demo mode to activate.
 */
void set_demo_mode(DemoMode mode);

/**
 * @brief Get the currently active demo mode.
 * 
 * @return Currently selected demo mode.
 */
DemoMode get_demo_mode(void);

/**
 * @brief Process the event data in a frame depending on selected demo mode.
 * 
 * @param frame_data Pointer to LPCPI event buffer (32-bit aligned)
 * @return 0 on success
 */
uint32_t demo_iterate_over_events(const uint32_t* frame_data);

/**
 * @brief Iterate through all available frame buffers and process them.
 * 
 * This function processes all frames in the circular buffer, unlocking each
 * slot after processing. The result (e.g. surface or count) is then sent over USB.
 * 
 * @return 0 on success, -1 on lock/unlock error.
 */
uint32_t process_ready_frames(void);

#endif // DEMO_H
