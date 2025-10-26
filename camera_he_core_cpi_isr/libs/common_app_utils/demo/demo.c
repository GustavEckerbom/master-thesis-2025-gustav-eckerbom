#pragma GCC push_options
#pragma GCC optimize("O3")

#include "demo.h"
#include "event_preprocessing.h"
#include "Driver_Common.h"
#include "tinyusb_wrapper.h"
#include "image_buffer.h"
#include "hwsem_wrapper.h"
// --- Mode state ---
static DemoMode current_mode = MODE_NONE;

// --- Function pointers for selected behavior ---
static void (*send_frame_func)(const uint8_t marker[4]) = NULL;
static uint32_t (*iterate_func)(const uint32_t*) = NULL;

// Safe markers: first nibble != any defined EVT2.0 format
static const uint8_t marker_event_visualize[4] = {0x23, 0xAA, 0xBB, 0xCC}; 
static const uint8_t marker_eros_surface[4]    = {0x35, 0xDE, 0xAD, 0x55}; 
static const uint8_t marker_event_count[4]     = {0x97, 0xC0, 0xFE, 0xEE};
static const uint8_t marker_timestamps[4]      = {0xDE, 0xAD, 0xBE, 0xEF};
static const uint8_t marker_spg[4]             = {0x42, 0x50, 0x47, 0x21}; 



void set_demo_mode(DemoMode mode) {
    current_mode = mode;

    switch (mode) {
        case MODE_VISUALIZE_EVENTS:
            iterate_func = iterate_event_visualization;
            send_frame_func = send_surface_over_vcp_clear;
            break;

        case MODE_VISUALIZE_EROS_K3:
            iterate_func = iterate_eros_k3;
            send_frame_func = send_surface_over_vcp;
            break;

        case MODE_VISUALIZE_EROS_K5:
            iterate_func = iterate_eros_k5;
            send_frame_func = send_surface_over_vcp;
            break;

        case MODE_COUNT_EVENTS:
            iterate_func = iterate_event_count;
            send_frame_func = send_count_data;
            break;

        case MODE_TIMESTAMPS:
            iterate_func = iterate_timestamps;
            send_frame_func = send_timestamps_over_vcp;
            break;

        case MODE_SPG:
            iterate_func = iterate_spg;
            send_frame_func = send_spg;
            break;
            
        default:
            iterate_func = NULL;
            send_frame_func = NULL;
            break;
    }
}

DemoMode get_demo_mode(void) {
    return current_mode;
}

/**
 * @brief Dispatch to the correct event iterator.
 */
uint32_t demo_iterate_over_events(const uint32_t* frame_data) {
    return (iterate_func != NULL) ? iterate_func(frame_data) : 0;
}

/**
 * @brief Process available CPI frame buffers and handle them according to the selected demo mode.
 *
 * This function loops over NUM_BUFFERS and processes any buffer that is locked.
 * At the end of processing, the corresponding data is sent over USB with the appropriate marker.
 *
 * @return 0 on success, -1 on error
 */
uint32_t process_ready_frames(void) {
    for (uint8_t i = 0; i < NUM_BUFFERS; i++) {

        if (hwsem_cam_custom_lock(i) == ARM_DRIVER_OK) {

            // Invalidate frame Cache
            invalidate_frame_cache(i);

            demo_iterate_over_events((const uint32_t*)frame_buffers[i].data);

            if (hwsem_unlock(i) != ARM_DRIVER_OK) {
                return -1;
            }
        }

        if ((i % 2) == 1){
            if (current_mode == MODE_VISUALIZE_EVENTS) send_frame_func(marker_event_visualize);
        }
    }

    if (send_frame_func != NULL) {
        switch (current_mode) {
            case MODE_VISUALIZE_EROS_K3:
                send_frame_func(marker_eros_surface);
            case MODE_VISUALIZE_EROS_K5:
                send_frame_func(marker_eros_surface);
                break;
            case MODE_COUNT_EVENTS:
                send_frame_func(marker_event_count);
                break;
            case MODE_TIMESTAMPS:
                send_frame_func(marker_timestamps);
                break;
            case MODE_SPG:
                send_frame_func(marker_spg); // Arbitrary maker not used
                break;
            default:
                break;
        }
    }
    return 0;
}
