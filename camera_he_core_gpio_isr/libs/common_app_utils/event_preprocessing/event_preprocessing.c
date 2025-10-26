/* --- Set max optimizations for the compiler in this file --- */
#pragma GCC push_options
#pragma GCC optimize ("O3")

/* --- Includes --- */
#include "event_preprocessing.h"
#include "image_buffer.h"
#include "utimer_wrapper.h"
#include "tinyusb_wrapper.h"
#include <stdint.h>
#include <math.h>
#include "utimer_wrapper.h"
#include "Core_select.h"
#include "system_utils.h"


uint64_t timestamp_event_buffer[TIMESTAMP_BUFFER_SIZE];

uint32_t timestamp_buffer_idx = 0;

bool pattern_ok = true;

/**
 * @brief Decay factor used in fallback table generation (not used in multi-profile).
 */
#define DECAY_FACTOR_FLOAT 0.92f

/**
 * @brief Number of decay profiles used for tuning temporal decay strength.
 */
#define NUM_DECAY_PROFILES 100

/**
 * @brief Lookup tables for multiple decay profiles. Each profile holds 256 decayed values.
 * 
 * decay_lookup_tables[p][v] = decayed value of intensity v for profile p.
 */
uint8_t decay_lookup_tables[NUM_DECAY_PROFILES][256];

/**
 * @brief Currently active decay lookup table, points to one of the profiles.
 */
uint8_t decay_lookup_table[256];

/**
 * @brief Index of the currently active decay profile.
 * 
 * Can be changed at runtime to tune the decay behavior interactively.
 */
volatile int current_decay_index = 20;

/**
 * @brief Array that holds total number of ON and OFF events processed in each frame
 */
uint32_t event_count[NUM_BUFFERS] = {0};

/**
 * @brief Number of 32-bit events in a single LPCPI 64KB frame.
 */
const size_t num_events = (1024 * 64) / sizeof(uint32_t);

/**
 * @brief Margin to avoid accessing pixels outside the image boundary.
 * 
 * Computed from the kernel size (e.g., 1 for 3x3, 2 for 5x5).
 */
const uint16_t margin = KERNEL_SIZE / 2;

/**
 * @brief Surface buffer representing the decayed event-based image.
 * 
 * Each pixel contains an 8-bit value updated based on incoming events.
 * This buffer is decayed using the lookup tables and visualized externally.
 */
uint8_t surface_buffer[IMAGE_HEIGHT][IMAGE_WIDTH] = {0};


/**
 * @brief Delta time in milliseconds between processed frames.
 *
 * Updated by the HE core every NUM_BUFFERS frames, after acquiring the last buffer.
 * Safe for the HP core to read once all buffers are processed. Stored in SRAM1 to
 * avoid occupying performance-critical memory.
 */
uint32_t deltatime __attribute__((section(".bss.camera_desc_buf"))) = 0;

/* ---------------------------------------------------------------------------- */
/*                        MACROS & INLINE FUNCTIONS                             */
/* ---------------------------------------------------------------------------- */

/** @section Macros and Inline Functions
 *  Helper macros and performance-critical pixel update functions.
*/

/**
 * @brief Apply decay to a pixel in the surface buffer at an offset from (x, y).
 *
 * This macro safely computes the address of a neighboring pixel at offset (xoff, yoff)
 * relative to the current event location (x, y), and applies the decay function
 * using the currently selected decay lookup table.
 *
 * The macro uses a `do { ... } while (0)` wrapper to ensure that it behaves
 * as a single statement in any context (e.g., inside `if` blocks without braces).
 *
 * @param xoff Horizontal offset from the event's x-coordinate.
 * @param yoff Vertical offset from the event's y-coordinate.
 */
#define D(xoff, yoff)                                           \
    do {                                                        \
        uint8_t *px = &surface_buffer[y + (yoff)][x + (xoff)];  \
        *px = decay_lookup_table[*px];                          \
    } while (0)



/**
 * @brief Generate precomputed decay lookup tables for different decay profiles.
 *
 * This function fills the `decay_lookup_tables` array with `NUM_DECAY_PROFILES` decay
 * profiles, ranging from a decay factor of max_decay down to min_decay. Each profile contains
 * a 256-entry table that maps 8-bit pixel values (0–255) to their decayed equivalents
 * using the formula:
 *
 *     decayed = round(original_value * scale)
 *
 * where `scale` is the decay factor for the given profile.
 *
 * The currently active decay table (`decay_lookup_table`) is also initialized
 * to the table corresponding to `current_decay_index`.
 *
 * This function should be called once during system initialization.
 */
void generate_decay_tables(void) {
    const float max_decay = 0.99f;  // Start just below 1.0
    const float min_decay = 0.80f;
    const float step = (max_decay - min_decay) / (NUM_DECAY_PROFILES - 1);

    for (int profile = 0; profile < NUM_DECAY_PROFILES; ++profile) {
        float scale = max_decay - profile * step;
        for (int i = 0; i < 256; ++i) {
            float val = i * scale + 0.5f;
            if (val > 255.0f) val = 255.0f;
            if (val < 0.0f) val = 0.0f;
            decay_lookup_tables[profile][i] = (uint8_t)val;
        }
    }

    // Initialize current active table
    memcpy(decay_lookup_table, decay_lookup_tables[current_decay_index], 256);
}


/**
 * @brief Change the currently active decay profile by a relative delta.
 *
 * Updates the `current_decay_index` by adding the provided `delta` (which can be
 * positive or negative). If the new index is within valid bounds
 * `[0, NUM_DECAY_PROFILES - 1]`, the function updates the active `decay_lookup_table`
 * to match the new profile from `decay_lookup_tables`.
 *
 *
 * @param delta The change in profile index (e.g., +1 to increase decay, -1 to decrease).
 */
void switch_decay(int8_t delta) {
    int new_index = current_decay_index + delta;
    if (new_index >= 0 && new_index < NUM_DECAY_PROFILES) {
        current_decay_index = new_index;
        memcpy(decay_lookup_table, decay_lookup_tables[current_decay_index], 256);
    }
}


/**
 * @brief Initialize the decay lookup table using a fixed user-defined decay factor.
 *
 * Populates `decay_lookup_table` using the global `DECAY_FACTOR_FLOAT`, which determines
 * how strongly previous surface values are decayed over time. This method is intended
 * for scenarios where the decay factor is manually specified by the user.
 *
 * The resulting table maps each input value in [0, 255] to its decayed counterpart,
 * rounded to the nearest integer.
 *
 * This function is an alternative to `generate_decay_tables()` and is typically used
 * when only a single decay behavior is needed, or for quick prototyping with a known
 * fixed decay factor.
 */
void init_decay_lookup(void) {
    for (int i = 0; i < 256; ++i) {
        float decayed = i * DECAY_FACTOR_FLOAT;
        if (decayed > 255.0f) decayed = 255.0f;
        decay_lookup_table[i] = (uint8_t)(decayed + 0.5f);  // round to nearest
    }
}

/**
 * @brief Initializes decay tables and clears the surface buffer.
 *
 * Sets up decay lookup tables and zeros the entire surface buffer.
 * Call once at startup. For buffer-only reset, use clear_surface().
*/
void init_surface(void) {
    generate_decay_tables();
    for (uint16_t y = 0; y < IMAGE_HEIGHT; ++y) {
        for (uint16_t x = 0; x < IMAGE_WIDTH; ++x) {
            surface_buffer[y][x] = 0;
        }
    }
}

/**
 * @brief Clear the surface buffer by setting all pixels to zero.
 *
 * This function resets all values in the surface buffer to zero.
 * Use it to clear the image representation before starting a new run
 * or after a reset.
 */
void clear_surface(void) {
    memset(surface_buffer, 0, sizeof(surface_buffer));
}

/**
 * @brief Count ON and OFF events in a given LPCPI EVT2.0 frame.
 * @param frame_data Pointer to 64KB LPCPI frame (as 32-bit words)
 * @return Number of CD_HIGH and CD_LOW events
 */
uint32_t count_events_in_frame(const uint32_t* frame_data) {
    const size_t num_events = (1024 * 64) / sizeof(uint32_t);
    uint32_t count = 0;

    for (size_t i = 0; i < num_events; ++i) {
        uint8_t event_type = (frame_data[i] >> 28) & 0xF;
        if (event_type == 0x0 || event_type == 0x1) {
            count++;
        }
    }

    return count;
}

/**
 * @brief Update the surface buffer using a 3x3 decay kernel.
 *
 * Applies the decay function to all neighboring pixels in a 3x3 region
 * centered at (x, y), excluding the center pixel. Then sets the center
 * pixel at (x, y) to the maximum value (255).
 *
 * @param x X-coordinate of the event.
 * @param y Y-coordinate of the event.
 */
__attribute__((always_inline)) static inline void update_surface_eros_k3(uint16_t x, uint16_t y) {
    D(-1, -1); D( 0, -1); D( 1, -1);
    D(-1,  0);            D( 1,  0);
    D(-1,  1); D( 0,  1); D( 1,  1);

    surface_buffer[y][x] = 255U;
}


/**
 * @brief Update the surface buffer using a 5x5 decay kernel.
 *
 * Applies the decay function to all neighboring pixels in a 5x5 region
 * centered at (x, y), excluding the center pixel. Then sets the center
 * pixel at (x, y) to the maximum value (255).
 *
 * @param x X-coordinate of the event.
 * @param y Y-coordinate of the event.
 */
__attribute__((always_inline)) static inline void update_surface_eros_k5(uint16_t x, uint16_t y) {
    D(-2, -2); D(-1, -2); D( 0, -2); D( 1, -2); D( 2, -2);
    D(-2, -1); D(-1, -1); D( 0, -1); D( 1, -1); D( 2, -1);
    D(-2,  0); D(-1,  0);            D( 1,  0); D( 2,  0);
    D(-2,  1); D(-1,  1); D( 0,  1); D( 1,  1); D( 2,  1);
    D(-2,  2); D(-1,  2); D( 0,  2); D( 1,  2); D( 2,  2);

    surface_buffer[y][x] = 255U;
}



/**
 * @brief Iterates over the events in a given LPCPI EVT2.0 frame.
 * @param frame_data Pointer to 64KB LPCPI frame
 * @return 0 on success
*/
uint32_t iterate_over_events(const uint32_t* frame_data) {
    size_t i;
    uint32_t evt;
    uint8_t event_type;
    uint16_t x, y;
    uint16_t width_border = IMAGE_WIDTH - margin;
    uint16_t height_border = IMAGE_HEIGHT - margin;
    

    for (i = 0; i < num_events; ++i) {
        evt = frame_data[i];
        event_type = (evt >> 28) & 0xF;

        if (event_type == 0x0 || event_type == 0x1) {  // CD_LOW or CD_HIGH
            x = (evt >> 11) & 0x7FF;
            y = evt & 0x7FF;

            if (x >= margin && x < width_border &&
                y >= margin && y < height_border) {
                update_surface_eros_k5(x, y);
            }
        }
    }
    return 0;
}

/**
 * @brief Extracts and reconstructs timestamps from EVT2.0 events.
 *
 * This function iterates through a 64KB LPCPI EVT2.0 frame, decoding:
 *   - EVT_TIME_HIGH (type 0x8) to update the MSB timestamp part (bits [33:6]).
 *   - CD_HIGH (0x1) and CD_LOW (0x0) to reconstruct the full 34-bit timestamp
 *     by combining MSB and LSB as: (msb << 6) | lsb.
 *
 * To reduce bandwidth, only every X-th polarity event is stored.
 * The stored value is the lower 32 bits of the full 34-bit timestamp,
 * meaning the 2 MSBs [33:32] are discarded.
 *
 * @param frame_data Pointer to 64KB LPCPI frame (32-bit aligned).
 * @return 1.
*/
/*
uint32_t iterate_timestamps(const uint32_t* frame_data) {
    static uint32_t timestamp_msb = 0;
    static uint32_t event_counter = 0;
    const uint32_t store_interval = 1024;

    for (size_t i = 0; i < num_events; ++i) {
        uint32_t evt = frame_data[i];
        uint8_t type = (evt >> 28) & 0xF;

        if (type == 0x8) {
            timestamp_msb = evt & 0x0FFFFFFF;
        } 
        else if (type == 0x0 || type == 0x1) {
            if ((event_counter++ % store_interval) == 0) {
                uint32_t timestamp_lsb = (evt >> 22) & 0x3F;
                uint32_t truncated_timestamp = ((timestamp_msb & 0x03FFFFFF) << 6) | timestamp_lsb;
                timestamp_event_buffer[timestamp_buffer_idx] = truncated_timestamp;
                timestamp_buffer_idx++;            
            }            
        }
    }

    return 1;
}
    */

uint32_t iterate_timestamps(const uint32_t* frame_data) {
    static uint32_t timestamp_msb = 0;
    static uint32_t event_counter = 0;
    const uint32_t store_interval = 1024;

    for (size_t i = 0; i < num_events; ++i) {
        uint32_t evt = frame_data[i];
        uint8_t type = (evt >> 28) & 0xF;

        if (type == 0x8) {
            timestamp_msb = evt & 0x0FFFFFFF;
        } 
        else if (type == 0x0 || type == 0x1) {
            if ((event_counter++ % store_interval) == 0) {
                uint32_t timestamp_lsb = (evt >> 22) & 0x3F;

                // Construct full 34-bit timestamp
                uint64_t full_timestamp = ((uint64_t)timestamp_msb << 6) | timestamp_lsb;

                // Store to buffer
                timestamp_event_buffer[timestamp_buffer_idx] = full_timestamp;
                timestamp_buffer_idx++;
            }
        }
    }

    return 1;
}

    

/**
 * @brief Iterate over events and apply EROS update with 3x3 kernel.
 *
 * This function decodes CD_HIGH/CD_LOW events in the EVT2.0 format
 * and applies a 3x3 EROS surface update for each valid event.
 * First it checks if user wants to cahnge the idex for the decay buffer
 *
 * @param frame_data Pointer to 64KB LPCPI frame (32-bit aligned words).
 * @return 0 on success, -1 on failure (e.g. unlock error).
 */
uint32_t iterate_eros_k3(const uint32_t* frame_data) {
    size_t i;
    uint32_t evt;
    uint8_t event_type;
    uint16_t x, y;
    uint16_t width_border = IMAGE_WIDTH - 1;
    uint16_t height_border = IMAGE_HEIGHT - 1;

    for (i = 0; i < num_events; ++i) {
        evt = frame_data[i];
        event_type = (evt >> 28) & 0xF;

        if (event_type == 0x0 || event_type == 0x1) {  // CD_LOW or CD_HIGH
            x = (evt >> 11) & 0x7FF;
            y = evt & 0x7FF;

            if (x >= 1 && x < width_border &&
                y >= 1 && y < height_border) {
                update_surface_eros_k3(x, y);
            }
        }
    }
    return 0;
}


/**
 * @brief Iterate over events and apply EROS update with 5x5 kernel.
 *
 * This function decodes CD_HIGH/CD_LOW events in the EVT2.0 format
 * and applies a 5x5 EROS surface update for each valid event.
 *
 * @param frame_data Pointer to 64KB LPCPI frame (32-bit aligned words).
 * @return 0 on success, -1 on failure (e.g. unlock error).
 */
uint32_t iterate_eros_k5(const uint32_t* frame_data) {
    size_t i;
    uint32_t evt;
    uint8_t event_type;
    uint16_t x, y;
    uint16_t width_border = IMAGE_WIDTH - 2;
    uint16_t height_border = IMAGE_HEIGHT - 2;

    for (i = 0; i < num_events; ++i) {
        evt = frame_data[i];
        event_type = (evt >> 28) & 0xF;

        if (event_type == 0x0 || event_type == 0x1) {  // CD_LOW or CD_HIGH
            x = (evt >> 11) & 0x7FF;
            y = evt & 0x7FF;

            if (x >= 2 && x < width_border &&
                y >= 2 && y < height_border) {
                update_surface_eros_k5(x, y);
            }
        }
    }
    return 0;
}

/**
 * @brief Visualize ON and OFF events directly in the surface buffer.
 *
 * This function scans the LPCPI EVT2.0 frame and sets the pixel at each event’s
 * coordinates to:
 *   - 1 for CD_LOW (OFF event)
 *   - 2 for CD_HIGH (ON event)
 *
 *
 * @param frame_data Pointer to 64KB LPCPI frame (32-bit aligned)
 * @return 0 on success
 */
uint32_t iterate_event_visualization(const uint32_t* frame_data) {
    uint32_t evt;
    uint8_t event_type;
    uint16_t x, y;

    for (size_t i = 0; i < num_events; ++i) {
        evt = frame_data[i];
        event_type = (evt >> 28) & 0xF;

        if (event_type == 0x0 || event_type == 0x1) {
            x = (evt >> 11) & 0x7FF;
            y = evt & 0x7FF;

            if (x < IMAGE_WIDTH && y < IMAGE_HEIGHT) {
                surface_buffer[y][x] = (event_type == 0x0) ? 1 : 2;
            }
        }
    }

    return 0;
}

/**
 * @brief Iterate over events and count CD_HIGH/CD_LOW events per buffer.
 *
 * This function parses a EVT2.0 frame and counts the number of
 * polarity events (CD_HIGH and CD_LOW). The result is stored in the
 * `event_count[]` array at a cyclic index corresponding to the most recently
 * processed buffer.
 *
 * @param frame_data Pointer to LPCPI EVT2.0 frame (64KB, 32-bit aligned words).
 * @return 0 on success
 */
uint32_t iterate_event_count(const uint32_t* frame_data) {
    static uint8_t buffer_index = 0;
    uint32_t count = 0;
    uint32_t evt;
    uint8_t event_type;

    for (size_t i = 0; i < num_events; ++i) {
        evt = frame_data[i];
        event_type = (evt >> 28) & 0xF;

        if (event_type == 0x0 || event_type == 0x1) {  // CD_LOW or CD_HIGH
            count++;
        }
    }

    event_count[buffer_index] = count;

    buffer_index = (buffer_index + 1) % NUM_BUFFERS;

    return 0;
}

/**
 * @brief Measure elapsed time since last frame, store it in `deltatime`, and clean it from cache.
 * 
 * In the demo app this is called by the HE core every time the NUM_BUFFER semaphore is locked
 * This function:
 * 1. Calculates the time delta (in ms) since the last reference using UTIMER.
 * 2. Stores the result in the `deltatime` variable located in SRAM1.
 * 3. Updates the UTIMER reference tick.
 * 4. Cleans the `deltatime` variable from the data cache to push it to memory,
 *    ensuring the HP core can read it without stale cache content.
 */
void update_frame_deltatime(void) {
    // Compute delta in ms
    uint32_t delta = utimer_get_delta_time_ms(UTIMER_CHANNEL);
    
    // Reset reference time for next measurement
    utimer_set_reference(UTIMER_CHANNEL);

    // Write to shared memory region
    deltatime = delta;

    // Clean cache for the variable to flush to SRAM1
    SCB_CleanDCache_by_Addr((uint32_t *)&deltatime, sizeof(deltatime));
    __DSB();  
}


/**
 * @brief Invalidate cache and safely read delta time written by the HE core.
 * 
 * @return Latest delta time in milliseconds.
 */
uint32_t get_latest_deltatime(void) {
    uintptr_t addr = (uintptr_t)&deltatime & ~(CACHE_LINE_SIZE - 1);
    SCB_InvalidateDCache_by_Addr((uint32_t *)addr, CACHE_LINE_SIZE);
    __DMB();  // Ensure memory read happens after cache invalidation
    return deltatime;
}

/* ---------------------------------------------------------------------------- */
/*                             DEBUGGING UTILITIES                              */
/*      The functions below are used for testing or debugging purposes.         */
/* ---------------------------------------------------------------------------- */


/** @section Debugging Utilities
 *  Functions and helpers for debugging and visualization purposes.
 *
 *  These are not used in the main data path, but are useful for testing
 *  rendering pipelines or mocking sensor input.
 */


/**
 * @brief Fill the surface buffer with a mock pattern for visualization.
 *
 * This function generates an alternating vertical stripe pattern in the
 * surface buffer, toggling between frames. It is useful for debugging or
 * testing display output when real event data is not available.
 *
 */
uint32_t update_surface_mock(const uint32_t* unused){
    static uint8_t toggle = 0;
    toggle ^= 1;  // Flip every frame it's called in

    for (uint16_t y = 0; y < IMAGE_HEIGHT; ++y) {
        for (uint16_t x = 0; x < IMAGE_WIDTH; ++x) {
            // Pattern: alternating vertical bands
            surface_buffer[y][x] = ((x / 40) % 2) ^ toggle ? 255 : 0;
        }
    }

    return 0;
}

uint32_t iterate_surface_mock(const uint32_t* unused) {
    static uint8_t call_count = 0;
    static uint8_t toggle = 0;

    call_count = (call_count + 1) % 4;

    if (call_count == 0) {
        toggle ^= 1;  // Flip every 4 calls
    }

    for (uint16_t y = 0; y < IMAGE_HEIGHT; ++y) {
        for (uint16_t x = 0; x < IMAGE_WIDTH; ++x) {
            surface_buffer[y][x] = ((x / 40) % 2) ^ toggle ? 255 : 0;
        }
    }

    return 0;
}


/**
 * @brief Stream 60 mock surface frames with a shifting gradient over USB.
 *
 * Useful for testing visualization pipelines without real camera input.
 * Simulates 50 FPS by inserting a 20 ms delay between frames.
 */
void send_test_surface_frames(void) {

    for (int frame = 0; frame < 60; frame++) {  // send 60 test frames
        // Create a dynamic test pattern: diagonal gradient that shifts over time
        for (uint16_t y = 0; y < IMAGE_HEIGHT; ++y) {
            for (uint16_t x = 0; x < IMAGE_WIDTH; ++x) {
                surface_buffer[y][x] = (x + y + frame * 10) % 256;
            }
        }
    uint8_t marker[4] = {0x23, 0xAA, 0xBB, 0xCC}; 
    send_surface_over_vcp(marker);  //  Send full frame via USB safely

    // Delay for 20ms between the frames
        for (int i = 0; i<1; i++){
            sys_busy_loop_us(1000 * 20);  
            tud_task();
        }
    }
}

/**
 * @brief Verify that the received frame matches the SPG Mode 2 byte pattern.
 *
 * This function treats the buffer as a flat byte stream and checks that each byte
 * follows the expected sequence: 0x00, 0x01, ..., 0xFF, 0x00, ...
 *
 * @param frame_data Pointer to 32-bit aligned CPI frame buffer.
 * @param num_bytes Number of bytes to check (e.g., 64*1024).
 * @return 0 if pattern is correct, 1 if mismatch found.
 */
uint32_t iterate_spg(const uint32_t* frame_data) {
    const uint8_t* byte_stream = (const uint8_t*)frame_data;
    static uint8_t expected = 0x00;

    for (size_t i = 0; i < num_events; ++i) {
        if (byte_stream[i] != expected) {
            pattern_ok = false;
        }
        expected = expected + 1; // wraps naturally at 256
    }

    return pattern_ok ? 0 : 1;
}
