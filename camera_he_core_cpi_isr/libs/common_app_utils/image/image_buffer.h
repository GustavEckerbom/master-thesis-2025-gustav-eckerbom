#ifndef IMAGE_BUFFER_H
#define IMAGE_BUFFER_H

#include "RTE_Components.h"
#include CMSIS_device_header
#include <stdint.h>
#include <stdbool.h>
#include "Core_select.h"
#include <stdio.h>
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CACHE_LINE_SIZE 32

// Image properties
#define CAMERA_IMAGE_WIDTH     1024
#define CAMERA_IMAGE_HEIGHT    64
#define CAMERA_IMAGE_RAW_SIZE  (CAMERA_IMAGE_WIDTH * CAMERA_IMAGE_HEIGHT)
#define GUARD_BAND_BYTES       (0)
#define CAMERA_BUFFER_SIZE     (CAMERA_IMAGE_RAW_SIZE + GUARD_BAND_BYTES)


// Number of frame buffers in the ring
#define NUM_BUFFERS            4

// Buffer state definitions
typedef enum {
    BUFFER_FREE         = 0,    // Ready to be written by CPI
    BUFFER_WRITING      = 1,    // CPI is currently writing to this buffer
    BUFFER_READY        = 2,    // Frame is ready for processing by main loop
    BUFFER_PROCESSING   = 3,    // Core is currently processing this buffer
    BUFFER_QUEUED       = 4     // Buffer is queued to write to by CPI at the next frame arrival
} buffer_status_t;

// Frame buffer descriptor
#define CACHE_LINE 32

typedef struct __attribute__((aligned(CACHE_LINE))) {
    uint8_t *                data;     // buffer pointer
    volatile buffer_status_t status;   // one of BUFFER_FREE, etc.
    uint8_t                  _pad[CACHE_LINE - 8];
} frame_buffer_t;


extern uint32_t dead_time_between_frames_HE_core __attribute__((section(".bss.camera_frame_buf"), aligned(16)));

// Raw memory for frame storage (placed in SRAM, must be defined in .c)
extern uint8_t arm_app_rawImage_1[CAMERA_BUFFER_SIZE] __attribute__((section(".bss.camera_frame_buf"), aligned(16)));
extern uint8_t arm_app_rawImage_2[CAMERA_BUFFER_SIZE] __attribute__((section(".bss.camera_frame_buf"), aligned(16)));
extern uint8_t arm_app_rawImage_3[CAMERA_BUFFER_SIZE] __attribute__((section(".bss.camera_frame_buf"), aligned(16)));
extern uint8_t arm_app_rawImage_4[CAMERA_BUFFER_SIZE] __attribute__((section(".bss.camera_frame_buf"), aligned(16)));

// Ring buffer array
extern volatile frame_buffer_t frame_buffers[NUM_BUFFERS];

// Camera streaming to buffer
extern volatile bool usb_ready;

// Flag to indicate if a buffer overrun has occurred 
extern volatile bool buffer_overrun; 

// Flag to indicate if a new frame is available
extern volatile bool new_frame_available;

// Index of the buffer currently being written to (-1 = none)
extern volatile uint8_t frame_index;

// Previous buffer index that was written to
extern volatile int8_t last_frame_index; 

extern volatile bool first_frame_pass; // Flag to indicate if the first frame has been received

// Optional helper function to initialize buffer metadata
void init_frame_buffers(void);

/**
 * @brief  Clean (push) the D-cache for a frame’s data out to SRAM.
 * @param  idx  Frame buffer index [0..NUM_BUFFERS-1].
 */
void clean_frame_cache(uint8_t idx);

/**
 * @brief  Invalidate (discard) the D-cache for a frame’s data so next reads come from SRAM.
 * @param  idx  Frame buffer index [0..NUM_BUFFERS-1].
 */
void invalidate_frame_cache(uint8_t idx);

static inline void clean_frame_desc(uint8_t idx) {
    SCB_CleanDCache_by_Addr(
        (uint32_t*)((uintptr_t)&frame_buffers[idx] & ~(CACHE_LINE_SIZE - 1)),
        ((sizeof(frame_buffer_t) + CACHE_LINE_SIZE - 1) & ~(CACHE_LINE_SIZE - 1))
    );
    __DSB();  // ensure it really completed
    #ifdef HE_CORE
    // uint32_t *p = (uint32_t*)&frame_buffers[idx];
    // printf("CLEAN[%u] %08" PRIx32 " %08" PRIx32 "\n\r", idx, p[0], p[1]);
    #endif
}

static inline void invalidate_frame_desc(uint8_t idx) {
    SCB_InvalidateDCache_by_Addr(
        (uint32_t*)((uintptr_t)&frame_buffers[idx] & ~(CACHE_LINE_SIZE - 1)),
        ((sizeof(frame_buffer_t) + CACHE_LINE_SIZE - 1) & ~(CACHE_LINE_SIZE - 1))
    );
    __DMB();  // ensure subsequent reads see fresh data
}


/* --- DEBUGGING PRINTS ---*/
const char* buffer_status_to_string(buffer_status_t s);
void dump_frame_buffers(void);
void dump_single_desc(uint8_t i);

#ifdef __cplusplus
}
#endif

#endif // IMAGE_BUFFER_H
