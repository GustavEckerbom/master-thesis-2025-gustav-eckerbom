#pragma GCC push_options
#pragma GCC optimize ("O3")

#include "image_buffer.h"
#include "core_cm55.h"
#include "Core_select.h"
#include <stdio.h>
#include "tinyusb_wrapper.h"

// Frame buffer memory blocks — must be placed in special memory section and aligned


uint8_t arm_app_rawImage_1[CAMERA_BUFFER_SIZE] __attribute__((section(".bss.camera_frame_buf"), aligned(16))) = {0};
uint8_t arm_app_rawImage_2[CAMERA_BUFFER_SIZE] __attribute__((section(".bss.camera_frame_buf"), aligned(16))) = {0};
uint8_t arm_app_rawImage_3[CAMERA_BUFFER_SIZE] __attribute__((section(".bss.camera_frame_buf"), aligned(16))) = {0};
uint8_t arm_app_rawImage_4[CAMERA_BUFFER_SIZE] __attribute__((section(".bss.camera_frame_buf"), aligned(16))) = {0};

uint32_t dead_time_between_frames_HE_core __attribute__((section(".bss.camera_frame_buf"), aligned(16))) = 0; // 875 us

// Buffer descriptors
volatile frame_buffer_t frame_buffers[NUM_BUFFERS]; // __attribute__((section(".bss.camera_desc_buf"), aligned(32)));

// Camera streaming to buffer
volatile bool usb_ready = 0; // Flag to indicate if the camera is streaming to the buffer

// Currently active CPI write index
volatile uint8_t frame_index = 0;

// Previous buffer index that was written to
volatile int8_t last_frame_index = 0; 

volatile bool buffer_overrun = false; // Flag to indicate if a buffer overrun has occurred

volatile bool new_frame_available = false; // Flag to indicate if a new frame is available

// Flag to indicate if the first frame has been received
volatile bool first_frame_pass = false;

void init_frame_buffers(void)
{
    for (uint8_t i = 0; i < NUM_BUFFERS; i++) {
        uint8_t *buf = (i==0 ? arm_app_rawImage_1
                       : i==1 ? arm_app_rawImage_2
                       : i==2 ? arm_app_rawImage_3
                              : arm_app_rawImage_4);
        frame_buffers[i].data   = buf;
        frame_buffers[i].status = BUFFER_FREE;      // ← initialize status
        clean_frame_desc(i);                        // ← push the whole 32-byte struct into SRAM
    }

    frame_index = 0;
    __DSB();  // ensure all stores (including cache clean) complete
}

void clean_frame_cache(uint8_t idx)
{
    uint8_t *data_ptr = frame_buffers[idx].data;
    uintptr_t addr = (uintptr_t)data_ptr & ~(CACHE_LINE_SIZE - 1);
    size_t len = CAMERA_IMAGE_RAW_SIZE;
    len = (len + CACHE_LINE_SIZE - 1) & ~(CACHE_LINE_SIZE - 1);

    SCB_CleanDCache_by_Addr((uint32_t *)addr, len);
    __DSB();
}

void invalidate_frame_cache(uint8_t idx)
{
    uint8_t *data_ptr = frame_buffers[idx].data;
    uintptr_t addr = (uintptr_t)data_ptr & ~(CACHE_LINE_SIZE - 1);
    size_t len = CAMERA_IMAGE_RAW_SIZE;
    len = (len + CACHE_LINE_SIZE - 1) & ~(CACHE_LINE_SIZE - 1);

    SCB_InvalidateDCache_by_Addr((uint32_t *)addr, len);
    __DMB();
}



/* --- DEBUGGING --- */
// Functions for the framebuffer and status flags memory regions
// In your image_buffer.c (definition)

const char* buffer_status_to_string(buffer_status_t s) {
    switch (s) {
        case BUFFER_FREE:       return "BUFFER_FREE";
        case BUFFER_WRITING:    return "BUFFER_WRITING";
        case BUFFER_READY:      return "BUFFER_READY";
        case BUFFER_PROCESSING: return "BUFFER_PROCESSING";
        case BUFFER_QUEUED:     return "BUFFER_QUEUED";
        default:                return "UNKNOWN";
    }
}


void dump_frame_buffers(void)
{
    printf("---- FRAME BUFFER DESCRIPTORS DUMP ----\r\n");
    for (int i = 0; i < NUM_BUFFERS; i++) {
        #ifdef HP_CORE
        /*// 1) Where is the struct in SRAM?
        usb_blocking_printf(" desc[%d] @ %p: ", i, (void*)&frame_buffers[i]);
        tud_task();
        // 2) Invalidate and read the real status
        invalidate_frame_desc(i);
        buffer_status_t st = frame_buffers[i].status;

        // 3) Print data pointer + status
        usb_blocking_printf(".data=%p, .status=%s\r\n",
               (void*)frame_buffers[i].data,
               buffer_status_to_string(st));

        tud_task();
        */
        #elif defined(HE_CORE)
        // 1) Where is the struct in SRAM?
        printf(" desc[%d] @ %p: ", i, (void*)&frame_buffers[i]);

        // 2) Invalidate and read the real status
        invalidate_frame_desc(i);
        buffer_status_t st = frame_buffers[i].status;

        // 3) Print data pointer + status
        printf(".data=%p, .status=%s\r\n",
               (void*)frame_buffers[i].data,
               buffer_status_to_string(st));
        #endif
    }
    printf("---- END DUMP ----\r\n");
    
}

// Helper to dump one descriptor’s raw words + interpreted fields
void dump_single_desc(uint8_t i) {
    uint32_t *words = (uint32_t*)&frame_buffers[i];
    const char *st = buffer_status_to_string((buffer_status_t)words[1]);
#ifdef HE_CORE
    printf("DUMP[%u] @%p raw0=0x%08" PRIx32 " raw1=0x%08" PRIx32 
           " → data=0x%08" PRIx32 " status=%s\n\r",
           i, (void*)&frame_buffers[i],
           words[0], words[1], words[0], st);
#elif defined(HP_CORE)
    /*
    usb_blocking_printf("DUMP[%u] @%p raw0=0x%08" PRIx32 " raw1=0x%08" PRIx32
                        " → data=0x%08" PRIx32 " status=%s\r\n",
           i, (void*)&frame_buffers[i],
           words[0], words[1], words[0], st);
    tud_task();
    */
#endif
}
