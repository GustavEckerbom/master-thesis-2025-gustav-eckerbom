#ifndef EVENT_PREPROCESSING_H
#define EVENT_PREPROCESSING_H

#include <stdint.h>
#include <stddef.h>
#include "image_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMAGE_WIDTH     320
#define IMAGE_HEIGHT    320
#define KERNEL_SIZE     5
#define HALF_KERNEL     (KERNEL_SIZE / 2)
#define NUM_DECAY_PROFILES 100
#define TIMESTAMP_BUFFER_SIZE 1024*64*4/256 // 64KB frame, 4 frames before transmissition, store every 256nd event


extern uint8_t decay_lookup_tables[NUM_DECAY_PROFILES][256];
extern uint8_t decay_lookup_table[256];
extern volatile int current_decay_index;
extern uint8_t surface_buffer[IMAGE_HEIGHT][IMAGE_WIDTH];
extern uint32_t event_count[NUM_BUFFERS];
extern uint32_t deltatime __attribute__((section(".bss.camera_desc_buf")));

extern uint64_t timestamp_event_buffer[TIMESTAMP_BUFFER_SIZE];
extern uint32_t timestamp_buffer_idx;

extern bool pattern_ok;

void generate_decay_tables(void);
void switch_decay(int8_t delta);
void init_decay_lookup(void);
void init_surface(void);
void clear_surface(void);

uint32_t count_events_in_frame(const uint32_t* frame_data);
uint32_t iterate_over_events(const uint32_t* frame_data);
uint32_t iterate_event_visualization(const uint32_t* frame_data);
uint32_t iterate_eros_k3(const uint32_t* frame_data);
uint32_t iterate_eros_k5(const uint32_t* frame_data);
uint32_t iterate_event_count(const uint32_t* frame_data);

uint32_t iterate_timestamps(const uint32_t* frame_data);

void update_frame_deltatime(void);
uint32_t get_latest_deltatime(void);

__attribute__((always_inline)) static inline void update_surface_eros_k3(uint16_t x, uint16_t y);
__attribute__((always_inline)) static inline void update_surface_eros_k5(uint16_t x, uint16_t y);

uint32_t update_surface_mock(const uint32_t* unused);
uint32_t iterate_surface_mock(const uint32_t* unused);
uint32_t iterate_spg(const uint32_t* frame_data);
void send_test_surface_frames(void);

#ifdef __cplusplus
}
#endif

#endif // EVENT_PREPROCESSING_H
