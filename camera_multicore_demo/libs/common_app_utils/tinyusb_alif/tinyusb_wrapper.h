#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "tusb.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handles commands from the host in the vendor class.
 */
void handle_vendor_input();

/**
 * @brief Streams frame over VCP to host. Called when frame is ready.
 */
void send_raw_image_over_vcp(uint8_t frame);

/**
 * @brief Handle incoming VCP input stream. Call regularly from main loop.
 */
void handle_input_stream(void);

/**
 * @brief Process a completed line of user input.
 * 
 * @param line Null-terminated string from user.
 */
void process_line(const char *line);

/**
 * @brief Blocking USB CDC print. Sends a null-terminated string over USB.
 * 
 * @param str The string to send (must be null-terminated).
 */
void usb_blocking_print(const char *str);

/**
 * @brief printf-style formatted USB CDC output (blocking).
 * 
 * @param fmt Format string (like printf)
 * @param ... Variable arguments
 */
void usb_blocking_printf(const char *fmt, ...);

/**
 * @brief Custom sync marker used when sending count data.
 *
 * This marker helps the receiver distinguish between surface data and
 * event count payloads over USB. Can be changed at runtime.
 */
extern uint8_t count_sync_marker[4];

extern uint64_t timestamp_event_buffer[];

extern const size_t max_timestamp_events;

void poll_usb_control_commands(void);

void send_surface_over_vcp(const uint8_t marker[4]);

void send_timestamps_over_vcp(const uint8_t marker[4]);

void send_surface_over_vcp_clear(const uint8_t marker[4]);

void send_count_data(const uint8_t marker[4]);

void send_spg(const uint8_t marker[4]);

#include "tusb.h"

void send_hello_over_vendor(void);

#ifdef __cplusplus
}
#endif
