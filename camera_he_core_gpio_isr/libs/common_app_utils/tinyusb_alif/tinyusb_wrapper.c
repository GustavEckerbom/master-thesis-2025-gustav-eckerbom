#include "RTE_Components.h"
#include CMSIS_device_header
#include "image_buffer.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include "tusb.h"
#include "class/cdc/cdc_device.h"
#include <string.h>
#include <stdlib.h>
#include "tinyusb_wrapper.h"
#include "event_preprocessing.h"
#include "Core_select.h"
#include "image_buffer.h"
#include "hwsem_wrapper.h"
#include "board_defs.h"
#include "Driver_GPIO.h"

#ifdef HP_CORE
#include "demo.h"
#endif

#pragma GCC push_options
#pragma GCC optimize ("O3")

#ifdef __cplusplus
extern "C" {
#endif


uint32_t deadtime = 0;

void handle_input_stream(void);
void process_line(const char *line);

#ifdef __cplusplus
}
#endif

#define MAX_VENDOR_LINE_LEN 64
static char vendor_line_buf[MAX_VENDOR_LINE_LEN];
static uint32_t vendor_line_pos = 0;

static cdc_line_coding_t lc __attribute__((used));

#define MAX_LINE_LEN 128
#define USB_CHUNK_SIZE 64

static char line_buf[MAX_LINE_LEN];
static uint32_t line_pos = 0;

extern bool capture_new_frame;
extern bool stream_data_over_usb; // Flag to indicate if a buffer overrun has occurred
extern bool serial_ready;

static uint8_t charbuf[512];

uint32_t deadtime_between_frames = 200; // was 6 before testing

extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_LED_TOP_GPIO_PORT);
ARM_DRIVER_GPIO *ledyHP = &ARM_Driver_GPIO_(BOARD_LED_TOP_GPIO_PORT);

bool tud_vendor_open(uint8_t itf, tusb_desc_interface_t const *desc, uint16_t max_len)
{
    printf(">> tud_vendor_open() called for itf: %d\n", itf);
    return true;
}

typedef enum {
    CMD_UNKNOWN,
    CMD_TESTING1,
    CMD_LED_ON,
    CMD_LED_OFF,
    CMD_NEW_FRAME,
    CMD_PRINT_FRAME,
    CMD_PRINT_FRAME_1,
    CMD_PRINT_FRAME_2,
    CMD_PRINT_FRAME_3,
    CMD_PRINT_FRAME_4,
    CMD_PRINT_BUFFER_OVERRUN,
    CMD_START_STREAM,
    CMD_START,
    CMD_MODE_EROS_K3,
    CMD_MODE_EROS_K5,
    CMD_MODE_EVENT_VIZ,
    CMD_MODE_COUNT_EVENTS,
    CMD_MODE_TIMESTAMPS,
    CMD_MODE_SPG
} command_t;


static command_t parse_command(const char *line)
{
    usb_blocking_printf(">> Received command: '%s'\r\n", line);

    if (strcmp(line, "testing1") == 0) return CMD_TESTING1;
    if (strcmp(line, "led_on") == 0) return CMD_LED_ON;
    if (strcmp(line, "led_off") == 0) return CMD_LED_OFF;
    if (strcmp(line, "newframe") == 0) return CMD_NEW_FRAME;
    if (strcmp(line, "printframe") == 0) return CMD_PRINT_FRAME;
    if (strcmp(line, "printframe1") == 0) return CMD_PRINT_FRAME_1;
    if (strcmp(line, "printframe2") == 0) return CMD_PRINT_FRAME_2;
    if (strcmp(line, "printframe3") == 0) return CMD_PRINT_FRAME_3;
    if (strcmp(line, "printframe4") == 0) return CMD_PRINT_FRAME_4;
    if (strcmp(line, "overrun") == 0) return CMD_PRINT_BUFFER_OVERRUN;
    if (strcmp(line, "start") == 0) return CMD_START;
    if (strcmp(line, "startstream") == 0) return CMD_START_STREAM;
    #ifdef HP_CORE
    if (strcmp(line, "mode_eros_k3") == 0) return CMD_MODE_EROS_K3;
    if (strcmp(line, "mode_eros_k5") == 0) return CMD_MODE_EROS_K5;
    if (strcmp(line, "mode_event_viz") == 0) return CMD_MODE_EVENT_VIZ;
    if (strcmp(line, "mode_count") == 0) return CMD_MODE_COUNT_EVENTS;
    if (strcmp(line, "mode_timestamps") == 0) return CMD_MODE_TIMESTAMPS;
    if (strcmp(line, "mode_spg_echo") == 0) return CMD_MODE_SPG;
    #endif
    return CMD_UNKNOWN;
}

#define VENDOR_BUF_SIZE 512

void handle_vendor_input(void)
{
    uint8_t buf[VENDOR_BUF_SIZE];

    int available = tud_vendor_available();
    if (available > 0) {
        int read_bytes = tud_vendor_read(buf, VENDOR_BUF_SIZE);
        printf("Received %d bytes over VENDOR endpoint\r\n", read_bytes);

        // Debug print first few bytes
        printf("First %d bytes: ", read_bytes < 8 ? read_bytes : 8);
        for (int i = 0; i < read_bytes && i < 8; ++i) {
            printf("%02X ", buf[i]);
        }
        printf("\r\n");
    }
}

void send_hello_over_vendor(void)
{
    const char *msg = "Hello from Vendor Interface!\r\n";

    // Check if the vendor interface is mounted and ready
    if (tud_vendor_mounted() && tud_vendor_write_available() >= strlen(msg)) {
        tud_vendor_write(msg, strlen(msg));
        tud_vendor_write_flush();
    }
    else {
        // Handle the case where the vendor interface is not ready
        printf("From func send_hello_over_vendor: Vendor interface not ready\r\n");
    }
}

/**
 * @brief Send event count data and timing information over USB with a custom sync marker.
 *
 * This function sends a 4-byte sync marker (provided as argument),
 * followed by the `event_count` array and the `deltatime` value.
 *
 * @param marker Pointer to a 4-byte marker used to identify this data packet.
 */
void send_count_data(const uint8_t marker[4]) {
    const size_t payload_size = 4                             // Marker
                              + NUM_BUFFERS * sizeof(uint32_t) // Event counts
                              + sizeof(uint32_t);              // Delta time

    uint8_t buffer[payload_size];
    size_t offset = 0;

    memcpy(&buffer[offset], marker, 4);
    offset += 4;

    memcpy(&buffer[offset], event_count, NUM_BUFFERS * sizeof(uint32_t));
    offset += NUM_BUFFERS * sizeof(uint32_t);

    uint32_t dt = get_latest_deltatime();
    memcpy(&buffer[offset], &dt, sizeof(uint32_t));

    tud_cdc_n_write(0, buffer, payload_size);
    tud_cdc_n_write_flush(0);
}

/**
 * @brief Send the current surface buffer over USB with a custom sync marker and decay index.
 *
 * @param marker Pointer to a 4-byte sync marker used to identify the surface frame.
 */
void send_surface_over_vcp(const uint8_t marker[4]) {
    if (!tud_cdc_connected()) {
        return;
    }

    const uint8_t* data = &surface_buffer[0][0];
    const size_t total_size = IMAGE_WIDTH * IMAGE_HEIGHT;
    const size_t chunk_size = 480;
    uint8_t decay = (uint8_t)current_decay_index;

    // Wait for enough space to send header
    while (tud_cdc_write_available() < 5) {
        tud_task();
    }

    tud_cdc_write(marker, 4);
    tud_cdc_write(&decay, 1);
    tud_cdc_write_flush();
    tud_task();

    size_t offset = 0;
    while (offset < total_size) {
        size_t remaining = total_size - offset;
        size_t to_write = (remaining < chunk_size) ? remaining : chunk_size;

        while (tud_cdc_write_available() < to_write) {
            tud_task();
        }

        size_t written = 0;
        while (written < to_write) {
            size_t this_written = tud_cdc_write(data + offset + written, to_write - written);
            written += this_written;
            tud_task();
        }

        offset += written;
        tud_cdc_write_flush();
        tud_task();
    }
}

void send_surface_over_vcp_clear(const uint8_t marker[4])
{
    send_surface_over_vcp(marker);
    clear_surface();
}


/**
 * @brief Send the SPG pattern check result as a binary packet over USB CDC.
 *
 * @param marker Pointer to a 4-byte sync marker used to identify the SPG packet.
 */
void send_spg(const uint8_t marker[4]) {
    if (!tud_cdc_connected()) {
        return;
    }

    uint8_t result_byte = pattern_ok ? 0x01 : 0x00;

    // Wait for enough space to send marker + 1 byte
    while (tud_cdc_write_available() < 5) {
        tud_task();
    }

    tud_cdc_write(marker, 4);
    tud_cdc_write(&result_byte, 1);
    tud_cdc_write_flush();
    tud_task();
}

/**
 * @brief Send the buffer with the accumulated event timestamps over USB with a custom sync marker.
 *
 * @param marker Pointer to a 4-byte sync marker used to identify the timestamp frame.
 */
/*void send_timestamps_over_vcp(const uint8_t marker[4]) {
    if (!tud_cdc_connected()) {
        return;
    }

    const uint8_t* data = (const uint8_t*)&timestamp_event_buffer[0];
    const size_t total_bytes = timestamp_buffer_idx * sizeof(uint32_t);
    const size_t chunk_size = 480;

    // Wait for enough space to send marker
    while (tud_cdc_write_available() < 5) {
        tud_task();
    }

    tud_cdc_write(marker, 4);
    tud_cdc_write(&timestamp_buffer_idx, 4);
    tud_cdc_write_flush();
    tud_task();

    size_t offset = 0;
    while (offset < total_bytes) {
        size_t remaining = total_bytes - offset;
        size_t to_write = (remaining < chunk_size) ? remaining : chunk_size;

        while (tud_cdc_write_available() < to_write) {
            tud_task();
        }

        size_t written = 0;
        while (written < to_write) {
            size_t this_written = tud_cdc_write(data + offset + written, to_write - written);
            written += this_written;
            tud_task();
        }

        offset += written;
        tud_cdc_write_flush();
        tud_task();
    }

    // Reset after send
    timestamp_buffer_idx = 0;
    
    // clear the buffer
    memset(timestamp_event_buffer, 0, TIMESTAMP_BUFFER_SIZE * sizeof(uint32_t));
}

*/
void send_timestamps_over_vcp(const uint8_t marker[4]) {
    if (!tud_cdc_connected()) {
        return;
    }

    const uint8_t* data = (const uint8_t*)&timestamp_event_buffer[0];
    const size_t total_bytes = timestamp_buffer_idx * sizeof(uint64_t);  // now 8 bytes each
    const size_t chunk_size = 480;

    // Wait for enough space to send marker + index
    while (tud_cdc_write_available() < 8) {
        tud_task();
    }

    tud_cdc_write(marker, 4);
    tud_cdc_write(&timestamp_buffer_idx, 4);  // still 32-bit count
    tud_cdc_write_flush();
    tud_task();

    size_t offset = 0;
    while (offset < total_bytes) {
        size_t remaining = total_bytes - offset;
        size_t to_write = (remaining < chunk_size) ? remaining : chunk_size;

        while (tud_cdc_write_available() < to_write) {
            tud_task();
        }

        size_t written = 0;
        while (written < to_write) {
            size_t this_written = tud_cdc_write(data + offset + written, to_write - written);
            written += this_written;
            tud_task();
        }

        offset += written;
        tud_cdc_write_flush();
        tud_task();
    }

    // Reset after send
    timestamp_buffer_idx = 0;
    memset(timestamp_event_buffer, 0, TIMESTAMP_BUFFER_SIZE * sizeof(uint64_t));
}


void send_raw_image_over_vcp(uint8_t frame)
{
    if (!tud_cdc_connected()) {
        return;
    }

    uint8_t* data = frame_buffers[frame].data;

    const size_t total_size = CAMERA_IMAGE_RAW_SIZE;

    const size_t chunk_size = 480;

    size_t offset = 0;

    while (offset < total_size)
    {
        size_t remaining = total_size - offset;
        size_t to_write = (remaining < chunk_size) ? remaining : chunk_size;
     
        // Wait until there is enough space in the TX buffer
        while (tud_cdc_write_available() < to_write) {
            tud_task();  // Allow USB stack to drain buffers
        }

        // Write and flush this chunk
        tud_cdc_write(&data[offset], to_write);
        tud_cdc_write_flush();
        offset += to_write;

        // Continue servicing USB
        tud_task();
    }

    // Final flush to make sure everything has been sent
    tud_cdc_write_flush();
}

void process_line(const char *line_input)
{
    char line[MAX_LINE_LEN];
    strncpy(line, line_input, MAX_LINE_LEN);
    line[strcspn(line, "\r\n")] = '\0';  // Strip any newline characters
    switch (parse_command(line)) {

        case CMD_START_STREAM:
            tud_cdc_write_str("Start stream requested\r\n");
            capture_new_frame = true;
            stream_data_over_usb = true;
            break;

        case CMD_TESTING1:
            tud_cdc_write_str("test1 received\r\n");
            break;

        case CMD_NEW_FRAME:
            tud_cdc_write_str("New frame requested\r\n");
            capture_new_frame = true;
            break;

        case CMD_PRINT_FRAME:
            tud_cdc_write_str("Print new frame\r\n");
            send_raw_image_over_vcp(0);
            break;

        case CMD_PRINT_FRAME_1:
            tud_cdc_write_str("Print new frame 1\r\n");
            send_raw_image_over_vcp(0);
            break;

        case CMD_PRINT_FRAME_2:
            tud_cdc_write_str("Print new frame 2\r\n");
            send_raw_image_over_vcp(1);
            break;

        case CMD_PRINT_FRAME_3:
            tud_cdc_write_str("Print new frame 3\r\n");
            send_raw_image_over_vcp(2);
            break;

        case CMD_PRINT_FRAME_4:
            tud_cdc_write_str("Print new frame 4\r\n");
            send_raw_image_over_vcp(3);
            break;
        
        case CMD_PRINT_BUFFER_OVERRUN:
            if (buffer_overrun) {
                tud_cdc_write_str("Buffer overrun detected\r\n");
            } else {
                tud_cdc_write_str("No buffer overrun\r\n");
            }
            break;

        case CMD_START:
            tud_cdc_write_str("Start requested\r\n");
            serial_ready = true;
            break;
        
        #ifdef HP_CORE
            case CMD_MODE_EROS_K3:
                set_demo_mode(MODE_VISUALIZE_EROS_K3);
                tud_cdc_write_str("Set mode: EROS K3\n");
                break;
                
            case CMD_MODE_EROS_K5:
                set_demo_mode(MODE_VISUALIZE_EROS_K5);
                tud_cdc_write_str("Set mode: EROS K5\n");
                break;
            
            case CMD_MODE_EVENT_VIZ:
                set_demo_mode(MODE_VISUALIZE_EVENTS);
                tud_cdc_write_str("Set mode: Event Visualization\n");
                break;
            
            case CMD_MODE_COUNT_EVENTS:
                set_demo_mode(MODE_COUNT_EVENTS);
                tud_cdc_write_str("Set mode: Event Count\n");
                break;

            case CMD_MODE_TIMESTAMPS:
                set_demo_mode(MODE_TIMESTAMPS);
                tud_cdc_write_str("Set mode: Timestamp Streaming\n");
                break;

            case CMD_MODE_SPG:
                set_demo_mode(MODE_SPG);
                tud_cdc_write_str("Set mode: SPG\n");
                break;
        #endif
        default:
            tud_cdc_write_str("Unknown command\r\n");
            break;
    }
    tud_cdc_write_flush();
}


void handle_input_stream(void)
{
    while (tud_cdc_available()) {
        char ch;
        tud_cdc_read(&ch, 1);

        if (ch == '\r' || ch == '\n') {
            tud_cdc_write_str("\r\n"); // Echo newline
            line_buf[line_pos] = '\0';
            process_line(line_buf);
            line_pos = 0;
        } else if (ch == 0x08 || ch == 0x7F) { // backspace or DEL
            if (line_pos > 0) {
                line_pos--;
                tud_cdc_write_str("\b \b");
            }
        } else if (line_pos < MAX_LINE_LEN - 1 && ch >= 32 && ch < 127) {
            line_buf[line_pos++] = ch;
            tud_cdc_write(&ch, 1);
        }
    }
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* p_line_coding)
{
    // line coding settings have changed
    lc = *p_line_coding;
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    if (dtr) {
        // tud_cdc_write_str("Hello from TinyUSB!\r\n");
        // tud_cdc_write_flush();
    }
}


void usb_blocking_print(const char *str)
{
    if (!tud_cdc_connected()) return;

    size_t len = strlen(str);
    size_t offset = 0;

    while (offset < len) {
        size_t available = tud_cdc_write_available();
        if (available == 0) {
            tud_task();
            continue;
        }

        size_t remaining = len - offset;
        size_t to_write = (remaining < available) ? remaining : available;

        size_t written = tud_cdc_write(&str[offset], to_write);
        offset += written;

        tud_cdc_write_flush();  // mark the data as ready
        tud_task();             // push it to the host
    }

    // Ensure all remaining data is flushed
    tud_cdc_write_flush();
    tud_task();
}

void usb_blocking_printf(const char *fmt, ...)
{ 
    // Wait for USB to be connected
    if (tud_cdc_connected()) {    
        char buffer[256];  // adjust size as needed
        va_list args;
        va_start(args, fmt);
        vsnprintf(buffer, sizeof(buffer), fmt, args);
        va_end(args);

        usb_blocking_print(buffer);
    }
}

void poll_usb_control_commands(void) {
    while (tud_cdc_available()) {

        char cmd = tud_cdc_read_char();
        
        if (cmd == '+') {
            switch_decay(+1);
            deadtime_between_frames += 1; // Add 5us
        
        } else if (cmd == '-') {
            switch_decay(-1);
            deadtime_between_frames -= 1; // decrease 5us
        }
        
        else if (cmd == 's'){
            // Init the LED GPIOs and header IOs
	        ledyHP->Initialize(BOARD_LED_TOP_GPIO_PIN, NULL);
	        ledyHP->PowerControl(BOARD_LED_TOP_GPIO_PIN, ARM_POWER_FULL);
	        ledyHP->SetDirection(BOARD_LED_TOP_GPIO_PIN, GPIO_PIN_DIRECTION_OUTPUT);
            
            // Set the deadtime between frames and allow HE core to transmit it to the sensor
            dead_time_between_frames_HE_core = deadtime_between_frames;
            ledyHP->SetValue(BOARD_LED_TOP_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_TOGGLE);
            tud_task();
            hwsem_unlock(7);
            while(!(hwsem_is_locked(7))) {
                // Wait for the HE core to lock the deadtime sem
                uint32_t cnt = 999999;
                tud_cdc_write_str("\r\n");
                tud_task();
            }
            hwsem_lock(7); // lock it again
        }

        tud_task();
    }
}
