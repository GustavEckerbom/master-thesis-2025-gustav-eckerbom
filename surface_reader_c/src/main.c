#include <windows.h>
#define SDL_ENABLE_OLD_NAMES
#include <SDL3/SDL.h>
#include <SDL3/SDL_keyboard.h>
#include <SDL3/SDL_keycode.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define COM_PORT "\\\\.\\COM7"
#define BAUD_RATE 115200
#define READ_BUFFER_SIZE 512
#define FRAME_WIDTH 320
#define FRAME_HEIGHT 320
#define WINDOW_SCALE 3
#define WINDOW_WIDTH (FRAME_WIDTH * WINDOW_SCALE)
#define WINDOW_HEIGHT (FRAME_HEIGHT * WINDOW_SCALE)
#define EVENT_COUNT_BUFFERS 4
#define MAX_TIMESTAMPS 32768
#define TIMESTAMP_FIFO_DEPTH 64

typedef enum {
    MODE_NONE,
    MODE_EROS_K3,
    MODE_EROS_K5,
    MODE_EVENT_VIZ,
    MODE_COUNT,
    MODE_TIMESTAMPS,
    MODE_SPG_ECHO
} ViewerMode;

static ViewerMode current_mode = MODE_NONE;

static const uint8_t marker_eros_surface[4]    = {0x35, 0xDE, 0xAD, 0x55};
static const uint8_t marker_event_visualize[4] = {0x23, 0xAA, 0xBB, 0xCC};
static const uint8_t marker_event_count[4]     = {0x97, 0xC0, 0xFE, 0xEE};
static const uint8_t marker_timestamps[4]      = {0xDE, 0xAD, 0xBE, 0xEF};
static const uint8_t marker_spg[4]             = {0x42, 0x50, 0x47, 0x21}; // "BPG!"

typedef struct {
    uint64_t data[MAX_TIMESTAMPS];
    uint32_t count;
} TimestampPacket;

static TimestampPacket timestamp_fifo[TIMESTAMP_FIFO_DEPTH];
static int fifo_head = 0;
static int fifo_count = 0;

int compare_marker(const uint8_t *buf, const uint8_t *marker) {
    return memcmp(buf, marker, 4) == 0;
}

void prompt_user_for_mode(HANDLE hSerial) {
    char input[64];
    printf("Select mode (eros_k3, eros_k5, event_viz, count, timestamps, spg_echo): ");
    scanf("%63s", input);

    const char *cmd = NULL;
    if (strcmp(input, "eros_k3") == 0) {
        cmd = "mode_eros_k3\n";
        current_mode = MODE_EROS_K3;
    } else if (strcmp(input, "eros_k5") == 0) {
        cmd = "mode_eros_k5\n";
        current_mode = MODE_EROS_K5;
    } else if (strcmp(input, "event_viz") == 0) {
        cmd = "mode_event_viz\n";
        current_mode = MODE_EVENT_VIZ;
    } else if (strcmp(input, "count") == 0) {
        cmd = "mode_count\n";
        current_mode = MODE_COUNT;
    } else if (strcmp(input, "timestamps") == 0) {
        cmd = "mode_timestamps\n";
        current_mode = MODE_TIMESTAMPS;
    } else if (strcmp(input, "spg_echo") == 0) {
        cmd = "mode_spg_echo\n";
        current_mode = MODE_SPG_ECHO;
    } else {
        printf("Invalid mode.\n");
        exit(1);
    }

    DWORD written;
    WriteFile(hSerial, cmd, (DWORD)strlen(cmd), &written, NULL);
    printf("Sent mode: %s", cmd);
}

void push_timestamp_packet(uint64_t *data, uint32_t count) {
    TimestampPacket *packet = &timestamp_fifo[fifo_head];
    memcpy(packet->data, data, count * sizeof(uint64_t));
    packet->count = count;
    fifo_head = (fifo_head + 1) % TIMESTAMP_FIFO_DEPTH;
    if (fifo_count < TIMESTAMP_FIFO_DEPTH) {
        fifo_count++;
    }
}

void render_timestamp_packets(SDL_Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);

    uint64_t global_min = UINT64_MAX, global_max = 0;
    int total_samples = 0;
    for (int i = 0; i < fifo_count; ++i) {
        int index = (fifo_head - fifo_count + i + TIMESTAMP_FIFO_DEPTH) % TIMESTAMP_FIFO_DEPTH;
        TimestampPacket *pkt = &timestamp_fifo[index];
        for (uint32_t j = 0; j < pkt->count; ++j) {
            if (pkt->data[j] < global_min) global_min = pkt->data[j];
            if (pkt->data[j] > global_max) global_max = pkt->data[j];
        }
        total_samples += pkt->count;
    }

    if (total_samples == 0 || global_max == global_min) {
        SDL_RenderPresent(renderer);
        return;
    }

    double y_scale = (double)(WINDOW_HEIGHT - 1) / (global_max - global_min);
    double x_step = (double)(WINDOW_WIDTH - 1) / (total_samples - 1);

    const SDL_Color colors[4] = {
        {255, 0, 0, 255},
        {0, 255, 0, 255},
        {0, 0, 255, 255},
        {255, 255, 0, 255}
    };

    int global_index = 0;
    for (int i = 0; i < fifo_count; ++i) {
        int index = (fifo_head - fifo_count + i + TIMESTAMP_FIFO_DEPTH) % TIMESTAMP_FIFO_DEPTH;
        TimestampPacket *pkt = &timestamp_fifo[index];
        SDL_Color c = colors[i % 4];
        SDL_SetRenderDrawColor(renderer, c.r, c.g, c.b, c.a);

        for (uint32_t j = 0; j < pkt->count; ++j) {
            int x = (int)(global_index * x_step);
            int y = (int)((pkt->data[j] - global_min) * y_scale);
            SDL_RenderPoint(renderer, x, WINDOW_HEIGHT - y);
            global_index++;
        }
    }

    SDL_RenderPresent(renderer);
}

int main(void) {
    HANDLE hSerial = CreateFileA(COM_PORT, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hSerial == INVALID_HANDLE_VALUE) {
        printf("Failed to open serial port %s\n", COM_PORT);
        return 1;
    }

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(hSerial, &dcb);
    dcb.BaudRate = BAUD_RATE * 8;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    SetCommState(hSerial, &dcb);

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 1;
    timeouts.ReadTotalTimeoutConstant = 1;
    SetCommTimeouts(hSerial, &timeouts);

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("SDL init failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("Event Camera Viewer", WINDOW_WIDTH, WINDOW_HEIGHT, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL);

    // SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, FRAME_WIDTH, FRAME_HEIGHT);

    SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING, FRAME_WIDTH, FRAME_HEIGHT);

    uint8_t frame_buffer[FRAME_WIDTH * FRAME_HEIGHT] = {0};
    uint32_t rgba_buffer[FRAME_WIDTH * FRAME_HEIGHT] = {0};
    uint8_t read_buf[READ_BUFFER_SIZE];
    uint64_t timestamp_buffer[MAX_TIMESTAMPS];
    DWORD bytesRead = 0;
    uint8_t sync[4] = {0};
    uint8_t decay_index = 0, prev_decay_index = 255;

    prompt_user_for_mode(hSerial);

    SDL_Event e;
    int running = 1;

    while (running) {
        memmove(sync, sync + 1, 3);
        while (1) {
            ReadFile(hSerial, sync + 3, 1, &bytesRead, NULL);
            if (bytesRead == 1) {
                if (compare_marker(sync, marker_eros_surface) ||
                    compare_marker(sync, marker_event_visualize) ||
                    compare_marker(sync, marker_event_count) ||
                    compare_marker(sync, marker_timestamps) ||
                    compare_marker(sync, marker_spg)) break;
            }
            memmove(sync, sync + 1, 3);
        }

        if (compare_marker(sync, marker_eros_surface)) {
            ReadFile(hSerial, &decay_index, 1, &bytesRead, NULL);
            if (decay_index != prev_decay_index) {
                printf("Decay index: %u\n", decay_index);
                prev_decay_index = decay_index;
            }

            size_t offset = 0;
            while (offset < FRAME_WIDTH * FRAME_HEIGHT) {
                size_t to_read = FRAME_WIDTH * FRAME_HEIGHT - offset;
                if (to_read > READ_BUFFER_SIZE) to_read = READ_BUFFER_SIZE;
                ReadFile(hSerial, read_buf, to_read, &bytesRead, NULL);
                memcpy(frame_buffer + offset, read_buf, bytesRead);
                offset += bytesRead;
            }

            for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; ++i) {
                uint8_t v = frame_buffer[i];
                rgba_buffer[i] = (v << 24) | (v << 16) | (v << 8) | 0xFF;
            }

            SDL_UpdateTexture(texture, NULL, rgba_buffer, FRAME_WIDTH * sizeof(uint32_t));
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_RenderClear(renderer);
            SDL_FRect dst_rect = {0, 0, (float)WINDOW_WIDTH, (float)WINDOW_HEIGHT};
            SDL_RenderTexture(renderer, texture, NULL, &dst_rect);
            SDL_RenderPresent(renderer);
        } else if (compare_marker(sync, marker_event_visualize)) {
            size_t offset = 0;
            while (offset < FRAME_WIDTH * FRAME_HEIGHT) {
                size_t to_read = FRAME_WIDTH * FRAME_HEIGHT - offset;
                if (to_read > READ_BUFFER_SIZE) to_read = READ_BUFFER_SIZE;
                ReadFile(hSerial, read_buf, to_read, &bytesRead, NULL);
                memcpy(frame_buffer + offset, read_buf, bytesRead);
                offset += bytesRead;
            }

            memset(rgba_buffer, 0, sizeof(rgba_buffer)); // Transparent by default

            for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; ++i) {
                uint8_t v = frame_buffer[i];
                if (v == 1) {
                    // ON event: #7FC3FF = R:127, G:195, B:255
                    rgba_buffer[i] = (255 << 24) | (255 << 16) | (195 << 8) | 127;
                } else if (v == 2) {
                    // OFF event: #FF8A8A = R:255, G:138, B:138
                    rgba_buffer[i] = (255 << 24) | (138 << 16) | (138 << 8) | 255;
                }
            }

            SDL_UpdateTexture(texture, NULL, rgba_buffer, FRAME_WIDTH * sizeof(uint32_t));
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_RenderClear(renderer);
            SDL_FRect dst_rect = {0, 0, (float)WINDOW_WIDTH, (float)WINDOW_HEIGHT};
            SDL_RenderTexture(renderer, texture, NULL, &dst_rect);
            SDL_RenderPresent(renderer);
        } else if (compare_marker(sync, marker_event_count)) {
            uint32_t event_counts[EVENT_COUNT_BUFFERS] = {0}, deltatime = 0;
            ReadFile(hSerial, (uint8_t*)event_counts, sizeof(event_counts), &bytesRead, NULL);
            ReadFile(hSerial, (uint8_t*)&deltatime, sizeof(uint32_t), &bytesRead, NULL);
            
            uint32_t total = 0;
            printf("Event counts: ");
            for (int i = 0; i < EVENT_COUNT_BUFFERS; ++i) {
                printf("F%d: %u", i + 1, event_counts[i]);
                total += event_counts[i];
                if (i < EVENT_COUNT_BUFFERS - 1) printf(" | ");
            }
            printf(" | Tot: %u | Dt: %u us | %.1f kevt/s\n", total, deltatime, (deltatime > 0) ? total / (deltatime / 1000.0f) : 0.0f);
        } else if (compare_marker(sync, marker_timestamps)) {
            uint32_t count = 0;
            ReadFile(hSerial, (uint8_t*)&count, sizeof(uint32_t), &bytesRead, NULL);
            if (count == 0 || count > MAX_TIMESTAMPS) continue;

            size_t to_read = count * sizeof(uint64_t);
            size_t read_offset = 0;
            while (read_offset < to_read) {
                size_t chunk = (to_read - read_offset > READ_BUFFER_SIZE) ? READ_BUFFER_SIZE : (to_read - read_offset);
                ReadFile(hSerial, read_buf, chunk, &bytesRead, NULL);
                memcpy((uint8_t*)timestamp_buffer + read_offset, read_buf, bytesRead);
                read_offset += bytesRead;
            }

            push_timestamp_packet(timestamp_buffer, count);
            render_timestamp_packets(renderer);
        } else if (compare_marker(sync, marker_spg)) {
            uint8_t result = 0;
            ReadFile(hSerial, &result, 1, &bytesRead, NULL);
            printf("SPG pattern result: %s\n", result ? "OK ✅" : "MISMATCH ❌");
        }

        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_EVENT_QUIT) running = 0;
            else if (e.type == SDL_EVENT_KEY_DOWN) {
                DWORD written;
                SDL_Keycode key = e.key.key;
                if ((key == SDLK_PLUS || key == SDLK_EQUALS) &&
                    (current_mode == MODE_EROS_K3 || current_mode == MODE_EROS_K5 || current_mode == MODE_TIMESTAMPS)) {
                    WriteFile(hSerial, "+", 1, &written, NULL);
                } else if (key == SDLK_MINUS &&
                           (current_mode == MODE_EROS_K3 || current_mode == MODE_EROS_K5 || current_mode == MODE_TIMESTAMPS)) {
                    WriteFile(hSerial, "-", 1, &written, NULL);
                } else if (key == SDLK_s || key == SDLK_S) {
                    WriteFile(hSerial, "s", 1, &written, NULL);
                    printf("Sent 's' command.\n");
                }
            }
        }
    }

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    CloseHandle(hSerial);
    return 0;
}
