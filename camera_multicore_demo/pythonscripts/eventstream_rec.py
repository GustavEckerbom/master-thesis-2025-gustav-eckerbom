import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

SERIAL_PORT = 'COM7'
BAUD_RATE = 115200
EVENT_SIZE = 3
HEADER_SIZE = 1
MAX_EVENTS = 50000  # Rolling buffer size
FRAME_WIDTH = 320
FRAME_HEIGHT = 320

def decode_event(packet):
    byte0, byte1, byte2 = packet
    state = (byte0 >> 7) & 0x01
    x = ((byte0 & 0x7F) << 2) | ((byte1 >> 6) & 0x03)
    y = ((byte1 & 0x3F) << 3) | ((byte2 >> 5) & 0x07)
    return x, y, state

def main():
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        print(f"📡 Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
        print("⏳ Listening for event stream...\n")

        # Wait for header
        while True:
            if ser.in_waiting >= HEADER_SIZE:
                header = ser.read(HEADER_SIZE)
                buffer_overrun = header[0] == 0x01
                break
            time.sleep(0.01)

        print(f"🟢 Stream started. Initial buffer overrun: {buffer_overrun}\n")

        # Rolling buffer of events
        x_buffer = deque(maxlen=MAX_EVENTS)
        y_buffer = deque(maxlen=MAX_EVENTS)
        p_buffer = deque(maxlen=MAX_EVENTS)

        # Set up live plot
        plt.ion()
        fig, ax = plt.subplots(figsize=(6, 6))
        scatter = ax.scatter([], [], c=[], cmap='bwr', s=1, vmin=0, vmax=1)
        ax.set_xlim(0, FRAME_WIDTH)
        ax.set_ylim(0, FRAME_HEIGHT)
        ax.invert_yaxis()
        ax.set_title("Live Event Stream (Last 50k Events)")

        last_print = time.time()
        event_count = 0

        while True:
            available = ser.in_waiting

            if available >= EVENT_SIZE:
                bytes_to_read = (available // EVENT_SIZE) * EVENT_SIZE
                data = ser.read(bytes_to_read)

                for i in range(0, len(data), EVENT_SIZE):
                    x, y, p = decode_event(data[i:i+3])
                    x_buffer.append(x)
                    y_buffer.append(y)
                    p_buffer.append(p)
                    event_count += 1

            # Every 1 second: update plot + print stats
            now = time.time()
            if now - last_print >= 1.0:
                if ser.in_waiting >= HEADER_SIZE:
                    header = ser.read(HEADER_SIZE)
                    buffer_overrun = header[0] == 0x01

                print(f"[{time.strftime('%H:%M:%S')}] Events/s: {event_count}, Overrun: {buffer_overrun}")
                event_count = 0
                last_print = now

                # Update scatter plot
                scatter.set_offsets(np.column_stack((x_buffer, y_buffer)))
                scatter.set_array(np.array(p_buffer))
                fig.canvas.draw()
                fig.canvas.flush_events()

if __name__ == "__main__":
    main()