import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# --- Configuration ---
SERIAL_PORT = 'COM12'
BAUD_RATE = 115200
FRAME_SIZE = 1024 * 64  # 64 KiB = 16,384 events
IMG_SIZE = 320          # 320x320 image size
MAX_EVENTS = 65536      # FIFO holds 4 full frames of events

# EVT2.0 event types
CD_LOW  = 0x0  # OFF event (red)
CD_HIGH = 0x1  # ON event (blue)

# --- FIFO buffer for events ---
event_fifo = deque(maxlen=MAX_EVENTS)

def receive_frame(ser):
    """Read a full 64KB frame from the serial port."""
    received = bytearray()
    start_time = time.time()
    timeout = 3  # seconds

    while len(received) < FRAME_SIZE:
        chunk = ser.read(FRAME_SIZE - len(received))
        if chunk:
            received.extend(chunk)
            start_time = time.time()  # Reset timeout on progress
        elif time.time() - start_time > timeout:
            print(f"⚠️ Stalled at {len(received)} / {FRAME_SIZE} bytes")
            return None
    return received

def decode_evt2_events(frame_data):
    """Decode EVT2.0 events and add to global FIFO."""
    events = np.frombuffer(frame_data, dtype='<u4')  # little-endian 32-bit

    for evt in events:
        evt_type = (evt >> 28) & 0xF
        if evt_type not in (CD_LOW, CD_HIGH):
            continue

        x = (evt >> 11) & 0x7FF
        y = evt & 0x7FF
        if x >= IMG_SIZE or y >= IMG_SIZE:
            continue

        polarity = 1 if evt_type == CD_HIGH else 0
        event_fifo.append((x, y, polarity))

def render_event_image():
    """Render all events in FIFO into RGB image."""
    img = np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8)
    for x, y, p in event_fifo:
        if p == 1:
            img[y, x, 2] = 255  # Blue = ON
        else:
            img[y, x, 0] = 255  # Red = OFF
    return img

def wait_for_text(ser, expected: str, timeout=2):
    """Wait for specific string response from MCU."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            print(f"<< {line}")
            if expected in line:
                return True
    print(f"⚠️ Did not receive expected response: '{expected}'")
    return False

def main():
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        time.sleep(2)
        print(f"✅ Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
        ser.write(b'startstream\r\n')
        print(">> Sent: startstream")

        # Wait for confirmation message from MCU
        wait_for_text(ser, "Start stream requested")

        # Set up real-time plot
        plt.ion()
        fig, ax = plt.subplots()
        display = ax.imshow(np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8))
        plt.title("EVT2.0 Live Stream (Red = OFF, Blue = ON)")
        plt.axis('off')

        # Main streaming loop
        while True:
            frame = receive_frame(ser)
            if frame is None:
                continue

            decode_evt2_events(frame)
            img = render_event_image()
            display.set_data(img)
            fig.canvas.flush_events()
            plt.pause(0.001)

if __name__ == "__main__":
    main()
