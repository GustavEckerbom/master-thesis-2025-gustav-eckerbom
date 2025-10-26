import serial
import numpy as np
import matplotlib.pyplot as plt

# Open the serial port
ser = serial.Serial('COM7', 115200 * 8, timeout=1)

# Constants
sync_marker = b'\xFF\x00\x1F\x7E'
frame_size = 320 * 320

# Set up live plot
plt.ion()
fig, ax = plt.subplots()
img_display = ax.imshow(np.zeros((320, 320), dtype=np.uint8), cmap='gray', vmin=0, vmax=255)
ax.set_title("Live Surface Frame")

frame_count = 0

while True:
    # Search for sync marker
    sync = b''
    while sync != sync_marker:
        byte = ser.read(1)
        if not byte:
            continue
        sync = (sync + byte)[-4:]

    # Read full image data
    data = bytearray()
    while len(data) < frame_size:
        chunk = ser.read(min(512, frame_size - len(data)))
        if not chunk:
            print("Timeout while reading image data.")
            break
        data.extend(chunk)

    if len(data) != frame_size:
        print(f"Incomplete frame: got {len(data)} bytes.")
        continue

    # Display image
    frame_count += 1
    img = np.frombuffer(data, dtype=np.uint8).reshape((320, 320))
    img_display.set_data(img)
    ax.set_title(f"Live Surface Frame #{frame_count}")
    fig.canvas.draw()
    fig.canvas.flush_events()
