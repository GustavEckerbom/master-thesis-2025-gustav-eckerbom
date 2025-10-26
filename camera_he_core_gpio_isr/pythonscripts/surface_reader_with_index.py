import serial
import numpy as np
import matplotlib.pyplot as plt
import threading
import keyboard  # cross-platform key listener

# Serial setup
ser = serial.Serial('COM7', 115200 * 8, timeout=1)

# Constants
sync_marker = b'\xFF\x00\x1F\x7E'
frame_size = 320 * 320

# Shared state
frame_count = 0
current_decay_index = 0
running = True

# Background keypress listener
def handle_keyboard():
    global running
    while running:
        if keyboard.is_pressed('+'):
            ser.write(b'+')
            keyboard.clear_all_hotkeys()
        elif keyboard.is_pressed('-'):
            ser.write(b'-')
            keyboard.clear_all_hotkeys()
        elif keyboard.is_pressed('q'):
            running = False
            break

# Start keyboard thread
threading.Thread(target=handle_keyboard, daemon=True).start()

# Set up plot
plt.ion()
fig, ax = plt.subplots()
img_display = ax.imshow(np.zeros((320, 320), dtype=np.uint8), cmap='gray', vmin=0, vmax=255)
title = ax.set_title("Live Surface Frame")

# Main loop
while running:
    # Search for sync marker
    sync = b''
    while sync != sync_marker:
        byte = ser.read(1)
        if not byte:
            continue
        sync = (sync + byte)[-4:]

    # Read decay index byte
    decay_byte = ser.read(1)
    if not decay_byte:
        continue
    current_decay_index = decay_byte[0]

    # Read image data
    data = bytearray()
    while len(data) < frame_size:
        chunk = ser.read(min(512, frame_size - len(data)))
        if not chunk:
            break
        data.extend(chunk)

    if len(data) != frame_size:
        continue

    # Display
    frame_count += 1
    img = np.frombuffer(data, dtype=np.uint8).reshape((320, 320))
    img_display.set_data(img)
    title.set_text(f"Frame #{frame_count} | Decay Index: {current_decay_index}")
    fig.canvas.draw()
    fig.canvas.flush_events()

print("Exiting.")
