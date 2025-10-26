import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

SERIAL_PORT = 'COM7'
BAUD_RATE = 115200
FRAME_WIDTH = 1024
FRAME_HEIGHT = 64
FRAME_SIZE = FRAME_WIDTH * FRAME_HEIGHT

COMMAND_OUTPUT_FILES = {
    'printframe': 'frame.raw',
    'printframe1': 'frame1.raw',
    'printframe2': 'frame2.raw',
    'printframe3': 'frame3.raw',
    'printframe4': 'frame4.raw',
}

COMMAND_ACK_STRINGS = {
    'printframe': 'Print new frame',
    'printframe1': 'Print new frame 1',
    'printframe2': 'Print new frame 2',
    'printframe3': 'Print new frame 3',
    'printframe4': 'Print new frame 4',
}

def wait_for_ack(ser, expected_str):
    print(f"<< Waiting for: '{expected_str}'")
    while True:
        line = ser.readline().decode('ascii', errors='ignore').strip()
        if line:
            print(f"<< {line}")
            if expected_str in line:
                break

def receive_frame(ser, output_file):
    print(f"<< Receiving {FRAME_SIZE} bytes...")
    received = bytearray()
    start_time = time.time()
    stall_timeout = 3

    while len(received) < FRAME_SIZE:
        chunk = ser.read(FRAME_SIZE - len(received))
        if chunk:
            received.extend(chunk)
            start_time = time.time()  # Reset stall timer on progress
        elif time.time() - start_time > stall_timeout:
            print(f"⚠️ Transfer stalled at {len(received)} / {FRAME_SIZE} bytes")
            break

    if len(received) == FRAME_SIZE:
        print("✅ Frame fully received")
    else:
        print("❌ Incomplete frame received")

    with open(output_file, 'wb') as f:
        f.write(received)

    print(f"Frame saved to '{output_file}' ({len(received)} bytes)")
    ser.reset_input_buffer()
    ser.write(b'\r\n')

    # Display the frame
    if len(received) == FRAME_SIZE:
        try:
            img = np.frombuffer(received, dtype=np.uint8).reshape((FRAME_HEIGHT, FRAME_WIDTH))
            plt.imshow(img, cmap='gray')
            plt.title(f"Received Frame: {output_file}")
            plt.axis('off')
            plt.show(block=False)
        except Exception as e:
            print(f"⚠️ Could not display image: {e}")

def read_serial_response(ser, timeout_sec=2.0):
    """Read and print lines from the serial port with dynamic timeout."""
    timeout = time.time() + timeout_sec
    while time.time() < timeout:
        line = ser.readline().decode('ascii', errors='ignore').strip()
        if line:
            print(f"<< {line}")
            # Extend timeout slightly on each new line
            timeout = time.time() + 1.0
        else:
            break

def main():
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        time.sleep(2)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.\n")

        while True:
            cmd = input("Enter command (e.g. printframe1, printframe2, overrun, exit): ").strip()
            if not cmd:
                continue
            if cmd.lower() == 'exit':
                print("Exiting.")
                break

            ser.write((cmd + '\r\n').encode('ascii'))
            print(f">> Sent: {cmd}")

            if cmd in COMMAND_OUTPUT_FILES:
                expected_ack = COMMAND_ACK_STRINGS[cmd]
                wait_for_ack(ser, expected_ack)
                receive_frame(ser, COMMAND_OUTPUT_FILES[cmd])
            else:
                read_serial_response(ser)

if __name__ == "__main__":
    main()

