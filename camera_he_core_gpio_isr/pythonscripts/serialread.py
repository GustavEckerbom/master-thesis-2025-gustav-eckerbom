import serial

with serial.Serial('COM7', 115200, timeout=1) as ser:  # Replace COM3 with your COM port
    with open("log.txt", "w", encoding="utf-8") as f:
        while True:
            try:
                line = ser.readline()  # Read until \n
                if line:
                    text = line.decode('utf-8', errors='replace')
                    print(text, end='')   # Already contains \n, don't add another
                    f.write(text)
                    f.flush()
            except KeyboardInterrupt:
                print("\nExiting...")
                break
