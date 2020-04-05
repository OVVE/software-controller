import serial


def main():
    ser = serial.Serial('/dev/ttyACM0')
    ser.flushInput()

    while True:
        ser_bytes = ser.readline()
        decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
        print(decoded_bytes)


if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        pass
