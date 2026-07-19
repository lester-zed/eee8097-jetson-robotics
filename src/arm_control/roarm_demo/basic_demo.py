import serial

ser = serial.Serial(
    "/dev/ttyUSB0",
    baudrate=115200,
    timeout=1,
    dsrdtr=False,
)

ser.setRTS(False)
ser.setDTR(False)

while True:
    command = input("JSON> ")
    ser.write((command + "\n").encode("utf-8"))

    response = ser.readline().decode("utf-8", errors="replace")
    if response:
        print("RX:", response.strip())
