import serial
ard_Data = serial.Serial('COM4', 115200)

while True:
    cmd = input("SHITTTTT: ")
    cmd+='\r'
    ard_Data.write(cmd.encode())

