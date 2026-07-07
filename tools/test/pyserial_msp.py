import serial

port = "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0"

try:
    ser = serial.Serial(port, baudrate=115200, timeout=1)
    print(f"Successfully opened {port}")
    ser.close()
except Exception as e:
    print(f"Failed to open {port}: {e}")