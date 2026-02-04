import glob

def list_unique_usb_ports():
    ports = glob.glob("/dev/serial/by-id/*")
    if not ports:
        print("No unique USB serial devices found.")
    for port in ports:
        print(port)

if __name__ == "__main__":
    list_unique_usb_ports()
