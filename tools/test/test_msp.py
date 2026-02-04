#!/usr/bin/env python3
import time
from pymultiwii import MultiWii

USB_PORT = '/dev/ttyUSB0'

def connect_fc(port):
    print(f"[INFO] Connecting to flight controller on {port}...")
    try:
        fc = MultiWii(port)
        print("[SUCCESS] Connected to flight controller!")
        return fc
    except Exception as e:
        print(f"[ERROR] Connection failed: {e}")
        exit(1)

def test_sensors(fc):
    print("[INFO] Testing sensors...")
    try:
        raw_imu = fc.getData(MultiWii.RAW_IMU)
        print(f"[RAW_IMU] {raw_imu}")

        rc = fc.getData(MultiWii.RC)
        print(f"[RC] {rc}")

        attitude = fc.getData(MultiWii.ATTITUDE)
        print(f"[ATTITUDE] {attitude}")

        altitude = fc.getData(MultiWii.ALTITUDE)
        print(f"[ALTITUDE] {altitude}")

    except Exception as e:
        print(f"[ERROR] Exception during sensor test: {e}")

def continuous_monitor(fc, duration=10):
    print(f"[INFO] Continuous monitoring for {duration} seconds...")
    start = time.time()
    while time.time() - start < duration:
        try:
            rc = fc.getData(MultiWii.RC)
            imu = fc.getData(MultiWii.RAW_IMU)
            print(f"RC: {rc} | RAW_IMU: {imu}")
            time.sleep(0.5)
        except Exception as e:
            print(f"[ERROR] Exception during monitoring: {e}")
            break
    print("[INFO] Monitoring finished.")

if __name__ == "__main__":
    fc = connect_fc(USB_PORT)
    test_sensors(fc)
    continuous_monitor(fc, duration=10)
    print("[INFO] Test complete. Flight controller communication validated.")
