#!/usr/bin/env python3

import time
import board
import busio

I2C_ADDR = 0x31

# Create I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

while not i2c.try_lock():
    pass

def read_register(reg, length):
    """Read one or more bytes starting at register."""
    result = bytearray(length)
    i2c.writeto_then_readfrom(
        I2C_ADDR,
        bytes([reg]),
        result
    )
    return result

def read_u8(reg):
    return read_register(reg, 1)[0]

def read_u16(reg):
    data = read_register(reg, 2)
    return data[0] | (data[1] << 8)

def read_s16(reg):
    value = read_u16(reg)
    if value >= 0x8000:
        value -= 0x10000
    return value

try:
    while True:

        device_id = read_u8(0x00)

        tof_distance = read_u16(0x01)
        tof_strength = read_u8(0x03)

        flow_x = read_s16(0x05)
        flow_y = read_s16(0x07)

        integration_time = read_u16(0x09)

        valid = read_u8(0x0B)
        version = read_u8(0x0C)

        print(
            f"ID=0x{device_id:02X} "
            f"Distance={tof_distance:4d} mm "
            f"Strength={tof_strength:3d} "
            f"FlowX={flow_x:6d} "
            f"FlowY={flow_y:6d} "
            f"dt={integration_time:5d} "
            f"Valid={valid} "
            f"FW={version}"
        )

        # time.sleep(0.05)

except KeyboardInterrupt:
    pass

finally:
    i2c.unlock()