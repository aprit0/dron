#!/usr/bin/env python3

import time
import psutil
import socket
import shutil
import subprocess
import RPi.GPIO as GPIO
from PIL import Image, ImageDraw, ImageFont
import board
import busio
import adafruit_ssd1306

# ---------------- CONFIG ----------------
WIDTH = 128
HEIGHT = 64
FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_SIZE = 9
LINE_HEIGHT = FONT_SIZE + 1

ETH_IFACE = "enp3s0"   # change to your wired interface
WIFI_IFACE = "wlp2s0"  # change to your wireless interface

BUTTON_PIN = 21         # BCM pin for shutdown button

# ---------------- SETUP ----------------
i2c = busio.I2C(board.SCL, board.SDA)
oled = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c)
oled.fill(0)
oled.show()
time.sleep(0.1)

font = ImageFont.truetype(FONT_PATH, FONT_SIZE)

# GPIO setup for shutdown button
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ---------------- FUNCTIONS ----------------
def get_ip(interface):
    try:
        addrs = psutil.net_if_addrs().get(interface, [])
        for addr in addrs:
            if addr.family == socket.AF_INET:
                return addr.address
    except Exception:
        pass
    return "---"

def get_tailscale_ip():
    try:
        out = subprocess.check_output(["tailscale", "ip", "-4"], timeout=1)
        return out.decode().strip() or "---"
    except Exception:
        return "---"

def draw_status():
    image = Image.new("1", (WIDTH, HEIGHT))
    draw = ImageDraw.Draw(image)

    # Get system info
    eth_ip = get_ip(ETH_IFACE)
    wifi_ip = get_ip(WIFI_IFACE)
    ts_ip = get_tailscale_ip()
    cpu = psutil.cpu_percent(percpu=True)
    used, total, _ = shutil.disk_usage("/")
    used_gb = used // (1024**3)
    total_gb = total // (1024**3)

    y = 2  # small offset to avoid OLED line artifact

    # Network info
    draw.text((0, y), f"TS:   {ts_ip}", font=font, fill=255)+3
    y += LINE_HEIGHT
    draw.text((0, y), f"WIFI: {wifi_ip}", font=font, fill=255)
    y += LINE_HEIGHT
    draw.text((0, y), f"ETH:  {eth_ip}", font=font, fill=255)
    y += LINE_HEIGHT
    

    # CPU cores split 2 per line
    group1 = " ".join([f"C{i}:{int(c)}%" for i, c in enumerate(cpu[:2])])
    group2 = " ".join([f"C{i}:{int(c)}%" for i, c in enumerate(cpu[2:4])])
    # Add alert if any core 0 or 1 >=90%
    alert_text = " ---" if any(c >= 90 for c in cpu) else ""
    draw.text((0, y), group1 + alert_text, font=font, fill=255)
    y += LINE_HEIGHT
    draw.text((0, y), group2 + alert_text, font=font, fill=255)
    y += LINE_HEIGHT

    # Disk usage at bottom
    draw.text((0, HEIGHT - LINE_HEIGHT), f"Disk: {used_gb}/{total_gb}GB", font=font, fill=255)

    oled.image(image)
    oled.show()

def shutdown_display():
    oled.fill(0)
    draw = ImageDraw.Draw(Image.new("1", (WIDTH, HEIGHT)))
    draw.text((0, HEIGHT//2 - LINE_HEIGHT//2), "SHUTTING DOWN...", font=font, fill=255)
    oled.image(draw.im)
    oled.show()

# ---------------- MAIN LOOP ----------------
try:
    while True:
        if GPIO.input(BUTTON_PIN) == GPIO.LOW:
            # Shutdown pressed
            shutdown_display()
            time.sleep(0.5)  # small delay to let message show
            subprocess.run(["sudo", "shutdown", "-h", "now"])
            break

        draw_status()
        time.sleep(1)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    GPIO.cleanup()
    oled.fill(0)
    oled.show()
