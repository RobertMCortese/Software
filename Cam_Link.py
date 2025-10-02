import raw1394
import avc
import time
import subprocess
import signal
import RPi.GPIO as GPIO
import shutil
import os

# --- Constants ---
DV_MBYTES_PER_SEC = 3.125    # DV bitrate ~25 Mb/s ≈ 3.125 MB/s
SAFETY_BUFFER_MB = 100       # leave 100MB free
CAPTURE_DIR = "."            # where capture files are saved

# --- GPIO Setup ---
REC_LED_PIN = 23   # Recording LED
IDLE_LED_PIN = 2   # Idle LED
BUTTON_PIN = 17    # Toggle button

GPIO.setmode(GPIO.BCM)
GPIO.setup(REC_LED_PIN, GPIO.OUT)
GPIO.setup(IDLE_LED_PIN, GPIO.OUT)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.output(REC_LED_PIN, GPIO.LOW)
GPIO.output(IDLE_LED_PIN, GPIO.HIGH)  # idle at startup

# --- FireWire Setup ---
handle = raw1394.Raw1394()
handle.set_port(0)

node_id = None
for node in range(handle.get_nodecount()):
    try:
        resp = avc.command(handle, node, bytes([0x01, 0xFF, 0xFF, 0x67, 0,0,0,0]))
        if resp:
            node_id = node
            print(f"Found AV/C device on node {node}")
            break
    except Exception:
        continue

if node_id is None:
    print("No AV/C device found")
    exit(1)

# --- Helpers ---
def check_recording():
    cmd = bytes([0x01, 0x20, 0x00, 0x75, 0x00, 0x00, 0x00, 0x00])
    resp = avc.command(handle, node_id, cmd)
    if not resp:
        return "Unknown"
    status = resp[4]
    if status == 0x60:
        return "Recording"
    elif status == 0x61:
        return "Stopped"
    else:
        return f"Unknown (0x{status:02X})"

def get_recording_time_remaining():
    """Return remaining recording time in seconds (after buffer)."""
    total, used, free = shutil.disk_usage(CAPTURE_DIR)
    free_mb = free / (1024*1024)
    free_mb -= SAFETY_BUFFER_MB
    if free_mb <= 0:
        return 0
    seconds = free_mb / DV_MBYTES_PER_SEC
    return int(seconds)

ffmpeg_proc = None

def start_capture():
    global ffmpeg_proc
    if ffmpeg_proc is None:
        print("▶ Starting FFmpeg capture...")
        filename = os.path.join(CAPTURE_DIR, f"capture_{int(time.time())}.dv")
        ffmpeg_proc = subprocess.Popen([
            "ffmpeg",
            "-f", "iec61883", "-i", "auto",
            "-c", "copy", filename
        ])
        GPIO.output(REC_LED_PIN, GPIO.HIGH)

def stop_capture():
    global ffmpeg_proc
    if ffmpeg_proc is not None:
        print("■ Stopping FFmpeg capture...")
        ffmpeg_proc.send_signal(signal.SIGINT)
        ffmpeg_proc.wait()
        ffmpeg_proc = None
        GPIO.output(REC_LED_PIN, GPIO.LOW)

# --- Polling Toggle ---
polling_enabled = False   # OFF by default
last_state = None

def toggle_polling(channel):
    global polling_enabled, last_state
    polling_enabled = not polling_enabled
    if polling_enabled:
        print("✅ Polling ENABLED")
        GPIO.output(IDLE_LED_PIN, GPIO.LOW)
    else:
        print("❌ Polling DISABLED (Idle)")
        stop_capture()
        GPIO.output(IDLE_LED_PIN, GPIO.HIGH)
        GPIO.output(REC_LED_PIN, GPIO.LOW)
    last_state = None

GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=toggle_polling, bouncetime=500)

# --- Main Loop ---
print("Press button to toggle AV/C polling. Ctrl+C to quit.")
print("❌ Polling DISABLED (Idle)")  # startup status
last_report = 0

try:
    while True:
        if polling_enabled:
            state = check_recording()
            if state != last_state:
                print(f"Camcorder state: {state}")
                if state == "Recording":
                    start_capture()
                elif state == "Stopped":
                    stop_capture()
            last_state = state

            # If recording, check disk space every 10s
            if state == "Recording":
                now = time.time()
                if now - last_report >= 10:
                    secs = get_recording_time_remaining()
                    mins, sec = divmod(secs, 60)
                    print(f"⏱ Recording time remaining: {mins}m {sec}s")

                    if secs <= 0:
                        print("⚠️ Storage almost full! Auto-stopping capture.")
                        stop_capture()
                    last_report = now
        time.sleep(1)

except KeyboardInterrupt:
    print("Exiting...")
    stop_capture()
    GPIO.cleanup()
