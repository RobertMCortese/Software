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
GPIO.setmode(GPIO.BCM)
GPIO.setup(REC_LED_PIN, GPIO.OUT)
GPIO.output(REC_LED_PIN, GPIO.LOW)

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

ffmpeg_proc = None
record_start_time = None
initial_secs_remaining = None

def calculate_remaining_time():
    """Estimate remaining recording time based on start free space and elapsed time."""
    if record_start_time is None or initial_secs_remaining is None:
        return 0
    elapsed = time.time() - record_start_time
    est_remaining = initial_secs_remaining - int(elapsed)
    return max(est_remaining, 0)

def start_capture():
    global ffmpeg_proc, record_start_time, initial_secs_remaining
    if ffmpeg_proc is None:
        print("▶ Starting FFmpeg capture...")
        filename = os.path.join(CAPTURE_DIR, f"capture_{int(time.time())}.dv")
        ffmpeg_proc = subprocess.Popen([
            "ffmpeg",
            "-f", "iec61883", "-i", "auto",
            "-c", "copy", filename
        ])
        GPIO.output(REC_LED_PIN, GPIO.HIGH)

        # Compute initial space & time left once
        total, used, free = shutil.disk_usage(CAPTURE_DIR)
        free_mb = free / (1024*1024)
        free_mb -= SAFETY_BUFFER_MB
        if free_mb < 0:
            free_mb = 0
        initial_secs_remaining = int(free_mb / DV_MBYTES_PER_SEC)
        record_start_time = time.time()
        mins, sec = divmod(initial_secs_remaining, 60)
        print(f"⏱ Initial recording time available: {mins}m {sec}s")

def stop_capture():
    global ffmpeg_proc, record_start_time, initial_secs_remaining
    if ffmpeg_proc is not None:
        print("■ Stopping FFmpeg capture...")
        ffmpeg_proc.send_signal(signal.SIGINT)
        ffmpeg_proc.wait()
        ffmpeg_proc = None
        GPIO.output(REC_LED_PIN, GPIO.LOW)
    record_start_time = None
    initial_secs_remaining = None

# --- Main Loop ---
print("Polling camcorder state. Ctrl+C to quit.")
last_state = None
last_report = 0

try:
    while True:
        state = check_recording()
        if state != last_state:
            print(f"Camcorder state: {state}")
            if state == "Recording":
                start_capture()
            elif state == "Stopped":
                stop_capture()
        last_state = state

        # If recording, report estimated time left every 10s
        if state == "Recording" and record_start_time is not None:
            now = time.time()
            if now - last_report >= 10:
                secs = calculate_remaining_time()
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
