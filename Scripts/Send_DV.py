import subprocess
import sys
import os
import time
import argparse

def camcorder_detected():
    """Check if camcorder shows up on FireWire bus via plugreport."""
    try:
        result = subprocess.run(
            ["plugreport"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True
        )
        output = result.stdout.lower()
        return "node" in output and "guid" in output
    except subprocess.CalledProcessError:
        return False

def wait_for_camcorder(timeout=None, interval=3):
    print("🔎 Waiting for camcorder to appear on FireWire bus...")
    start = time.time()
    while True:
        if camcorder_detected():
            print("✅ Camcorder detected!")
            return True
        if timeout and (time.time() - start) > timeout:
            print("❌ Timeout: No camcorder detected.")
            return False
        time.sleep(interval)

def export_dv_to_camcorder(dv_file):
    """Send DV stream to camcorder using dvconnect."""
    if not os.path.exists(dv_file):
        raise FileNotFoundError(f"{dv_file} not found")

    if not wait_for_camcorder(timeout=None, interval=3):
        sys.exit(1)

    print("🎥 Starting export...")
    cmd = ["dvconnect", "-o"]

    with open(dv_file, "rb") as f:
        try:
            subprocess.run(cmd, stdin=f, check=True)
            print("✅ DV file successfully exported to camcorder.")
        except subprocess.CalledProcessError as e:
            print(f"⚠️ Export failed: {e}")
            sys.exit(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Export a .dv file to a MiniDV camcorder over FireWire")
    parser.add_argument("dv_file", help="Path to the .dv file")
    args = parser.parse_args()

    export_dv_to_camcorder(args.dv_file)
