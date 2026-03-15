"""
Serial capture tool — connects to Arduino and saves output to file.
You interact with the Arduino through this script (type commands,
see responses, everything gets logged).

Usage:
    python calibration/capture.py --port /dev/cu.usbmodem14101 --output calibration/raw_data.txt

    # On Mac, find your port with:
    ls /dev/cu.usbmodem*
"""

import argparse
import sys
import threading

try:
    import serial
except ImportError:
    print("pyserial not installed. Run: pip install pyserial")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Capture Arduino serial output")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/cu.usbmodem14101)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--output", default="calibration/raw_data.txt", help="Output file")
    args = parser.parse_args()

    print(f"Connecting to {args.port} at {args.baud}...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        print("\nAvailable ports:")
        import serial.tools.list_ports
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device} — {p.description}")
        sys.exit(1)

    print(f"Connected! Logging to {args.output}")
    print("Type commands below (they're sent to Arduino). Ctrl+C to stop.\n")

    outfile = open(args.output, "w")
    running = True

    def read_serial():
        while running:
            try:
                if ser.in_waiting:
                    line = ser.readline().decode("utf-8", errors="replace").rstrip()
                    if line:
                        print(line)
                        outfile.write(line + "\n")
                        outfile.flush()
            except Exception:
                break

    reader = threading.Thread(target=read_serial, daemon=True)
    reader.start()

    try:
        while True:
            user_input = input()
            ser.write((user_input + "\n").encode("utf-8"))
    except (KeyboardInterrupt, EOFError):
        print("\nStopping...")
        running = False
        ser.close()
        outfile.close()
        print(f"Data saved to {args.output}")
        print(f"Now run: python calibration/analyze.py --input {args.output}")


if __name__ == "__main__":
    main()
