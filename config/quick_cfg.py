#!/usr/bin/env python3
# file: quick_cfg.py
# pip install pyserial

"""
Send a config to ESP32
python quick_cfg.py /dev/ttyACM0 --mode write --json config.json

Print the config on the ESP32
python quick_cfg.py /dev/ttyACM0 --mode read --out current.json


(Linux/macOS: use /dev/ttyACM0 or /dev/ttyUSB0 ; windows ... e.g. COM6)

"""

import argparse, json, time, sys
import serial
from serial.serialutil import SerialException

def wait_and_open(port, baud, probe_interval, resp_timeout, dtr_low):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = resp_timeout
    ser.write_timeout = resp_timeout
    ser.rtscts = False
    if dtr_low:
        ser.dtr = False  # avoid auto-reset on some boards
    print(f"Waiting for {port} ... (Ctrl+C to quit)")
    while True:
        try:
            ser.open()
            break
        except SerialException:
            time.sleep(probe_interval)
    time.sleep(0.15)
    ser.setRTS(False)
    ser.setDTR(not dtr_low)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser

def read_lines_until(ser, timeout):
    t0 = time.time()
    lines = []
    while time.time() - t0 < timeout:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        lines.append(line)
        if line.startswith("OK") or line.startswith("ERR"):
            break
    return lines

def main():
    ap = argparse.ArgumentParser(description="Read/Write ESP32 config via tiny serial protocol.")
    ap.add_argument("port", help="COMx or /dev/ttyACMx|USBx")
    ap.add_argument("--mode", choices=["read","write"], default="write")
    ap.add_argument("--json", help="Path to config JSON (required for --mode write)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--probe-interval", type=float, default=0.2)
    ap.add_argument("--resp-timeout", type=float, default=15.0)
    ap.add_argument("--dtr-low", action="store_true", help="Hold DTR low to avoid auto-reset on open")
    ap.add_argument("--out", help="When reading, save JSON to this file as well")
    ap.add_argument("--peer-mac", help="Override peer MAC (format: AA:BB:CC:DD:EE:FF)")
    ap.add_argument("--wifi-chan", type=int, help="Override wifi channel (ex: 10)")
    ap.add_argument("--esp-interval", type=int, help="The esp messaging interval in microseconds. A minimum of 1000 = 1ms is enforced internally.")
    args = ap.parse_args()

    if args.mode == "write" and not args.json and not args.peer_mac and not args.wifi_chan and not args.esp_interval:
        ap.error("--json or --peer-mac or --wifi-chan or --esp-interval required for --mode write")

    ser = wait_and_open(args.port, args.baud, args.probe_interval, args.resp_timeout, args.dtr_low)

    try:
        if args.mode == "write":
            print("Writing...")
            if args.json:
                with open(args.json, "r", encoding="utf-8") as f:
                    data = json.load(f)
            else:
                data = json.loads("{}")

            if args.peer_mac:
                data["espnow_peer"] = args.peer_mac
            if args.wifi_chan:
                data["wifi_channel"] = args.wifi_chan
            if args.esp_interval:
                data["esp_interval_us"] = abs(args.esp_interval)

            payload = json.dumps(data, separators=(",", ":")) + "\n"

            print("JSON:", payload)

            ser.write(b"WRITE\n")
            ser.flush()
            time.sleep(1)

            # not working on windows : ser.write(payload.encode("utf-8"))
            buf = payload.encode("utf-8")

            total = 0
            while total < len(buf):
                n = ser.write(buf[total:])
                if n is None or n == 0:
                    raise RuntimeError(f"Serial write stalled at {total}/{len(buf)} bytes")
                total += n

            ser.flush()
            print(f"Sent {total}/{len(buf)} bytes")

            lines = read_lines_until(ser, args.resp_timeout)
            for L in lines: print(L)
            if any(L.startswith("OK") for L in lines):
                print("Successfully wrote config.")
                return 0
            print("Error. Writing config was not successful.")
            return 2

        else:  # read
            print("Reading...")
            ser.write(b"READ\n")
            ser.flush()
            lines = read_lines_until(ser, args.resp_timeout)
            # Expect one JSON line then OK
            #print("Got:", lines)
            json_line = None
            for L in lines[1:]:
                if L.startswith("{") and L.endswith("}"):
                    json_line = L
                print(L)
            print("JSON Line: ",json_line)
            if json_line and args.out:
                with open(args.out, "w", encoding="utf-8") as f:
                    f.write(json_line + "\n")
            # exit code
            if any(L.startswith("OK") for L in lines):
                return 0
            return 3
    except OSError as e:
        print(e)
    finally:
        ser.close()

if __name__ == "__main__":
    sys.exit(main())
