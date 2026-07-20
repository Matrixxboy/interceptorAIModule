"""Standalone Drone Serial Tool to Check Telemetry, Arming Disable Flags, and Arm/Disarm via MSP.

Connects to Flight Controller over MSP (MultiWii Serial Protocol), reads live telemetry
(Attitude, Battery Voltage, Altitude, Status & Arming Disable Flags), and sends 50 Hz MSP RC Arming packets.

Usage:
  python scripts/check_telemetry_and_arm.py [--port COM6] [--baud 115200] [--arm] [--map AETR]
"""

from __future__ import annotations

import argparse
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import serial

from control.msp_link import (
    ARM_HIGH,
    ARM_LOW,
    MSP_ALTITUDE,
    MSP_ANALOG,
    MSP_ATTITUDE,
    MSP_STATUS,
    MSP_STATUS_EX,
    build_msp_request,
    build_msp_set_raw_rc,
    list_serial_ports,
    make_rc_channels,
    parse_msp_altitude,
    parse_msp_analog,
    parse_msp_attitude,
    parse_msp_status_ex,
    read_msp_response,
)


def query_telemetry(ser: serial.Serial) -> dict:
    """Send MSP requests for Attitude, Analog, Altitude, Status_EX and parse responses."""
    data = {}
    for cmd in [MSP_ATTITUDE, MSP_ANALOG, MSP_ALTITUDE, MSP_STATUS_EX]:
        ser.write(build_msp_request(cmd))
        res = read_msp_response(ser, timeout=0.04)
        if res:
            res_cmd, payload = res
            if res_cmd == MSP_ATTITUDE:
                att = parse_msp_attitude(payload)
                if att:
                    data.update(att)
            elif res_cmd == MSP_ANALOG:
                ana = parse_msp_analog(payload)
                if ana:
                    data.update(ana)
            elif res_cmd == MSP_ALTITUDE:
                alt = parse_msp_altitude(payload)
                if alt:
                    data.update(alt)
            elif res_cmd == MSP_STATUS_EX or res_cmd == MSP_STATUS:
                st = parse_msp_status_ex(payload)
                if st:
                    data.update(st)
    return data


def print_telemetry_hud(data: dict) -> None:
    roll = data.get("roll_deg", 0.0)
    pitch = data.get("pitch_deg", 0.0)
    yaw = data.get("yaw_deg", 0.0)
    vbat = data.get("vbat_volts", 0.0)
    rssi = data.get("rssi", 0)
    alt = data.get("alt_m", 0.0)
    armed = data.get("armed", False)
    disable_reasons = data.get("arming_disable_reasons", [])

    status_str = "\033[92mARMED\033[0m" if armed else "\033[91mDISARMED\033[0m"
    reasons_str = f" [BLOCKED BY: {', '.join(disable_reasons)}]" if (not armed and disable_reasons) else ""

    print(
        f"\r[TELEMETRY] State:{status_str}{reasons_str} | Roll:{roll:+5.1f}° Pitch:{pitch:+5.1f}° Yaw:{yaw:+5.1f}° | VBat:{vbat:4.1f}V | Alt:{alt:4.1f}m | RSSI:{rssi}       ",
        end="",
        flush=True,
    )


def stream_telemetry_loop(ser: serial.Serial, duration_s: float = 15.0) -> None:
    print(f"\n--- Streaming Telemetry for {duration_s} seconds (Press Ctrl+C to stop) ---")
    t0 = time.time()
    try:
        while time.time() - t0 < duration_s:
            data = query_telemetry(ser)
            print_telemetry_hud(data)
            time.sleep(0.08)
        print("\n--- Stream Finished ---")
    except KeyboardInterrupt:
        print("\n--- Stopped by user ---")


def send_arm_command(
    ser: serial.Serial,
    arm: bool,
    duration_s: float = 5.0,
    aux_channel: int = 1,
    channel_map: str = "AETR",
) -> None:
    action = "ARMING" if arm else "DISARMING"
    print(f"\n[DANGER] Transmitting 50 Hz MSP RC {action} command on AUX{aux_channel} (Map={channel_map}) for {duration_s}s...")

    aux1_val = 1800 if (aux_channel == 1 and arm) else 1000
    aux2_val = 1800 if (aux_channel == 2 and arm) else 1000
    aux3_val = 1800 if (aux_channel == 3 and arm) else 1000
    aux4_val = 1800 if (aux_channel == 4 and arm) else 1000

    ch = make_rc_channels(
        roll=1500,
        pitch=1500,
        yaw=1500,
        throttle=1000,
        aux1=aux1_val,
        aux2=aux2_val,
        aux3=aux3_val,
        aux4=aux4_val,
        channel_map=channel_map,
    )
    packet = build_msp_set_raw_rc(ch)

    t0 = time.time()
    last_print = 0.0
    armed_detected = False

    try:
        while time.time() - t0 < duration_s:
            ser.write(packet)

            now = time.time()
            if now - last_print >= 0.15:
                last_print = now
                data = query_telemetry(ser)
                print_telemetry_hud(data)
                if data.get("armed", False):
                    armed_detected = True

            time.sleep(0.02)  # 50 Hz transmission loop
    except KeyboardInterrupt:
        print("\nInterrupted by user!")

    # Return to neutral safe channels after test
    safe_ch = make_rc_channels(1500, 1500, 1500, 1000, aux1=1000, aux2=1000, aux3=1000, aux4=1000, channel_map=channel_map)
    ser.write(build_msp_set_raw_rc(safe_ch))

    print(f"\n[OK] {action} sequence finished.")
    if arm:
        if armed_detected:
            print("\033[92m[SUCCESS] Flight Controller ARMED successfully!\033[0m")
        else:
            print("\033[91m[NOTICE] Flight Controller did NOT arm.\033[0m Check active arming disable reasons above (e.g., ANGLE, THROTTLE, RX_FAILSAFE, or MSP RX mode disabled in Betaflight).")


def main() -> None:
    parser = argparse.ArgumentParser(description="MSP Telemetry & Drone Arming Test Tool")
    parser.add_argument("--port", type=str, default="", help="Serial COM port (e.g. COM6, /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--arm", action="store_true", help="Directly test ARM command for 5 seconds")
    parser.add_argument("--map", type=str, default="AETR", help="RC Channel Map (AETR or TAER)")
    args = parser.parse_args()

    print("=" * 80)
    print(" FPV DRONE TELEMETRY & MSP ARMING BENCH TOOL")
    print("=" * 80)

    port = args.port
    if not port:
        ports = list_serial_ports()
        if not ports:
            print("[ERROR] No serial COM ports detected on system!")
            sys.exit(1)
        print("\nDetected Serial COM Ports:")
        for idx, (p_dev, p_desc) in enumerate(ports):
            print(f"  [{idx + 1}] {p_desc}")
        sel = input("\nSelect COM Port Number [1]: ").strip()
        p_idx = int(sel) - 1 if sel.isdigit() and 1 <= int(sel) <= len(ports) else 0
        port = ports[p_idx][0]

    print(f"\nOpening {port} @ {args.baud} baud...")
    try:
        ser = serial.Serial(port, args.baud, timeout=0.05)
        time.sleep(0.5)
        print(f"[SUCCESS] Connected to {port}!")
    except Exception as e:
        print(f"[ERROR] Failed to open {port}: {e}")
        sys.exit(1)

    if args.arm:
        send_arm_command(ser, arm=True, duration_s=5.0, channel_map=args.map)
        ser.close()
        return

    while True:
        print("\n" + "-" * 40)
        print(" MENU OPTIONS:")
        print("  1. Stream Live Telemetry & Arming Disable Flags")
        print("  2. Send DISARM Command (AUX1-4 = 1000)")
        print("  3. Test ARM DRONE on AUX1 (Channel 5 = 1800)")
        print("  4. Test ARM DRONE on AUX2 (Channel 6 = 1800)")
        print("  5. Test ARM DRONE on AUX3 (Channel 7 = 1800)")
        print("  6. Exit")
        print("-" * 40)
        choice = input("Select option (1-6): ").strip()

        if choice == "1":
            stream_telemetry_loop(ser, duration_s=15.0)
        elif choice == "2":
            send_arm_command(ser, arm=False, duration_s=2.0, channel_map=args.map)
        elif choice in ["3", "4", "5"]:
            aux_idx = int(choice) - 2
            confirm = input(f"\n[WARNING] ARE PROPELLERS REMOVED? Type 'YES' to test ARM on AUX{aux_idx}: ").strip()
            if confirm == "YES":
                send_arm_command(ser, arm=True, duration_s=5.0, aux_channel=aux_idx, channel_map=args.map)
            else:
                print("Arming cancelled.")
        elif choice == "6" or choice.lower() == "q":
            print("Exiting...")
            break

    ser.close()


if __name__ == "__main__":
    main()
