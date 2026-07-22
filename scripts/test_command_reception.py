"""Standalone test script to verify if commands are being received by the drone.

It sends a specific RC command and reads back the MSP_RC values to verify.
"""

import argparse
import os
import struct
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import serial

from control.msp_link import (
    MSP_RC,
    build_msp_request,
    build_msp_set_raw_rc,
    list_serial_ports,
    make_rc_channels,
    read_msp_response,
)

def parse_msp_rc(payload: bytes) -> list[int] | None:
    """Parse MSP_RC (105) payload: Returns list of RC channel values."""
    if len(payload) >= 32:
        return list(struct.unpack("<16H", payload[:32]))
    elif len(payload) >= 16:  # Older FC versions might send 8 channels
        num_ch = len(payload) // 2
        return list(struct.unpack(f"<{num_ch}H", payload[:num_ch * 2]))
    return None

def test_command():
    parser = argparse.ArgumentParser(description="Test if RC commands are received by the Drone")
    parser.add_argument("--port", type=str, default="", help="Serial COM port (e.g. COM6)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--map", type=str, default="AETR", help="RC Channel Map (AETR or TAER)")
    args = parser.parse_args()

    print("=" * 60)
    print(" DRONE COMMAND VERIFICATION TEST")
    print("=" * 60)

    port = args.port
    if not port:
        ports = list_serial_ports()
        if not ports:
            print("[ERROR] No serial COM ports detected on system!")
            sys.exit(1)
        print("\nDetected Serial COM Ports:")
        for idx, (p_dev, p_desc) in enumerate(ports):
            print(f"  [{idx + 1}] {p_desc}")
        try:
            sel = input("\nSelect COM Port Number [1]: ").strip()
            p_idx = int(sel) - 1 if sel.isdigit() and 1 <= int(sel) <= len(ports) else 0
            port = ports[p_idx][0]
        except KeyboardInterrupt:
            print("\nCancelled.")
            sys.exit(0)

    print(f"\nOpening {port} @ {args.baud} baud...")
    try:
        ser = serial.Serial(port, args.baud, timeout=0.05)
        time.sleep(1.0) # Wait for connection
        print(f"[SUCCESS] Connected to {port}!")
    except Exception as e:
        print(f"[ERROR] Failed to open {port}: {e}")
        sys.exit(1)

    # Unique test values to ensure we aren't just reading default mid-points
    test_roll = 1555
    test_pitch = 1666
    test_yaw = 1444
    test_throttle = 1000

    print(f"\n[INFO] Sending Test RC Command: Roll={test_roll}, Pitch={test_pitch}, Yaw={test_yaw}, Throttle={test_throttle}")
    
    # 1. Send the command
    ch = make_rc_channels(
        roll=test_roll, 
        pitch=test_pitch, 
        yaw=test_yaw, 
        throttle=test_throttle,
        channel_map=args.map
    )
    packet = build_msp_set_raw_rc(ch)
    
    # Send a few times in a loop to ensure FC processes it and overrides RX
    for _ in range(10):
        ser.write(packet)
        time.sleep(0.02)
        
    print("\n[INFO] Checking basic drone communication (MSP_STATUS)...")
    ser.reset_input_buffer()
    ser.write(build_msp_request(101)) # MSP_STATUS
    time.sleep(0.2)
    if ser.in_waiting > 0:
        print(f"\033[92m[SUCCESS] Received {ser.in_waiting} bytes from drone. RX communication is working.\033[0m")
    else:
        print("\033[91m[FAILED] Received 0 bytes from drone for MSP_STATUS.\033[0m")
        print("  - Is the drone powered by battery? (Sometimes USB alone isn't enough)")
        print("  - Are you sure this COM port has MSP enabled in Betaflight?")
        print("  - Make sure another program (like Betaflight Configurator) isn't already connected to this COM port.")
    
    # 2. Read back the RC channels
    print("\n[INFO] Verifying command reception via MSP_RC...")
    ser.reset_input_buffer()
    ser.write(build_msp_request(MSP_RC))
    
    success = False
    raw_buffer = bytearray()
    t0 = time.time()
    
    # Read raw bytes manually to see what we get
    while time.time() - t0 < 1.0:
        if ser.in_waiting > 0:
            raw_buffer.extend(ser.read(ser.in_waiting))
            
    if raw_buffer:
        print(f"[DEBUG] Received {len(raw_buffer)} raw bytes for MSP_RC: {raw_buffer.hex()}")
        # Now try to parse it
        idx = 0
        while idx <= len(raw_buffer) - 6:
            if raw_buffer[idx:idx+3] == b"$M>":
                size = raw_buffer[idx+3]
                cmd = raw_buffer[idx+4]
                if idx + 5 + size < len(raw_buffer):
                    payload = raw_buffer[idx+5:idx+5+size]
                    chk = raw_buffer[idx+5+size]
                    
                    expected_chk = size ^ cmd
                    for b in payload:
                        expected_chk ^= b
                    expected_chk &= 0xFF
                    
                    if chk == expected_chk:
                        if cmd == MSP_RC:
                            rc_vals = parse_msp_rc(bytes(payload))
                            if rc_vals:
                                print(f"\n[REPLY] Received RC Channels: {rc_vals[:4]} (First 4 ch)")
                                if test_roll in rc_vals and test_pitch in rc_vals and test_yaw in rc_vals:
                                    success = True
                                    print("\033[92m[SUCCESS] Command was successfully received and applied by the drone!\033[0m")
                                else:
                                    print(f"\033[93m[FAILED] Drone received RC data, but values don't match test values.\033[0m")
                                    print("  Received: " + str(rc_vals))
                                    print("  Expected: " + str([test_roll, test_pitch, test_yaw, test_throttle]))
                            break
                        else:
                            print(f"[DEBUG] Parsed packet cmd={cmd}, size={size}")
                    else:
                        print(f"[DEBUG] Checksum mismatch for cmd={cmd}")
                idx += (6 + size)
            else:
                idx += 1
    
    if not success:
         print("\033[91m[FAILED] No valid MSP_RC response found.\033[0m")

    # Revert to safe neutral values
    print("\n[INFO] Reverting to safe channel values...")
    safe_ch = make_rc_channels(1500, 1500, 1500, 1000, channel_map=args.map)
    ser.write(build_msp_set_raw_rc(safe_ch))

    ser.close()
    print("[INFO] Connection closed.")

if __name__ == "__main__":
    try:
        test_command()
    except KeyboardInterrupt:
        print("\nInterrupted.")
