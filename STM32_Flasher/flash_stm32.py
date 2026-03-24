#!/usr/bin/env python3
"""
STM32L053R8 SWD Flash Tool

Sends a .bin firmware to an Arduino Nano running stm32_programmer.ino,
which programs the STM32L053R8T6 via SWD (Serial Wire Debug).

Usage:
    python3 flash_stm32.py <serial_port> <firmware.bin>

Example:
    python3 flash_stm32.py /dev/cu.usbserial-1420 firmware.bin
    python3 flash_stm32.py COM3 firmware.bin
"""

import serial
import sys
import struct
import time

# Protocol constants (must match Arduino sketch)
PC_CONNECT = 0x01
PC_ERASE   = 0x02
PC_WRITE   = 0x03
PC_RESET   = 0x05
PC_PING    = 0x06

RESP_ACK   = 0x79
RESP_NACK  = 0x1F
RESP_READY = 0x76

FLASH_BASE = 0x08000000
CHUNK_SIZE = 128  # Must be multiple of 4, max 128


def wait_response(ser, timeout=30):
    ser.timeout = timeout
    data = ser.read(1)
    if len(data) == 0:
        return None
    return data[0]


def check_resp(resp, step_name):
    if resp == RESP_ACK:
        return
    if resp == RESP_NACK:
        print(f"\nERROR: {step_name} - NACK")
        sys.exit(1)
    if resp is None:
        print(f"\nERROR: {step_name} - timeout")
        sys.exit(1)
    print(f"\nERROR: {step_name} - unexpected: {resp:#04x}")
    sys.exit(1)


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <serial_port> <firmware.bin>")
        sys.exit(1)

    port = sys.argv[1]
    bin_file = sys.argv[2]

    with open(bin_file, "rb") as f:
        firmware = f.read()
    print(f"Firmware: {bin_file} ({len(firmware)} bytes)")

    if len(firmware) == 0:
        print("ERROR: Firmware file is empty")
        sys.exit(1)

    if len(firmware) > 0x10000:
        print("ERROR: Firmware too large for STM32L053R8 (max 64KB)")
        sys.exit(1)

    # Pad firmware to multiple of 4 bytes
    while len(firmware) % 4 != 0:
        firmware += b"\xFF"

    # Open serial (Nano resets on connect)
    print(f"Opening {port}...")
    ser = serial.Serial(port, 115200, timeout=5)
    time.sleep(2)
    ser.reset_input_buffer()

    # Ping Arduino
    print("Connecting to Arduino...")
    ser.write(bytes([PC_PING]))
    resp = wait_response(ser, timeout=3)
    if resp not in (RESP_ACK, RESP_READY):
        print("WARNING: No response from Arduino, trying anyway...")

    # Step 1: Connect to STM32 via SWD
    print("Connecting to STM32 via SWD...")
    ser.write(bytes([PC_CONNECT]))
    resp = wait_response(ser, timeout=10)
    check_resp(resp, "SWD connect")
    print("  Connected, core halted, flash unlocked")

    # Step 2: Erase all flash
    print("Erasing 64KB flash (512 pages)...")
    ser.write(bytes([PC_ERASE]))
    resp = wait_response(ser, timeout=120)
    check_resp(resp, "Flash erase")
    print("  Erase complete")

    # Step 3: Write firmware
    print("Writing firmware...")
    total = len(firmware)
    offset = 0
    start_time = time.time()

    while offset < total:
        chunk = firmware[offset : offset + CHUNK_SIZE]
        addr = FLASH_BASE + offset
        length = len(chunk)

        packet = bytes([PC_WRITE])
        packet += struct.pack(">I", addr)
        packet += bytes([length])
        packet += chunk
        ser.write(packet)

        resp = wait_response(ser, timeout=30)
        check_resp(resp, f"Write at {addr:#010x}")

        offset += length
        pct = offset * 100 // total
        bar = "#" * (pct // 5) + "." * (20 - pct // 5)
        elapsed = time.time() - start_time
        speed = offset / elapsed if elapsed > 0 else 0
        print(
            f"\r  [{bar}] {pct:3d}% ({offset}/{total}) {speed:.0f} B/s",
            end="",
            flush=True,
        )

    elapsed = time.time() - start_time
    print(f"\n  Write complete in {elapsed:.1f}s")

    # Step 4: Reset STM32
    print("Resetting STM32...")
    ser.write(bytes([PC_RESET]))
    wait_response(ser, timeout=3)

    print("Done! STM32 should now be running the new firmware.")
    ser.close()


if __name__ == "__main__":
    main()
