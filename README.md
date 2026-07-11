# Guarda-Rios-Firmware

Firmware and tooling for the **Guarda Rios** water quality monitoring system. This repository contains everything needed to build, flash, and run both the STM32 sensor node and the ESP32 gateway station.

## Repository Structure

```
├── AquaNode/                  # STM32 sensor node firmware (bare-metal)
├── PGR_Station/               # ESP32 gateway station firmware (Arduino)
├── ESP32_Flasher/             # Python host tool for configuring/flashing the Main PCB ESP32 board
├── STM32_Programmer/          # Arduino Nano Every SWD programmer firmware
└── STM32_Flasher/             # Python host tool for flashing the STM32
```

## AquaNode

**Bare-metal firmware for the STM32L053R8** — the sensor node that reads water quality data.

| File | Description |
|------|-------------|
| `main.c` | Core firmware: reads the SEN0554 turbidity sensor via Modbus (USART1), the DS18B20 temperature sensor via one-wire (PA0), and outputs combined readings over RS-485 (USART2). Fully register-level, no RTOS or libc. |
| `Makefile` | Builds `firmware.bin` and `firmware.elf` using `arm-none-eabi-gcc`. |
| `stm32l053r8.ld` | Linker script defining the STM32L053R8 memory layout (64 KB Flash, 8 KB RAM). |

### Build

```bash
cd AquaNode
make        # produces firmware.bin
make clean  # remove build artifacts
```

Requires `arm-none-eabi-gcc` on PATH.

## PGR_Station

**ESP32 Arduino sketch** — the gateway that receives RS-485 data and pushes it to the cloud.

| File | Description |
|------|-------------|
| `PGR_Station.ino` | Wakes from ESP32 timer deep sleep, reads the RS-485 sensor frame, sends authenticated binary UDP packets to the Guarda-Rios ingest server over SIM7028 NB-IoT, then returns to deep sleep. Unsent batch data and the packet counter are retained across timer wakes. |

Upload via Arduino IDE, PlatformIO, or the configurable flashing helper below.

## Main PCB (ESP32 Board) Flasher

The **Main PCB** uses the ESP32 station firmware in `PGR_Station/`.
`ESP32_Flasher/flash_station.py` is the recommended flashing helper for this
board. It prepares a temporary configured copy of the sketch, generates
`station_secrets.h` with the station token, builds it with `arduino-cli`, and
uploads it to the ESP32 over USB serial.

The current binary ingest protocol sends a numeric `station_id`, not a text
station name or public key. The flasher asks for a station name for operator
convenience and derives a default numeric ID from it, but the server must map
that numeric ID to the station name. Authentication uses the shared
`STATION_TOKEN` HMAC key from `station_secrets.h`.

### Requirements

- Python 3.
- `arduino-cli` on PATH.
- ESP32 Arduino core installed for `arduino-cli`.
- USB serial access to the Main PCB ESP32 board.

First-time `arduino-cli` setup:

```bash
arduino-cli core update-index \
  --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32 \
  --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
```

On Linux, the serial port is usually `/dev/ttyUSB0` or `/dev/ttyACM0`. On
Windows, it is usually `COM3` or similar.

### Flash the Main PCB

Connect the Main PCB ESP32 board over USB, then run:

```bash
python3 ESP32_Flasher/flash_station.py --port /dev/ttyUSB0
```

If no station details are passed on the command line, the helper prompts for:

- station name
- numeric station ID sent to the server
- station token/shared key
- send interval
- serial port, when `--port` is omitted

Useful options:

```bash
python3 ESP32_Flasher/flash_station.py \
  --port /dev/ttyUSB0 \
  --station-name Station-A \
  --station-id 1 \
  --station-token '<server-shared-token>' \
  --send-interval 60 \
  --timestamp-offset 3600
```

Use `--station-id` when the backend already has an assigned numeric station
ID. If it is omitted, the helper derives a stable default from
`--station-name`, then lets the operator accept or override it.

By default the helper sets `sample-count` to `1`, so the ESP32 wakes, reads,
sends, and returns to deep sleep every `--send-interval` seconds. For batching,
set `--sample-count`; samples are retained in RTC memory across deep sleeps and
the packet send cadence is approximately `send_interval * sample_count`
seconds. The SIM7028 socket is closed and its radio/baseband is set to minimum
functionality before every ESP32 deep sleep.
The helper sends the modem clock unchanged by default. Use
`--timestamp-offset` to add or subtract seconds when a deployment needs a
fixed correction.

The default board FQBN is `esp32:esp32:esp32`. Override it with `--fqbn` if
the Main PCB is configured with a more specific ESP32 board profile:

```bash
python3 ESP32_Flasher/flash_station.py \
  --port /dev/ttyUSB0 \
  --fqbn esp32:esp32:esp32
```

To verify the generated configuration and build without uploading, use:

```bash
python3 ESP32_Flasher/flash_station.py \
  --compile-only \
  --station-name Station-A \
  --station-id 1 \
  --station-token '<server-shared-token>' \
  --send-interval 60
```

Add `--keep-build-dir` if you need to inspect the temporary configured sketch
after the run.

## STM32_Programmer

**Arduino Nano Every firmware** that turns the board into a custom SWD programmer for the STM32.

| File | Description |
|------|-------------|
| `stm32_programmer.ino` | Bit-bangs the SWD debug protocol (D6=SWDIO, D7=SWCLK, D5=NRST) to erase and write the STM32 flash. Communicates with the host PC over USB serial at 115200 baud. |

Upload this sketch to an Arduino Nano Every before using the flashing tool.

## STM32_Flasher

**Python host tool** that drives the Arduino-based SWD programmer to flash firmware onto the STM32.

| File | Description |
|------|-------------|
| `flash_stm32.py` | Sends the `.bin` file in 128-byte chunks over serial to the programmer board. Handles connect, erase, write, and reset sequences with progress feedback. |

### Usage

```bash
python3 STM32_Flasher/flash_stm32.py <serial_port> <firmware.bin>
# Example:
python3 STM32_Flasher/flash_stm32.py COM3 AquaNode/firmware.bin
```

Requires `pyserial` (`pip install pyserial`).

## RS-485 Frame Format

| Byte | Content |
|------|---------|
| 0 | `0xAA` (sync) |
| 1 | `0x55` (sync) |
| 2–3 | Turbidity (uint16, NTU); `0xFFFF` = sensor offline |
| 4–5 | Temperature (int16, °C × 100); `-9999` = sensor offline |
| 6 | Checksum (XOR of bytes 2–5) |
