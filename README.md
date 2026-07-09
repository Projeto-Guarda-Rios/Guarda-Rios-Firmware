# Guarda-Rios-Firmware

Firmware and tooling for the **Guarda Rios** water quality monitoring system. This repository contains everything needed to build, flash, and run both the STM32 sensor node and the ESP32 gateway station.

## Repository Structure

```
├── AquaNode/                  # STM32 sensor node firmware (bare-metal)
├── PGR_Station/               # ESP32 gateway station firmware (Arduino)
├── ESP32_Flasher/             # Python host tool for configuring/flashing ESP32 stations
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
| `PGR_Station.ino` | Reads the RS-485 sensor frame, batches samples, and sends authenticated binary UDP packets to the Guarda-Rios ingest server over the SIM7028 NB-IoT modem. |

Upload via Arduino IDE, PlatformIO, or the configurable flashing helper below.

## ESP32_Flasher

**Python host tool** that prepares a temporary configured copy of the ESP32
station sketch, builds it with `arduino-cli`, and uploads it to the ESP32.

The current binary ingest protocol sends a numeric `station_id`, not a text
station name or public key. The flasher asks for a station name for operator
convenience and derives a default numeric ID from it, but the server must map
that numeric ID to the station name. Authentication uses the shared
`STATION_TOKEN` HMAC key from `station_secrets.h`.

```bash
python3 ESP32_Flasher/flash_station.py --port /dev/ttyUSB0
```

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

By default the helper sets `sample-count` to `1`, so a packet is sent every
`--send-interval` seconds. For batching, set `--sample-count`; the packet send
cadence is approximately `send_interval * sample_count` seconds.
The helper sends the modem clock unchanged by default. Use
`--timestamp-offset` to add or subtract seconds when a deployment needs a
fixed correction.

Requires `arduino-cli` with the ESP32 core installed. The default board FQBN is
`esp32:esp32:esp32`; override it with `--fqbn` if the PCB uses a more specific
ESP32 board profile.

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
