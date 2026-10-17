# VorsprungLogger

An Audi-focused vehicle telemetry logger for the [Freematics ONE+ Model B](https://freematics.com/products/freematics-one-plus-model-b/) (ESP32). It collects OBD-II live data plus a large set of manufacturer-specific UDS signals from the VAG **EA897 evo** 3.0 TDI powertrain (Audi A6 Allroad) and streams them, with GNSS position, to a [Traccar](https://www.traccar.org) server over a cellular link. Anything that cannot be sent is spooled to microSD and replayed, stamped with its original capture time, when the link returns.

VorsprungLogger is derived from the Freematics **Telelogger** by Stanley Huang and keeps his original BSD copyright (see [Credits](#credits)). It trims the upstream's broad feature set down to one specific vehicle and use case.

## What it collects

* **OBD-II live PIDs** — speed, RPM, throttle, engine load every cycle; fuel pressure, timing, coolant and intake temperature rotating behind them.
* **UDS DIDs (51 signals across two ECUs)** — engine (`0x7E0`) and SCR/reductant (`0x7EA`): EGTs, oil/fuel/charge pressures, MAF, torque, DPF soot/ash/regen counters, NOx, AdBlue tank and dosing, odometer. Decoded into engineering units and emitted as synthetic PIDs in the `0x200+` range. The table is [`include/uds_dids.h`](include/uds_dids.h), the one file to change for a different car.
* **Derived-inputs group** — RPM, MAF, charge pressure, IAT, fuel rate and engine torque are co-read atomically so they always share a packet, making downstream volumetric-efficiency / power / AFR / BSFC math plain arithmetic. See [`SPDD.md`](SPDD.md) §8.
* **GNSS** — position, altitude, speed, heading, satellite count and HDOP from an external u-blox module on the molex header, falling back to the board's internal receiver if none is detected. The receiver is never powered down, so every trip starts with a hot fix. See [`SPDD.md`](SPDD.md) §9.
* **MEMS** — accelerometer, used for standby/wake and logged as 3-axis acceleration.
* **Housekeeping** — battery voltage, cellular signal level, device temperature.

## Scheduling

UDS DIDs are polled by a tiered countdown scheduler (`UDS_FAST` / `UDS_SLOW` / `UDS_GLACIAL`) with a per-cycle read budget, plus the atomic `UDS_GROUP` tier for the derived-inputs group. See `processUDS` in [`src/main.cpp`](src/main.cpp) and the full design in [`SPDD.md`](SPDD.md) §7.

## Transmission, spooling and time

* Telemetry over **cellular UDP** to a Traccar endpoint, framed as `devid#PID:value,...*checksum` (the Freematics protocol, port 5170). Stock Traccar stores the UDS PIDs as `io<decimal>` attributes; the author's deployment runs a fork whose decoder gives them names.
* On a cell outage, unsent packets spool to microSD (`/SPOOL/<seq>.PKT`, one file per outage) and drain back to the server in the gaps between live packets once the link returns. The SD card holds nothing else; there is no continuous logging. See [`SPDD.md`](SPDD.md) §14.
* **Every record carries its own capture time**, derived from a monotonic tick plus a UTC anchor taken from GNSS, or from the cellular network when there is no fix yet. Replayed records are filed at the moment they were captured, not the moment they arrived; records captured before any clock source existed are back-stamped on the way out of the spool. See [`TIME.md`](TIME.md).
* There is no end-to-end acknowledgement. A packet is spooled when the modem fails to confirm the send, which sometimes happens after the datagram has already gone out, so Traccar receives some rows twice. Traccar accepts the duplicates.

## In-car feedback

The buzzer plays a rising C-major arpeggio when the cellular link comes up, a short high two-note ping when the external GNSS module is detected, and a low descending three-note grumble when it is not and the internal receiver took over. The console boot line says the same thing: `GNSS:EXTERNAL`, `GNSS:INTERNAL` or `GNSS:NONE`.

## Hardware

* Freematics ONE+ **Model B** (ESP32-D0WDQ6, no PSRAM), SIM7670G-LN cellular module.
* MicroSD card (FAT32), used only as the outage spool.
* A cellular SIM (APN via `CELL_APN` in [`include/config.h`](include/config.h), overridable at runtime from NVS).
* Optional external u-blox GNSS module on the molex header. It shares UART0's receive pin with the console, so the **console runs at 38400 baud**. The module is probed at several rates at boot and told to speak 38400.

## Building

Built with [PlatformIO](https://platformio.org/):

```sh
cp include/secrets.h.example include/secrets.h   # set your Traccar endpoint
pio run -e esp32dev                              # compile
pio run -e esp32dev -t upload                    # flash
pio device monitor                               # 38400 baud, set in platformio.ini
```

`include/secrets.h` holds the deployment-specific `SERVER_HOST` and is gitignored; the build fails without it rather than defaulting to a placeholder endpoint. The Traccar UDP protocol identifies a device by its ID alone, so host plus device ID is enough for anyone to inject positions. Keep both private.

The Freematics SDK lives under [`libraries/`](libraries). It is upstream code with local changes confined to the GNSS path in `FreematicsPlus/` (external-module detection and the hardened NMEA parser); everything else is as shipped.

Release history is in [`CHANGELOG.md`](CHANGELOG.md); the firmware version and build timestamp are printed at boot. [`tools/traccar_baseline.py`](tools/traccar_baseline.py) queries Traccar's Postgres to measure fix availability, gaps and outages for a device, and is how the field checks in `SPDD.md` §16 were run.

## Credits

Based on the [Freematics Telelogger](https://github.com/stanleyhuangyc/Freematics) by **Stanley Huang** `<stanley@freematics.com.au>`, distributed under the BSD license. All original copyright notices are retained in the derived source files. Thanks to Stanley and the Freematics project for the foundation this builds on.
