# VorsprungLogger

An Audi-focused vehicle telemetry logger for the [Freematics ONE+ Model B](https://freematics.com/products/freematics-one-plus-model-b/) (ESP32). It collects OBD-II live data plus a large set of manufacturer-specific UDS signals from the VAG **EA897 evo** 3.0 TDI powertrain (Audi A6 Allroad), logs to a microSD card, and streams to a [Traccar](https://www.traccar.org) server over a cellular link.

VorsprungLogger is derived from the Freematics **Telelogger** by Stanley Huang and keeps his original BSD copyright (see [Credits](#credits)). It trims the upstream's broad feature set down to one specific vehicle and use case.

## What it collects

* **OBD-II live PIDs** — speed, RPM, throttle, engine load, and more, read every cycle.
* **UDS DIDs (~50 signals across two ECUs)** — engine (`0x7E0`) and SCR/reductant (`0x7EA`), decoded into engineering units and emitted as synthetic PIDs in the `0x200+` range. See [`include/uds_dids.h`](include/uds_dids.h).
* **Derived-inputs group** — RPM, MAF, charge pressure, IAT, fuel rate and engine torque are co-read atomically so they always share a packet, making downstream volumetric-efficiency / power / AFR / BSFC math plain arithmetic. See [`SPDD.md`](SPDD.md) §8.
* **GNSS** — position, speed, heading from the built-in u-blox M10.
* **MEMS** — accelerometer / gyroscope motion, used for standby/wake and logging.
* **Housekeeping** — battery voltage, cellular signal level, device temperature.

## Scheduling

UDS DIDs are polled by a tiered countdown scheduler (`UDS_FAST` / `UDS_SLOW` / `UDS_GLACIAL`) with a per-cycle read budget, plus the atomic `UDS_GROUP` tier for the derived-inputs group. See the scheduler notes in [`src/main.cpp`](src/main.cpp) (`processUDS`) and the full design in [`SPDD.md`](SPDD.md) §7.

## Storage & transmission

* Telemetry over **cellular UDP** to a Traccar endpoint, framed as `devid#PID:value,...*checksum`.
* On a cell outage, unsent packets spool to microSD (`/SPOOL/<seq>.PKT`, one file per outage) and drain back to the server on reconnect, so a dropout leaves no permanent gap. See [`SPDD.md`](SPDD.md) §14.
* Every record carries its own capture time, derived from a monotonic tick plus a GNSS/cellular UTC anchor — so replayed records are filed at the moment they were captured, not the moment they arrived. See [`TIME.md`](TIME.md) and [`SPDD.md`](SPDD.md) §16.3.

## Hardware

* Freematics ONE+ **Model B** (ESP32-D0WDQ6, no PSRAM).
* MicroSD card.
* A cellular SIM (configured via `CELL_APN` in [`include/config.h`](include/config.h)).

## Building

Built with [PlatformIO](https://platformio.org/):

```sh
pio run -e esp32dev            # compile
pio run -e esp32dev -t upload  # flash
```

The Freematics SDK lives under [`libraries/`](libraries) and is consumed as-is.

Release history is in [`CHANGELOG.md`](CHANGELOG.md); the firmware version and build timestamp are printed at boot.

## Credits

Based on the [Freematics Telelogger](https://github.com/stanleyhuangyc/Freematics) by **Stanley Huang** `<stanley@freematics.com.au>`, distributed under the BSD license. All original copyright notices are retained in the derived source files. Thanks to Stanley and the Freematics project for the foundation this builds on.
