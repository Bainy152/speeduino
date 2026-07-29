# Baldur DID1 CAN Dashboard for Arduino Mega 2560

An Arduino sketch that drives an SPI TFT display as a standalone gauge
cluster for a **Baldur DID1** ECU (a Speeduino-derived diesel injection
controller), reading live data over CAN bus. It does not modify or
interfere with the ECU firmware — it's a passive CAN client sitting
alongside TunerStudio on the same bus.

## Why CAN, and why OBD-II PIDs

Baldur DID1 inherits Speeduino's onboard-CAN handling
(`speeduino/cancomms.ino`). When the ECU's internal CAN module is enabled
in TunerStudio, it answers standard OBD-II PID requests sent to broadcast
ID `0x7DF` with responses on `0x7E8` — the same mechanism generic OBD-II
scan tools and aftermarket dashes use. That's what this sketch polls, so
it works without needing any ECU-side per-channel broadcast configuration.

## Hardware you need

- Arduino Mega 2560 (or Elegoo Mega 2560 clone)
- An MCP2515-based CAN module (MCP2515 + TJA1050/SN65HVD230 transceiver),
  wired to the Mega's hardware SPI pins (50/51/52)
- An SPI TFT display (e.g. a 2.4"/2.8"/3.2" ILI9341-based module — the
  common type sold on Alibaba/AliExpress as a "SPI TFT LCD module")

**Before wiring anything, confirm your display's driver chip.** The
sketch is written for `Adafruit_ILI9341`. If your board's silkscreen or
listing says ST7735, ST7789, or something else, swap the
`Adafruit_ILI9341` include/object for the matching Adafruit driver
library — everything else in the sketch (the drawing code) is written
against `Adafruit_GFX` and doesn't need to change.

## Wiring

See the comment block at the top of `BaldurDID1_CAN_Dashboard.ino` for
full pin-by-pin wiring. In short: both the CAN module and the TFT share
the Mega's hardware SPI bus (pins 50/51/52) and each gets its own CS pin
(9 for CAN, 10 for TFT by default — change the `#define`s if you wire it
differently).

The CAN bus needs 120 ohm termination resistors at **both** physical
ends. Most MCP2515 breakout boards have a solder-jumper resistor — only
enable it if the ECU end of the bus isn't already terminated.

## Libraries (install via Arduino Library Manager)

- `CAN_BUS_Shield` by coryjfowler (provides `mcp_can.h`)
- `Adafruit GFX Library`
- `Adafruit ILI9341` (or the matching driver for your display chip)

## ECU-side setup (TunerStudio, Baldur DID1 project)

1. Open the CAN Configuration page and enable the internal CAN module.
2. Confirm the configured CAN bus speed. Speeduino/Baldur defaults to
   500 kbps; if you've changed it, update `CAN_BAUD` in the sketch to match.
3. No further per-channel broadcast setup is needed — this sketch talks
   to the built-in OBD-II PID emulation directly.

## What's shown

RPM, coolant temperature, MAP (absolute + boost relative to barometric
pressure), throttle position, intake air temperature, barometric
pressure, road speed, battery voltage, and ignition timing advance — all
from standard, version-stable OBD-II PIDs.

## Custom (non-standard) channels — read this before enabling

Baldur DID1 also exposes every other internal live variable (rail
pressure, oil pressure, etc.) through a custom PID (mode `0x22`, PID high
byte `0x78`) that indexes directly into the firmware's internal status
array — the same array `sendcanValues()` in `cancomms.ino` serializes for
TunerStudio. **That array's layout is firmware-specific and can change
between Baldur versions**, so the offsets in the sketch
(`CUSTOM_PID_RAIL_PRESSURE`, `CUSTOM_PID_OIL_PRESSURE`) are placeholders
copied from mainline Speeduino's layout. Before enabling
`ENABLE_CUSTOM_PIDS`:

1. Check the actual field order in your Baldur DID1 firmware's status
   struct / `sendcanValues()` equivalent to get the right byte offset.
2. Or, empirically confirm it: request an offset, and check the decoded
   value against the same channel's live value in TunerStudio.

## Tuning the layout

`drawStaticLayout()` and `updateDisplay()` use plain `Adafruit_GFX`
calls (`fillRect`, `setCursor`, `print`, ...), so rearranging gauges,
adding bar graphs, or changing colors/thresholds is just normal Arduino
graphics code — no protocol knowledge needed beyond what's already
decoded into the `EngineData` struct.
