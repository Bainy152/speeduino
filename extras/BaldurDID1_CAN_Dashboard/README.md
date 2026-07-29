# Baldur DID1 CAN Dashboard for Arduino Mega 2560

An Arduino sketch that drives an SPI TFT display as a standalone gauge
cluster for a **Baldur DID1** diesel ECU, reading live data over CAN bus.
It does not modify or interfere with the ECU — it's a passive CAN client
that can sit alongside TunerStudio on the same bus.

## Why CAN, and why OBD-II PIDs

This does **not** assume anything about Speeduino, despite living in this
repo — the DID1 is not Speeduino-derived. The only DID1-specific claim
this sketch relies on is that the DID1's reference manual (published by
Baldur's Control Systems at `controls.is/manuals/did1.pdf`) documents it
as implementing **ISO 15765-4 (11-bit) OBD-over-CAN**, specifically so it
can be read by generic OBD2 scan tools, phone apps, and aftermarket
gauges. That's a real, standardised protocol (SAE J1979 / ISO 15765-4),
not something specific to this codebase, and it's what this sketch polls.

**Caveat on sourcing:** the build environment this was written in could
not reach `controls.is` (blocked by network policy), so the manual was
never read directly — the ISO 15765-4 claim above and the OBD2 connector
pinout mentioned in the wiring section come from a web-search summary of
the manual, not a first-hand read. Before wiring anything, pull up
`https://controls.is/manuals/did1.pdf` yourself and check the CAN/OBD2
section against what's written here and in the sketch's header comment.

## Hardware you need

- Arduino Mega 2560 (or Elegoo Mega 2560 clone)
- An MCP2515-based CAN module (MCP2515 + TJA1050/SN65HVD230 transceiver),
  wired to the Mega's hardware SPI pins (50/51/52)
- An SPI TFT display (e.g. a 2.4"/2.8"/3.2" ILI9341-based module — a
  common type sold on Alibaba/AliExpress as a "SPI TFT LCD module")

**Before wiring anything, also confirm your display's driver chip.**
The listing this was originally requested from (an Alibaba product page)
couldn't be loaded — it returned a 403 both through the fetch tool and a
direct request, and the page description alone wasn't enough to identify
the exact hardware. The sketch is written for `Adafruit_ILI9341`, the
most common chip on cheap SPI TFT boards. If your board's silkscreen or
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

Per the search summary of the DID1 manual, its OBD2 connector uses pin 6
for CAN-H and pin 14 for CAN-L, with pins 4/5 as ground — verify this
against the manual/your harness before connecting, per the caveat above.

The CAN bus needs 120 ohm termination resistors at **both** physical
ends. Most MCP2515 breakout boards have a solder-jumper resistor — only
enable it if the ECU end of the bus isn't already terminated.

## Libraries (install via Arduino Library Manager)

- `CAN_BUS_Shield` by coryjfowler (provides `mcp_can.h`)
- `Adafruit GFX Library`
- `Adafruit ILI9341` (or the matching driver for your display chip)

## CAN bus speed

ISO 15765-4's 11-bit variant is standardised at 500 kbit/s, which is what
the sketch defaults to (`CAN_BAUD`). The DID1 reportedly also has a
second, separately configurable CAN interface for arbitrary data up to
1 Mbps — that's not what this sketch talks to; it only uses the standard
OBD2 service, so 500 kbit/s should be correct.

## What's shown, and what's actually verified

RPM (PID `0x0C`) and coolant temperature (PID `0x05`) are close to
universal on any OBD2-compliant ECU, diesel included, so these should
work as-is.

The rest of the polled PIDs — MAP (`0x0B`), throttle position (`0x11`),
barometric pressure (`0x33`), battery voltage (`0x42`), timing advance
(`0x0E`), and road speed (`0x0D`) — are standard PIDs, but several of
them (MAP, TPS) are gasoline-oriented and it's **not verified here**
whether the DID1 populates them with meaningful values for a diesel
engine (e.g. MAP as boost pressure, no throttle plate for TPS). Check
each gauge against a known-good OBD2 scan tool/app connected to the same
ECU before trusting it.

Diesel-specific values like rail pressure are **not** standard PIDs and
are not implemented in this sketch at all — earlier drafts of this code
guessed at a manufacturer-specific PID for them based on the unrelated
Speeduino codebase in this repo, which was a mistake since the DID1 isn't
Speeduino-based; that guess has been removed. If the DID1 manual
documents a manufacturer-specific PID (mode `0x22`) for such values, that
would need to be added from the actual manual, not inferred.

## Tuning the layout

`drawStaticLayout()` and `updateDisplay()` use plain `Adafruit_GFX` calls
(`fillRect`, `setCursor`, `print`, ...), so rearranging gauges, adding bar
graphs, or changing colors/thresholds is just normal Arduino graphics
code — no protocol knowledge needed beyond what's already decoded into
the `EngineData` struct.
