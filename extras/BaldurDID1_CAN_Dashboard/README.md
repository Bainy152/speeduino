# Baldur DID1 CAN Dashboard for Arduino Mega 2560

An Arduino sketch that drives an SPI TFT display as a standalone gauge
cluster for a **Baldur DID1** diesel ECU, reading live data over CAN bus.
It does not modify or interfere with the ECU — it's a passive CAN client
that can sit alongside BG Calibrator (the DID1's own tuning software).

This is built from section 4.2 ("OBD2 communications") of the actual
DID1 reference manual, quoted directly from a user's own copy — not
inferred from this repo's (unrelated) Speeduino firmware, and not
scraped from a search-engine summary. Earlier drafts of this sketch made
exactly that mistake (assuming DID1 shared Speeduino's CAN internals);
that's been removed.

## The protocol

Per the manual, the DID1 supports **ISO 15765-4 (11-bit) OBD-over-CAN on
CAN bus 1**, meant to be read by generic OBD2 scan tools, phone apps, and
aftermarket gauges — which is exactly what this sketch does: it polls
standard PID requests (`0x7DF` broadcast) and decodes the `0x7E8`
responses.

Three settings need to be enabled on the ECU itself for this to work
(exact parameter names from the manual):

- `CAN bus data mode` = `500kbit`
- `CAN receiving enable` = `Enabled`
- `OBD2 service enable` = `Enabled`

## Hardware you need

- Arduino Mega 2560 (or Elegoo Mega 2560 clone)
- An MCP2515-based CAN module (MCP2515 + TJA1050/SN65HVD230 transceiver),
  wired to the Mega's hardware SPI pins (50/51/52)
- An SPI TFT display (e.g. a 2.4"/2.8"/3.2" ILI9341-based module — a
  common type sold on Alibaba/AliExpress as a "SPI TFT LCD module")

**Confirm your display's driver chip before wiring.** The listing this
was originally requested from (an Alibaba product page) couldn't be
loaded — it returned a 403 both through the fetch tool and a direct
request — so the exact chip on your board is still unconfirmed. The
sketch is written for `Adafruit_ILI9341`, the most common chip on cheap
SPI TFT boards; if yours is ST7735, ST7789, etc., swap the
`Adafruit_ILI9341` include/object for the matching Adafruit driver
library. The drawing code is plain `Adafruit_GFX` and doesn't change.

## Wiring

Full pin-by-pin wiring is in the header comment of
`BaldurDID1_CAN_Dashboard.ino`. Both the CAN module and the TFT share the
Mega's hardware SPI bus (pins 50/51/52) with their own CS pin (9 for CAN,
10 for TFT by default).

Per the manual, the DID1's OBD2 connector pinout is:

| Pin | Signal |
|-----|--------|
| 6   | CAN-H  |
| 14  | CAN-L  |
| 4, 5 | Ground |
| 16  | +12V (ideally fused straight to battery) |

A 120 ohm termination resistor across CAN-H/CAN-L may be needed if the
bus doesn't already have one — most MCP2515 breakout boards have a
solder jumper for this.

## Libraries (install via Arduino Library Manager)

- `CAN_BUS_Shield` by coryjfowler (provides `mcp_can.h`)
- `Adafruit GFX Library`
- `Adafruit ILI9341` (or the matching driver for your display chip)

## What's shown, and confidence level per gauge

The manual's PID table gives explicit value ranges for only two PIDs —
MAP (`0x0B`, "0 - 2550mbar") and vehicle speed (`0x0D`, "0 - 255 km/h") —
and both match the standard SAE J1979 decode formula for those PID
numbers exactly. That's the basis for using the standard J1979 formula
for every PID below.

**High confidence** (the DID1's stated meaning for the PID number matches
its standard J1979 meaning exactly): coolant temp (`0x05`), MAP/boost
(`0x0B`), RPM (`0x0C`), speed (`0x0D`), charge air temp (`0x0F`), **fuel
rail pressure (`0x23`** — this is the genuine J1979 "fuel rail gauge
pressure" PID, i.e. the diesel common-rail pressure reading you likely
care about most), barometer (`0x33`), supply voltage (`0x42`), oil
temperature (`0x5C`).

**Lower confidence** (DID1 repurposes a standard PID number for a
diesel-specific value the J1979 table doesn't define, so only the *byte
encoding* is assumed to carry over — not stated in the manual): pedal
position (`0x11`, standard meaning is throttle position) and main
injection angle cylinder 1 (`0x0E`, standard meaning is timing advance).
Cross-check these two gauges against the live values in BG Calibrator
before trusting the numbers.

**Documented but not implemented**, to keep the sketch focused: lambda
sensors 1/2 (`0x24`/`0x25`, 4-byte payload — straightforward to add) and
exhaust gas temperature sensors 1-8 (`0x78`/`0x79` — these need
multi-frame ISO-TP reassembly since the payload doesn't fit in one CAN
frame, more work than a single `case` in `decodeStandardPid()`).

## Tuning the layout

`drawStaticLayout()` and `updateDisplay()` use plain `Adafruit_GFX` calls
(`fillRect`, `setCursor`, `print`, ...), so rearranging gauges, adding bar
graphs, or changing colors/thresholds is just normal Arduino graphics
code — no protocol knowledge needed beyond what's already decoded into
the `EngineData` struct.
