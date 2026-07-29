/*
  Baldur DID1 -> Arduino Mega 2560 CAN dashboard
  ------------------------------------------------
  Reads live engine data from a Baldur DID1 diesel ECU over CAN bus 1
  using ISO 15765-4 (11-bit) OBD-II PID requests. This is built directly
  from section 4.2 ("OBD2 communications") of the DID1 reference manual,
  quoted by the user from their own copy -- not inferred from this
  repo's (unrelated) Speeduino firmware and not scraped from a search
  summary. Per the manual, OBD2 communication is on CAN bus 1 and
  requires these ECU-side settings to be enabled:
    CAN bus data mode     = 500kbit
    CAN receiving enable  = Enabled
    OBD2 service enable   = Enabled

  This does NOT modify the ECU. It is a separate, read-only CAN client
  that polls PIDs and draws them on an SPI TFT.

  Hardware
  --------
  - Arduino Mega 2560 (or Elegoo Mega 2560 clone)
  - MCP2515 CAN controller module (with TJA1050/SN65HVD230 transceiver),
    wired to the Mega's hardware SPI bus (MOSI 51 / MISO 50 / SCK 52)
  - An SPI TFT display. Written against Adafruit_ILI9341, which covers the
    common cheap 2.4"/2.8"/3.2" SPI TFT boards (ILI9341/ILI9488-family
    controllers). If your board uses a different driver chip (ST7735,
    ST7789, etc.), swap the two ADAFRUIT_* includes/object below for the
    matching Adafruit driver library -- the drawing code is unchanged
    because it only uses Adafruit_GFX primitives. (This sketch was
    originally requested against a specific Alibaba display listing that
    could not be loaded -- 403 on every fetch attempt -- so the exact
    driver chip on your board is still unconfirmed; verify it yourself.)

  Wiring (change the #defines below to match your actual wiring)
  -----------------------------------------------------------------------
    MCP2515 module:
      VCC -> 5V   GND -> GND
      SCK -> 52   SI(MOSI) -> 51   SO(MISO) -> 50
      CS  -> pin 9   INT -> pin 2 (not used by this sketch, wired for
             future use / some breakout boards require it pulled up)
      CAN_H / CAN_L -> the DID1's CAN bus 1. Per the manual, on the DID1's
      OBD2 connector this is pin 6 (CAN-H) and pin 14 (CAN-L); pins 4 and
      5 are ground and pin 16 is +12V (ideally fused straight to
      battery, per the OBD2 standard, though a switched 12V feed usually
      works too). A 120 ohm termination resistor across CAN-H/CAN-L may
      be needed if the bus doesn't already have one -- most MCP2515
      breakout boards have a solder jumper for this.

    TFT module (SPI mode):
      VCC -> 5V (or 3.3V -- check your board)   GND -> GND
      SCK -> 52   MOSI(SDA) -> 51   (TFT MISO is optional, tie to 50 if present)
      CS  -> pin 10   DC/RS -> pin 8   RESET -> pin 7   LED/BL -> 5V (or a PWM pin)

  Required libraries (Arduino Library Manager)
  -----------------------------------------------------------------------
    - "CAN_BUS_Shield" by coryjfowler   (provides <mcp_can.h>)
    - "Adafruit GFX Library"
    - "Adafruit ILI9341"

  PID formulas -- what's confirmed vs. assumed
  -----------------------------------------------------------------------
  The manual lists which PIDs the DID1 reports, but only gives explicit
  value ranges for two of them: MAP (0x0B, "0 - 2550mbar") and vehicle
  speed (0x0D, "0 - 255 km/h"). Both match the standard SAE J1979 formula
  for those PID numbers exactly (raw byte A directly, in kPa for MAP,
  i.e. A*10 = mbar; A = km/h for speed), which is why every PID below
  uses the standard J1979 decode formula for its PID number. Confidence
  varies by PID:
    - HIGH confidence (PID number's standard meaning matches the DID1's
      stated meaning exactly): coolant temp (0x05), MAP (0x0B), RPM
      (0x0C), speed (0x0D), charge air/IAT (0x0F), fuel rail pressure
      (0x23, the real J1979 "fuel rail gauge pressure" PID -- this is
      the diesel rail pressure reading), barometer (0x33), supply
      voltage (0x42), oil temperature (0x5C).
    - LOWER confidence (DID1 repurposes a standard PID number to a
      diesel-specific meaning the J1979 table doesn't define, so the
      *byte encoding* is assumed to follow the standard PID's formula
      but that reuse isn't stated in the manual): accelerator pedal
      position (0x11, standard PID 0x11 is throttle position, formula
      A*100/255) and main injection angle cylinder 1 (0x0E, standard PID
      0x0E is timing advance, formula A/2-64 degrees). Sanity-check these
      two against the DID1's own tuning software before trusting them.
  Values NOT decoded here despite being in the manual's PID list: lambda
  sensors (0x24/0x25, 4-byte payload) and exhaust gas temps (0x78/0x79,
  which need multi-frame ISO-TP reassembly since the payload is too long
  for a single CAN frame) -- both are straightforward to add following
  the same pattern if you want them, just left out to keep this sketch
  focused.
*/

#include <SPI.h>
#include <mcp_can.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// ---------------------------------------------------------------------
// Pin configuration -- adjust to match your wiring
// ---------------------------------------------------------------------
#define CAN_CS_PIN   9
#define CAN_INT_PIN  2

#define TFT_CS       10
#define TFT_DC       8
#define TFT_RST      7

// ---------------------------------------------------------------------
// CAN configuration
// ---------------------------------------------------------------------
#define CAN_BAUD     CAN_500KBPS   // must match the ECU's configured CAN speed
#define CAN_CRYSTAL  MCP_16MHZ     // most MCP2515 boards use a 16MHz crystal;
                                   // change to MCP_8MHZ if yours is 8MHz

#define OBD_REQUEST_ID   0x7DFUL   // standard OBD2 functional broadcast request
#define OBD_RESPONSE_ID  0x7E8UL   // standard single-ECU OBD2 response ID

// ---------------------------------------------------------------------
// OBD-II PIDs the DID1 manual documents as reported (mode 0x01)
// ---------------------------------------------------------------------
enum : uint8_t {
  PID_COOLANT        = 0x05,
  PID_MAP             = 0x0B,
  PID_RPM             = 0x0C,
  PID_SPEED           = 0x0D,
  PID_INJ_ANGLE       = 0x0E, // "Main injection angle cylinder 1" -- see header note
  PID_CHARGE_AIR_TEMP = 0x0F,
  PID_PEDAL_POSITION  = 0x11, // "Effective accelerator pedal position" -- see header note
  PID_RAIL_PRESSURE   = 0x23, // "Fuel rail pressure"
  PID_BARO            = 0x33,
  PID_SUPPLY_VOLTAGE  = 0x42,
  PID_OIL_TEMP        = 0x5C,
};

const uint8_t stdPidList[] = {
  PID_COOLANT, PID_MAP, PID_RPM, PID_SPEED, PID_INJ_ANGLE,
  PID_CHARGE_AIR_TEMP, PID_PEDAL_POSITION, PID_RAIL_PRESSURE,
  PID_BARO, PID_SUPPLY_VOLTAGE, PID_OIL_TEMP
};
const uint8_t stdPidCount = sizeof(stdPidList) / sizeof(stdPidList[0]);
uint8_t pidCursor = 0;

MCP_CAN CAN(CAN_CS_PIN);
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

// ---------------------------------------------------------------------
// Latest decoded engine data
// ---------------------------------------------------------------------
struct EngineData {
  int16_t  rpm            = 0;
  int16_t  coolantC       = 0;
  uint16_t mapMbar        = 0;
  int16_t  chargeAirC     = 0;
  uint8_t  pedalPct       = 0;
  float    injAngleDeg    = 0.0f;
  uint8_t  speedKmh       = 0;
  uint8_t  baroKPa        = 0;
  float    supplyV        = 0.0f;
  uint32_t railPressureKPa = 0;
  int16_t  oilTempC       = 0;
  unsigned long lastRxMillis = 0;
} engine;

const unsigned long POLL_INTERVAL_MS = 40;   // ~25 requests/sec round robin
const unsigned long STALE_TIMEOUT_MS = 2000; // show "--" if no CAN reply in time
unsigned long lastPollMillis = 0;

// ---------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);

  pinMode(CAN_INT_PIN, INPUT_PULLUP);

  while (CAN.begin(MCP_ANY, CAN_BAUD, CAN_CRYSTAL) != CAN_OK)
  {
    Serial.println(F("CAN init failed, retrying..."));
    delay(500);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println(F("CAN init OK"));

  tft.begin();
  tft.setRotation(1); // landscape; change 0-3 to suit your mounting
  tft.fillScreen(ILI9341_BLACK);
  drawStaticLayout();
}

// ---------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------
void loop()
{
  pollNextPid();
  readCanResponses();
  updateDisplay();
}

// ---------------------------------------------------------------------
// Send the next OBD-II PID request in round-robin order
// ---------------------------------------------------------------------
void pollNextPid()
{
  unsigned long now = millis();
  if (now - lastPollMillis < POLL_INTERVAL_MS) { return; }
  lastPollMillis = now;

  uint8_t pid = stdPidList[pidCursor];
  pidCursor = (pidCursor + 1) % stdPidCount;

  uint8_t data[8] = { 0x02, 0x01, pid, 0x00, 0x00, 0x00, 0x00, 0x00 };
  CAN.sendMsgBuf(OBD_REQUEST_ID, 0, 8, data);
}

// ---------------------------------------------------------------------
// Read and decode any pending CAN responses
// ---------------------------------------------------------------------
void readCanResponses()
{
  while (CAN.checkReceive() == CAN_MSGAVAIL)
  {
    long unsigned int rxId;
    uint8_t len;
    uint8_t buf[8];
    CAN.readMsgBuf(&rxId, &len, buf);

    if (rxId != OBD_RESPONSE_ID) { continue; }
    if (len < 3) { continue; }

    engine.lastRxMillis = millis();

    if (buf[1] == 0x41) // standard OBD-II response (mode 0x01 + 0x40)
    {
      decodeStandardPid(buf[2], buf);
    }
  }
}

void decodeStandardPid(uint8_t pid, uint8_t *buf)
{
  uint8_t A = buf[3];
  uint8_t B = buf[4];

  switch (pid)
  {
    case PID_COOLANT:        engine.coolantC    = (int16_t)A - 40; break;
    case PID_MAP:             engine.mapMbar     = (uint16_t)A * 10; break; // manual: 0-2550mbar
    case PID_RPM:             engine.rpm         = (((uint16_t)A << 8) | B) / 4; break;
    case PID_SPEED:           engine.speedKmh    = A; break; // manual: 0-255 km/h
    case PID_INJ_ANGLE:       engine.injAngleDeg = ((float)A / 2.0f) - 64.0f; break; // assumed, see header note
    case PID_CHARGE_AIR_TEMP: engine.chargeAirC  = (int16_t)A - 40; break;
    case PID_PEDAL_POSITION:  engine.pedalPct    = ((uint16_t)A * 100) / 255; break; // assumed, see header note
    case PID_RAIL_PRESSURE:   engine.railPressureKPa = 10UL * (((uint16_t)A << 8) | B); break;
    case PID_BARO:            engine.baroKPa     = A; break;
    case PID_SUPPLY_VOLTAGE:  engine.supplyV     = (((uint16_t)A << 8) | B) / 1000.0f; break;
    case PID_OIL_TEMP:        engine.oilTempC    = (int16_t)A - 40; break;
    default: break;
  }
}

// ---------------------------------------------------------------------
// Display: static labels drawn once, values redrawn only when changed
// (avoids full-screen redraw flicker)
// ---------------------------------------------------------------------
#define COLOR_BG     ILI9341_BLACK
#define COLOR_LABEL  ILI9341_DARKGREY
#define COLOR_VALUE  ILI9341_WHITE
#define COLOR_WARN   ILI9341_RED
#define COLOR_GOOD   ILI9341_GREEN

// 3-column grid below the RPM header row. Columns at x = 6 / 112 / 218,
// each ~100px wide on a 320px-wide landscape screen.
#define COL0_X 6
#define COL1_X 112
#define COL2_X 218

void drawStaticLayout()
{
  tft.setTextColor(COLOR_LABEL);
  tft.setTextSize(2);

  tft.setCursor(COL0_X, 4);
  tft.print(F("RPM"));

  tft.setCursor(COL0_X, 54);
  tft.print(F("COOLANT"));
  tft.setCursor(COL1_X, 54);
  tft.print(F("BOOST/MAP"));
  tft.setCursor(COL2_X, 54);
  tft.print(F("RAIL PRES"));

  tft.setCursor(COL0_X, 108);
  tft.print(F("CHG AIR"));
  tft.setCursor(COL1_X, 108);
  tft.print(F("BARO"));
  tft.setCursor(COL2_X, 108);
  tft.print(F("SPEED"));

  tft.setCursor(COL0_X, 154);
  tft.print(F("SUPPLY V"));
  tft.setCursor(COL1_X, 154);
  tft.print(F("PEDAL"));
  tft.setCursor(COL2_X, 154);
  tft.print(F("OIL TEMP"));

  tft.setCursor(COL0_X, 200);
  tft.print(F("INJ ANGLE"));
}

// Small helper: clear a value field then print new text
void printField(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color,
                 uint8_t textSize, const String &text)
{
  tft.fillRect(x, y, w, h, COLOR_BG);
  tft.setTextColor(color);
  tft.setTextSize(textSize);
  tft.setCursor(x, y);
  tft.print(text);
}

void updateDisplay()
{
  static unsigned long lastDrawMillis = 0;
  unsigned long now = millis();
  if (now - lastDrawMillis < 100) { return; } // ~10 fps is plenty for gauges
  lastDrawMillis = now;

  bool stale = (now - engine.lastRxMillis) > STALE_TIMEOUT_MS;

  if (stale)
  {
    printField(COL0_X, 25, 300, 24, COLOR_WARN, 3, F("NO CAN"));
    return;
  }

  // RPM, big numbers, full width
  printField(COL0_X, 25, 300, 24, COLOR_VALUE, 3, String(engine.rpm));

  // Coolant / boost / rail pressure row
  uint16_t coolantColor = (engine.coolantC >= 100) ? COLOR_WARN : COLOR_VALUE;
  printField(COL0_X, 74, 100, 24, coolantColor, 2, String(engine.coolantC) + "C");

  int32_t boostMbar = (int32_t)engine.mapMbar - ((int32_t)engine.baroKPa * 10);
  printField(COL1_X, 74, 100, 24, COLOR_VALUE, 2,
             String(engine.mapMbar / 1000.0f, 2) + "bar");
  printField(COL1_X, 92, 100, 14, COLOR_LABEL, 1,
             (boostMbar >= 0 ? "+" : "") + String(boostMbar / 1000.0f, 2) + "bar boost");

  printField(COL2_X, 74, 100, 24, COLOR_VALUE, 2,
             String(engine.railPressureKPa / 100.0f, 0) + "bar");

  // Charge air temp / barometer / speed row
  printField(COL0_X, 128, 100, 20, COLOR_VALUE, 2, String(engine.chargeAirC) + "C");
  printField(COL1_X, 128, 100, 20, COLOR_VALUE, 2, String(engine.baroKPa) + "kPa");
  printField(COL2_X, 128, 100, 20, COLOR_VALUE, 2, String(engine.speedKmh) + "km/h");

  // Supply voltage / pedal position / oil temp row
  uint16_t suppColor = (engine.supplyV < 11.5f || engine.supplyV > 15.0f)
                          ? COLOR_WARN : COLOR_GOOD;
  printField(COL0_X, 174, 100, 20, suppColor, 2, String(engine.supplyV, 1) + "V");
  printField(COL1_X, 174, 100, 20, COLOR_VALUE, 2, String(engine.pedalPct) + "%");
  uint16_t oilColor = (engine.oilTempC >= 130) ? COLOR_WARN : COLOR_VALUE;
  printField(COL2_X, 174, 100, 20, oilColor, 2, String(engine.oilTempC) + "C");

  // Injection angle
  printField(COL0_X, 220, 150, 18, COLOR_VALUE, 2, String(engine.injAngleDeg, 1) + "d");
}
