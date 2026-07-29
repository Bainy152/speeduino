/*
  Baldur DID1 -> Arduino Mega 2560 CAN dashboard
  ------------------------------------------------
  Reads live engine data from a Baldur DID1 ECU (a Speeduino-derived diesel
  injection controller) over its onboard CAN bus, using the standard OBD-II
  PID emulation that the Speeduino/Baldur firmware family implements for
  external dashes and loggers (see speeduino/cancomms.ino: can_Command() /
  obd_response(), which answers broadcast ID 0x7DF with responses on 0x7E8).

  This does NOT modify the ECU firmware. It is a separate, read-only CAN
  client that polls standard PIDs and draws them on an SPI TFT.

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
    because it only uses Adafruit_GFX primitives.

  Wiring (change the #defines below to match your actual wiring)
  -----------------------------------------------------------------------
    MCP2515 module:
      VCC -> 5V   GND -> GND
      SCK -> 52   SI(MOSI) -> 51   SO(MISO) -> 50
      CS  -> pin 9   INT -> pin 2 (not used by this sketch, wired for
             future use / some breakout boards require it pulled up)
      CAN_H / CAN_L -> ECU's CAN bus. The bus needs 120 ohm termination at
      *each* end. Most MCP2515 breakout boards have a solder jumper to add
      one; only enable it if the ECU end doesn't already terminate the bus.

    TFT module (SPI mode):
      VCC -> 5V (or 3.3V -- check your board)   GND -> GND
      SCK -> 52   MOSI(SDA) -> 51   (TFT MISO is optional, tie to 50 if present)
      CS  -> pin 10   DC/RS -> pin 8   RESET -> pin 7   LED/BL -> 5V (or a PWM pin)

  Required libraries (Arduino Library Manager)
  -----------------------------------------------------------------------
    - "CAN_BUS_Shield" by coryjfowler   (provides <mcp_can.h>)
    - "Adafruit GFX Library"
    - "Adafruit ILI9341"

  ECU-side configuration (in TunerStudio, on the Baldur DID1 project)
  -----------------------------------------------------------------------
    - CAN Configuration page: enable the internal/onboard CAN module,
      confirm the CAN speed (500000 kbps is the Speeduino/Baldur default --
      change CAN_BAUD below to match if you've changed it on the ECU).
    - This sketch only needs the ECU to answer standard OBD-II PID
      requests -- no per-channel CAN broadcast setup is required, since it
      polls the built-in OBD emulation directly.

  IMPORTANT caveat about "custom" channels
  -----------------------------------------------------------------------
  Standard OBD-II PIDs (RPM, coolant, MAP, IAT, TPS, battery voltage,
  barometric pressure, timing advance, speed) are answered from fixed,
  documented formulas and are safe to rely on across Speeduino/Baldur
  firmware versions.

  Baldur DID1 also exposes *every* other live variable (e.g. rail
  pressure, injector duty, etc.) via a custom PID (mode 0x22, PID high
  byte 0x78) that indexes directly into the firmware's internal status
  array. That array's layout is firmware-specific and can change between
  Baldur versions/forks, so the offsets below (CUSTOM_PID_RAIL_PRESSURE
  etc.) are placeholders -- verify them against your actual Baldur DID1
  firmware source (or cross-check the decoded value against TunerStudio's
  live gauges) before trusting them. Custom PIDs are OFF by default; set
  ENABLE_CUSTOM_PIDS to 1 once you've confirmed the offsets.
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

#define OBD_REQUEST_ID   0x7DFUL   // broadcast functional request, matches
                                   // the check in cancomms.ino can_Command()
#define OBD_RESPONSE_ID  0x7E8UL   // fixed response ID used by obd_response()

// Enable to poll firmware-specific channels (see caveat above)
#define ENABLE_CUSTOM_PIDS 0
#define CUSTOM_PID_RAIL_PRESSURE 103  // placeholder offset -- verify!
#define CUSTOM_PID_OIL_PRESSURE  104  // placeholder offset -- verify!

// ---------------------------------------------------------------------
// Standard OBD-II PIDs we poll (mode 0x01)
// ---------------------------------------------------------------------
enum : uint8_t {
  PID_RPM       = 0x0C,
  PID_COOLANT   = 0x05,
  PID_MAP       = 0x0B,
  PID_IAT       = 0x0F,
  PID_TPS       = 0x11,
  PID_TIMING    = 0x0E,
  PID_SPEED     = 0x0D,
  PID_BARO      = 0x33,
  PID_BATTERY   = 0x42,
};

const uint8_t stdPidList[] = {
  PID_RPM, PID_COOLANT, PID_MAP, PID_IAT, PID_TPS,
  PID_TIMING, PID_SPEED, PID_BARO, PID_BATTERY
};
const uint8_t stdPidCount = sizeof(stdPidList) / sizeof(stdPidList[0]);
uint8_t pidCursor = 0;

MCP_CAN CAN(CAN_CS_PIN);
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

// ---------------------------------------------------------------------
// Latest decoded engine data
// ---------------------------------------------------------------------
struct EngineData {
  int16_t rpm         = 0;
  int16_t coolantC    = 0;
  int16_t mapKPa      = 0;
  int16_t iatC         = 0;
  uint8_t tpsPct       = 0;
  int8_t  timingAdv    = 0;
  uint8_t speedKmh     = 0;
  uint8_t baroKPa      = 0;
  float   batteryV     = 0.0f;
  int16_t railPressure = 0;
  int16_t oilPressure  = 0;
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

#if ENABLE_CUSTOM_PIDS
  // Interleave one custom-channel request every few standard requests
  static uint8_t customTurn = 0;
  if (customTurn == 0)
  {
    sendCustomPidRequest(CUSTOM_PID_RAIL_PRESSURE);
    customTurn = 5;
    return;
  }
  customTurn--;
#endif

  uint8_t pid = stdPidList[pidCursor];
  pidCursor = (pidCursor + 1) % stdPidCount;

  uint8_t data[8] = { 0x02, 0x01, pid, 0x00, 0x00, 0x00, 0x00, 0x00 };
  CAN.sendMsgBuf(OBD_REQUEST_ID, 0, 8, data);
}

#if ENABLE_CUSTOM_PIDS
void sendCustomPidRequest(uint8_t channelIndex)
{
  // mode 0x22, PID high byte 0x78, PID low byte = channel index
  uint8_t data[8] = { 0x03, 0x22, channelIndex, 0x78, 0x00, 0x00, 0x00, 0x00 };
  CAN.sendMsgBuf(OBD_REQUEST_ID, 0, 8, data);
}
#endif

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
    else if (buf[1] == 0x62) // custom mode response (mode 0x22 + 0x40)
    {
      decodeCustomPid(buf[2], buf[3], buf);
    }
  }
}

void decodeStandardPid(uint8_t pid, uint8_t *buf)
{
  uint8_t A = buf[3];
  uint8_t B = buf[4];

  switch (pid)
  {
    case PID_RPM:      engine.rpm      = (((uint16_t)A << 8) | B) / 4; break;
    case PID_COOLANT:  engine.coolantC = (int16_t)A - 40; break;
    case PID_MAP:      engine.mapKPa   = A; break;
    case PID_IAT:      engine.iatC     = (int16_t)A - 40; break;
    case PID_TPS:      engine.tpsPct   = ((uint16_t)A * 100) / 255; break;
    case PID_TIMING:   engine.timingAdv = ((int16_t)A / 2) - 64; break;
    case PID_SPEED:    engine.speedKmh = A; break;
    case PID_BARO:     engine.baroKPa  = A; break;
    case PID_BATTERY:  engine.batteryV = (((uint16_t)A << 8) | B) / 1000.0f; break;
    default: break;
  }
}

void decodeCustomPid(uint8_t channelIndex, uint8_t pidHigh, uint8_t *buf)
{
  if (pidHigh != 0x78) { return; }
  int16_t value = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);

  if (channelIndex == CUSTOM_PID_RAIL_PRESSURE) { engine.railPressure = value; }
  else if (channelIndex == CUSTOM_PID_OIL_PRESSURE) { engine.oilPressure = value; }
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

void drawStaticLayout()
{
  tft.setTextColor(COLOR_LABEL);

  tft.setTextSize(2);
  tft.setCursor(10, 4);
  tft.print(F("RPM"));

  tft.setCursor(200, 4);
  tft.print(F("COOLANT"));

  tft.setCursor(10, 90);
  tft.print(F("BOOST/MAP"));

  tft.setCursor(200, 90);
  tft.print(F("TPS"));

  tft.setCursor(10, 150);
  tft.print(F("IAT"));
  tft.setCursor(100, 150);
  tft.print(F("BARO"));
  tft.setCursor(200, 150);
  tft.print(F("SPD"));

  tft.setCursor(10, 190);
  tft.print(F("BATT"));
  tft.setCursor(100, 190);
  tft.print(F("ADV"));

#if ENABLE_CUSTOM_PIDS
  tft.setCursor(200, 190);
  tft.print(F("RAIL"));
#endif
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
    printField(10, 25, 180, 40, COLOR_WARN, 4, F("NO CAN"));
    return;
  }

  // RPM, big numbers
  printField(10, 25, 180, 40, COLOR_VALUE, 4, String(engine.rpm));

  // Coolant, colour-warns above 100C
  uint16_t coolantColor = (engine.coolantC >= 100) ? COLOR_WARN : COLOR_VALUE;
  printField(200, 25, 100, 30, coolantColor, 3, String(engine.coolantC) + "C");

  // Boost/MAP relative to barometric pressure, shown alongside absolute kPa
  int16_t boostKPa = engine.mapKPa - engine.baroKPa;
  printField(10, 112, 150, 30, COLOR_VALUE,
             3, String(engine.mapKPa) + "kPa");
  printField(10, 140, 150, 20, COLOR_LABEL,
             2, (boostKPa >= 0 ? "+" : "") + String(boostKPa) + "kPa boost");

  // TPS
  printField(200, 112, 100, 30, COLOR_VALUE, 3, String(engine.tpsPct) + "%");

  // IAT / Baro / Speed
  printField(10, 168, 80, 20, COLOR_VALUE, 2, String(engine.iatC) + "C");
  printField(100, 168, 80, 20, COLOR_VALUE, 2, String(engine.baroKPa) + "kPa");
  printField(200, 168, 80, 20, COLOR_VALUE, 2, String(engine.speedKmh) + "km/h");

  // Battery / Timing advance
  uint16_t battColor = (engine.batteryV < 11.5f || engine.batteryV > 15.0f)
                          ? COLOR_WARN : COLOR_GOOD;
  printField(10, 208, 80, 20, battColor, 2, String(engine.batteryV, 1) + "V");
  printField(100, 208, 80, 20, COLOR_VALUE, 2, String(engine.timingAdv) + "d");

#if ENABLE_CUSTOM_PIDS
  printField(200, 208, 100, 20, COLOR_VALUE, 2, String(engine.railPressure));
#endif
}
