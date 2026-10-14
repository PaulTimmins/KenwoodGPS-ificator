/*
  ESP32 GPS bridge for Kenwood TM-D700A

  On every boot:
    - Finds the GPS baud rate automatically
    - Switches the GPS link to 38400 (RAM only) so all sentences fit
    - Turns on all standard NMEA sentences (GGA, GLL, GSA, GSV, RMC, VTG)
  For every sentence:
    - Drops bad checksums, renames $GN -> $GP (fixing the checksum)
    - Radio gets only $GPRMC + $GPGGA at 4800, RS-232 polarity
    - Bluetooth "D700-GPS" gets everything
    - USB Serial Monitor shows what goes to the radio
  Power saving:
    - Bluetooth turns off after 1 hour without moving more than 50 m
    - Turns back on as soon as it moves

  Add new features in handleSentence() at the bottom.

  Wiring:
    GPS VCC -> 3V3            GPS GND -> GND
    GPS TX  -> GPIO16         GPS RX  -> GPIO17
    GPIO4   -> 1k resistor -> 2.5mm plug TIP   (data into radio)
    GND     -> 2.5mm plug SLEEVE
    (leave RING unconnected)
*/

#include "BluetoothSerial.h"

#define GPS_RX_PIN    16
#define GPS_TX_PIN    17
#define GPS_LINK_BAUD 38400

#define RADIO_TX_PIN  4
#define RADIO_RX_PIN  5      // unused, but must be a safe pin
#define RADIO_BAUD    4800
#define RADIO_INVERT  true   // true = RS-232 polarity

#define BT_NAME       "D700-GPS"
#define STATIONARY_MS (60UL * 60UL * 1000UL)   // 1 hour
#define MOVE_METERS   50.0

HardwareSerial gps(2);
HardwareSerial radio(1);
BluetoothSerial bt;

String line;

bool btOn = false;
bool haveAnchor = false;
double anchorLat = 0, anchorLon = 0;
unsigned long lastMoveMs = 0;


// ---------- u-blox helpers ----------

void sendUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  uint8_t a = 0, b = 0;
  uint8_t header[4] = { cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
  for (int i = 0; i < 4; i++)   { a += header[i];  b += a; }
  for (int i = 0; i < len; i++) { a += payload[i]; b += a; }

  gps.write(0xB5);
  gps.write(0x62);
  gps.write(header, 4);
  gps.write(payload, len);
  gps.write(a);
  gps.write(b);
  gps.flush();
  delay(50);
}

void setNmeaRate(uint8_t msgId, uint8_t rate) {
  uint8_t p[3] = { 0xF0, msgId, rate };
  sendUbx(0x06, 0x01, p, 3);
}

void setGpsBaud(uint32_t baud) {
  uint8_t p[20] = {
    0x01, 0x00, 0x00, 0x00,                       // UART1, reserved, txReady
    0xD0, 0x08, 0x00, 0x00,                       // 8N1
    (uint8_t)baud, (uint8_t)(baud >> 8), (uint8_t)(baud >> 16), (uint8_t)(baud >> 24),
    0x03, 0x00, 0x03, 0x00,                       // in/out: UBX + NMEA
    0x00, 0x00, 0x00, 0x00
  };
  sendUbx(0x06, 0x00, p, sizeof(p));
  delay(150);
  gps.updateBaudRate(baud);
  delay(100);
}

long findGpsBaud() {
  const long bauds[] = { 38400, 4800, 9600, 115200, 19200, 57600 };

  for (long baud : bauds) {
    gps.updateBaudRate(baud);
    delay(50);
    while (gps.available()) gps.read();

    String seen = "";
    unsigned long until = millis() + 1500;
    while (millis() < until && seen.length() < 300) {
      if (gps.available()) seen += (char)gps.read();
    }
    if (seen.indexOf("RMC,") >= 0 || seen.indexOf("GGA,") >= 0) return baud;
  }
  return 0;
}


// ---------- NMEA helpers ----------

uint8_t nmeaChecksum(const String &body) {   // body = text between $ and *
  uint8_t cs = 0;
  for (unsigned int i = 0; i < body.length(); i++) cs ^= body[i];
  return cs;
}

// Returns true if s is a valid sentence. Fixes talker ID and checksum in place.
bool cleanSentence(String &s) {
  int star = s.indexOf('*');
  if (!s.startsWith("$") || star < 0 || star + 3 > (int)s.length()) return false;

  String body = s.substring(1, star);
  uint8_t given = strtol(s.substring(star + 1, star + 3).c_str(), NULL, 16);
  if (nmeaChecksum(body) != given) return false;

  if (body.startsWith("GN")) body[1] = 'P';

  char cs[3];
  sprintf(cs, "%02X", nmeaChecksum(body));
  s = "$" + body + "*" + cs;
  return true;
}

// Returns comma-separated field n (0 = "$GPRMC")
String field(const String &s, int n) {
  int start = 0;
  for (int i = 0; i < n; i++) {
    start = s.indexOf(',', start) + 1;
    if (start == 0) return "";
  }
  int end = s.indexOf(',', start);
  if (end < 0) end = s.indexOf('*', start);
  return s.substring(start, end);
}

// "4230.62836","N" -> 42.5104...
double toDegrees(const String &val, const String &hemi) {
  double v = val.toDouble();
  int deg = (int)(v / 100);
  double d = deg + (v - deg * 100) / 60.0;
  if (hemi == "S" || hemi == "W") d = -d;
  return d;
}

double metersBetween(double lat1, double lon1, double lat2, double lon2) {
  double dy = (lat2 - lat1) * 111320.0;
  double dx = (lon2 - lon1) * 111320.0 * cos(lat1 * PI / 180.0);
  return sqrt(dx * dx + dy * dy);
}


// ---------- Bluetooth on/off ----------

void bluetoothOn() {
  if (btOn) return;
  bt.begin(BT_NAME);
  btOn = true;
  Serial.println("** Bluetooth ON");
}

void bluetoothOff() {
  if (!btOn) return;
  bt.end();
  btOn = false;
  Serial.println("** Bluetooth OFF (stationary 1 hour)");
}

// Called with every valid RMC. Tracks movement.
void checkMovement(const String &rmc) {
  if (field(rmc, 2) != "A") return;   // no fix, don't count as movement

  double lat = toDegrees(field(rmc, 3), field(rmc, 4));
  double lon = toDegrees(field(rmc, 5), field(rmc, 6));

  if (!haveAnchor || metersBetween(anchorLat, anchorLon, lat, lon) > MOVE_METERS) {
    anchorLat = lat;
    anchorLon = lon;
    haveAnchor = true;
    lastMoveMs = millis();
    bluetoothOn();
  }
}


// ---------- setup / loop ----------

void setup() {
  Serial.begin(115200);
  gps.setRxBufferSize(2048);
  gps.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  radio.setTxBufferSize(1024);
  radio.begin(RADIO_BAUD, SERIAL_8N1, RADIO_RX_PIN, RADIO_TX_PIN, RADIO_INVERT);
  delay(1000);

  Serial.println("\nLooking for GPS...");
  long baud = 0;
  while (baud == 0) {
    baud = findGpsBaud();
    if (baud == 0) Serial.println("  no GPS data yet, retrying (check GPS TX -> GPIO16)");
  }
  Serial.printf("  GPS found at %ld baud\n", baud);

  if (baud != GPS_LINK_BAUD) {
    setGpsBaud(GPS_LINK_BAUD);
    Serial.printf("  GPS link switched to %d baud\n", GPS_LINK_BAUD);
  }

  setNmeaRate(0x00, 1);  // GGA
  setNmeaRate(0x01, 1);  // GLL
  setNmeaRate(0x02, 1);  // GSA
  setNmeaRate(0x03, 1);  // GSV
  setNmeaRate(0x04, 1);  // RMC
  setNmeaRate(0x05, 1);  // VTG
  Serial.println("  All standard sentences on");

  bluetoothOn();
  lastMoveMs = millis();

  Serial.printf("Radio: %d baud, invert=%s | Bluetooth: \"%s\"\n\n",
                RADIO_BAUD, RADIO_INVERT ? "yes" : "no", BT_NAME);
}

void loop() {
  while (gps.available()) {
    char c = gps.read();
    if (c == '$') line = "";
    if (c == '\r') continue;

    if (c == '\n') {
      if (cleanSentence(line)) handleSentence(line);
      line = "";
    } else if (line.length() < 120) {
      line += c;
    }
  }

  if (btOn && millis() - lastMoveMs > STATIONARY_MS) bluetoothOff();
}

// Every valid, cleaned-up sentence lands here.
void handleSentence(const String &s) {
  if (btOn && bt.hasClient()) bt.print(s + "\r\n");

  if (s.startsWith("$GPRMC") || s.startsWith("$GPGGA")) {
    radio.print(s + "\r\n");
    Serial.println("-> " + s);
  }

  if (s.startsWith("$GPRMC")) checkMovement(s);
}
