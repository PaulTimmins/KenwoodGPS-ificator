/*
  ESP32 GPS bridge for Kenwood TM-D700A

  On every boot:
    - Finds the GPS baud rate automatically (4800, 9600, ...)
    - Sets the GPS to output only GGA + RMC (in RAM, no flash wear)
  Then, for every sentence:
    - Drops anything with a bad checksum
    - Renames $GN... to $GP... (fixes checksum) so any u-blox module works
    - Passes only $GPRMC and $GPGGA on
  Outputs:
    - Radio: 4800 baud, RS-232 polarity (inverted)
    - Bluetooth SPP: "D700-GPS"
    - USB Serial Monitor: 115200

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

#define RADIO_TX_PIN  4
#define RADIO_RX_PIN  5      // unused, but must be a safe pin
#define RADIO_BAUD    4800
#define RADIO_INVERT  true   // true = RS-232 polarity

#define BT_NAME       "D700-GPS"

HardwareSerial gps(2);
HardwareSerial radio(1);
BluetoothSerial bt;

String line;


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

long findGpsBaud() {
  const long bauds[] = { 4800, 9600, 38400, 115200, 19200, 57600 };

  for (long baud : bauds) {
    gps.updateBaudRate(baud);
    delay(50);
    while (gps.available()) gps.read();

    String seen = "";
    unsigned long until = millis() + 1500;
    while (millis() < until && seen.length() < 300) {
      if (gps.available()) seen += (char)gps.read();
    }
    if (seen.indexOf("$GPRMC") >= 0 || seen.indexOf("$GNRMC") >= 0 ||
        seen.indexOf("$GPGGA") >= 0 || seen.indexOf("$GNGGA") >= 0) {
      return baud;
    }
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


// ---------- setup / loop ----------

void setup() {
  Serial.begin(115200);
  gps.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  radio.begin(RADIO_BAUD, SERIAL_8N1, RADIO_RX_PIN, RADIO_TX_PIN, RADIO_INVERT);
  bt.begin(BT_NAME);
  delay(1000);

  Serial.println("\nLooking for GPS...");
  long baud = 0;
  while (baud == 0) {
    baud = findGpsBaud();
    if (baud == 0) Serial.println("  no GPS data yet, retrying (check GPS TX -> GPIO16)");
  }
  Serial.printf("  GPS found at %ld baud\n", baud);

  setNmeaRate(0x00, 1);  // GGA on
  setNmeaRate(0x04, 1);  // RMC on
  setNmeaRate(0x01, 0);  // GLL off
  setNmeaRate(0x02, 0);  // GSA off
  setNmeaRate(0x03, 0);  // GSV off
  setNmeaRate(0x05, 0);  // VTG off
  Serial.println("  GPS set to GGA + RMC only");

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
}

// Every valid, cleaned-up sentence lands here.
void handleSentence(const String &s) {
  if (s.startsWith("$GPRMC") || s.startsWith("$GPGGA")) {
    radio.print(s + "\r\n");
    if (bt.hasClient()) bt.print(s + "\r\n");
    Serial.println("-> " + s);
  }
}
