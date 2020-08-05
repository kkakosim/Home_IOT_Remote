// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem
// configuration

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const char *ver = "!!version 0.2 server";

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

/**** OUTPUT Setup ****/
const int GREEN = 8;
const int YELLOW = 9;
const int RED = 10;
const int BUZZER = 7;

/********* LIMITS Setup **********/

const int HUMIDITY_LOW = 45;
const int HUMIDITY_HIGH = 55;
const int LEVEL_LOW = 75;
const int LEVEL_HIGH = 85;
bool normal = true;
bool updt = true;

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434.0

// who am i? (server address)
#define MY_ADDRESS     1

// These #defines make it easy to set the backlight color
#define LCD_OFF 0x0
#define LCD_RED 0x1
#define LCD_YELLOW 0x3
#define LCD_GREEN 0x2
#define LCD_TEAL 0x6
#define LCD_BLUE 0x4
#define LCD_VIOLET 0x5
#define LCD_WHITE 0x7

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
#define RFM69_CS      8
#define RFM69_INT     7
#define RFM69_RST     4
#define LED           13
#endif

#if defined(ADAFRUIT_FEATHER_M0) // Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13
#endif

#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
#define RFM69_INT     3  // 
#define RFM69_CS      4  //
#define RFM69_RST     2  // "A"
#define LED           13
#endif

#if defined(ESP8266)    // ESP8266 feather w/wing
#define RFM69_CS      2    // "E"
#define RFM69_IRQ     15   // "B"
#define RFM69_RST     16   // "D"
#define LED           0
#endif

#if defined(ESP32)    // ESP32 feather w/wing
#define RFM69_RST     13   // same as LED
#define RFM69_CS      33   // "B"
#define RFM69_INT     27   // "A"
#define LED           13
#endif

/* Teensy 3.x w/wing
  #define RFM69_RST     9   // "A"
  #define RFM69_CS      10   // "B"
  #define RFM69_IRQ     4    // "C"
  #define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/

/* WICED Feather w/wing
  #define RFM69_RST     PA4     // "A"
  #define RFM69_CS      PB4     // "B"
  #define RFM69_IRQ     PA15    // "C"
  #define RFM69_IRQN    RFM69_IRQ
*/

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);


int16_t packetnum = 0;  // packet counter, we increment per xmission
int errorcycles = 0;  // counter of error cyclew


void setup()
{
  Serial.begin(115200);
  Serial.println(ver);
  normal = false;

  // initialize the digital pin as an output.
  pinMode(GREEN, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  LedOut();

  Blink(GREEN, 100, 2);
  Buzz(BUZZER, 1000, 1);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("start up");
  //lcd.setBacklight(LOW);

  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);


  Serial.println("Feather Addressed RFM69 RX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x08, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x07, 0x06, 0x07, 0x08
                  };
  rf69.setEncryptionKey(key);

  //pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}


// Dont put this on the stack:
uint8_t data[] = "Link Ok";
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void loop() {
  char state[15];
  LedOut();
  updt = false;
  if (rf69_manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;

    if (rf69_manager.recvfromAck(buf, &len, &from)) {
      buf[len] = 0; // zero out remaining string
      String text = (char*) buf;
      char* incoming = (char*) buf;
      char* tok;
      char lvlc[3];
      normal = true;
      errorcycles = 0;
      updt = true;

      tok = strtok(incoming, "H");
      tok = strtok(tok, "%");
      int RH = atoi(tok);
      tok = strtok(0, "T");
      tok = strtok(0, "C");
      int tempC = atoi(tok);
      tok = strtok(0, "L");
      tok = strtok(0, "%");
      int lvl = atoi(tok);
      itoa(lvl, lvlc, 10);
      tok = strtok(0, "M");
      tok = strtok(0, "%");
      int magsensor = atoi(tok);
      //Serial.println(magsensor);
      //strcat(text,"

      //Serial.print("R#"); Serial.print(from);
      //Serial.print(" [RSSI :");
      //Serial.print(rf69.lastRssi());
      Serial.println(text);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(text);

      Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks

      strcpy(state, "Normal ");
      if (RH >= HUMIDITY_HIGH || lvl >= LEVEL_HIGH) {
        lcd.setBacklight(HIGH);
        Buzz(BUZZER, 100, 3);
        Blink(RED, 100, 3);
      }
      else if (RH >= HUMIDITY_LOW || lvl >= LEVEL_LOW) {
        lcd.setBacklight(LCD_YELLOW);
        Blink(YELLOW, 100, 3);
      }
      else {
        lcd.setBacklight(HIGH);
        digitalWrite(GREEN, HIGH);   // all LED OFF
      }

      if (RH >= HUMIDITY_HIGH) {
        strcpy(state, "H% High ");
      }
      else if (RH >= HUMIDITY_LOW) {
        strcpy(state, "H% Medium ");
      }
      if (lvl >= LEVEL_HIGH) {
        strcpy(state, "Tank Full ");
      }
      else if (lvl >= LEVEL_LOW) {
        strcpy(state, "Empty Tank ");
      }

      if (magsensor) {
        lcd.setBacklight(HIGH);
        strcpy(state, "Door Open");
        Buzz(BUZZER, 50, 5);
        Blink(RED, 50, 5);
        Blink(LED, 50, 5);
      }
      

      // Send a reply back to the originator client
      if (!rf69_manager.sendtoWait(state, sizeof(state), from)) {
        Serial.println("Sending failed (no ack)");
        normal = false;
        errorcycles += 1;
      }
    }

  }
  else {
    errorcycles += 1;
    delay(500);
  }
  if (errorcycles > 60) {
    strcpy(state, "No Com. ");
    lcd.setBacklight(HIGH);
    digitalWrite(RED, HIGH);
    delay(1000);
    Buzz(BUZZER, 100, 1);
  }
  //Serial.println(errorcycles);

  //*** Print Alarms ****/
  if (errorcycles > 60 || updt) {
    //char errc[3];
    //itoa(errorcycles, errc, 10);
    //strcat(state, errc);
    lcd.setCursor(0, 1);
    lcd.print("> ");
    lcd.print(state);
    lcd.setCursor(14, 1);
    lcd.print(errorcycles);
    Serial.print("Status: ");
    Serial.println(state);
  }

}


void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i = 0; i < loops; i++)  {
    digitalWrite(PIN, HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN, LOW);
    delay(DELAY_MS);
  }
}

void Buzz(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i = 0; i < loops; i++)  {
    tone(PIN, 1000);
    delay(DELAY_MS);
    noTone(PIN);
    delay(DELAY_MS);
  }
}

void LedOut() {
  digitalWrite(YELLOW, LOW);   // all LED OFF
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, LOW);
}

// https://stackoverflow.com/questions/9072320/split-string-into-string-array
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
