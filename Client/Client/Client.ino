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
#include <DHT.h>

const char *ver="!!version 0.2 client";

/************ Sensors Setup *************/
#define DHTTYPE DHT11   // DHT 11
#define DHTPIN 5     // Digital pin connected to the DHT sensor
DHT dht(DHTPIN, DHTTYPE);

#define SERIES_RESISTOR     560    // Value of the series resistor in ohms.    
#define SENSOR_PIN          0      // Analog pin which is connected to the sensor.
// The following are calibration values you can fill in to compute the volume of measured liquid.
// To find these values first start with no liquid present and record the resistance as the
// ZERO_VOLUME_RESISTANCE value.  Next fill the container with a known volume of liquid and record
// the sensor resistance (in ohms) as the CALIBRATION_RESISTANCE value, and the volume (which you've
// measured ahead of time) as CALIBRATION_VOLUME.
#define ZERO_VOLUME_RESISTANCE    2500.0    // Resistance value (in ohms) when no liquid is present.
#define CALIBRATION_RESISTANCE    660.00    // Resistance value (in ohms) when liquid is at max line.
#define CALIBRATION_VOLUME        100.00    // Volume (in any units) when liquid is at max line.

#define MAGSENSOR 7 //magnetic sensor pin
int magstate; //magnetic sensor state 0-close 1-open switch

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434.0

// Where to send packets to!
#define DEST_ADDRESS   1
// change addresses for each client board, any number :)
#define MY_ADDRESS     2


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

void setup()
{
  Serial.begin(115200);
  Serial.println(ver);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  //pinMode(LED, OUTPUT);
  //pinMode(13, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather Addressed RFM69 TX Test!");
  Serial.println();

  dht.begin(); //DHT11 sensor

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

  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  pinMode(MAGSENSOR, INPUT_PULLUP);
}


// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";

void loop() {
  delay(2000);  // Wait 2 second between transmits, could also 'sleep' here!

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float rh = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  float l;

  // Measure sensor resistance.
  float resistance = readResistance(SENSOR_PIN, SERIES_RESISTOR);
  Serial.print("Resistance: "); 
  Serial.print(resistance, 2);
  Serial.println(" ohms");
  // Map resistance to volume.
  float volume = resistanceToVolume(resistance, ZERO_VOLUME_RESISTANCE, CALIBRATION_RESISTANCE, CALIBRATION_VOLUME);
  Serial.print("Calculated volume: ");
  Serial.println(volume, 5);
  l=volume;
  
  Serial.print(F("RH: "));
  Serial.print(rh);
  Serial.print(F("%  T: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(l);
  Serial.println(F("%"));

  //check magnetic sensor
  magstate = digitalRead(MAGSENSOR);
  
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(rh) || isnan(t) || isnan(l)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  char radiopacket[23];
  char RelH[3];
  char tempC[3];
  char lvl[3];
  char res[5];
  char mags[2];
  strcpy (radiopacket,"H");
  dtostrf(rh, 2, 0, RelH); //convert RH to char
  strcat(radiopacket,RelH);
  dtostrf(t, 2, 0, tempC); // convert tempC to char
  strcat(radiopacket,"% T"); 
  strcat(radiopacket,tempC);
  strcat(radiopacket,"C ");
  strcat(radiopacket,"L");
  dtostrf(l, 2, 0, lvl); // convert level to char
  strcat(radiopacket,lvl);
  strcat(radiopacket,"% ");
  strcat(radiopacket,"Res");
  dtostrf(resistance, 4, 0, res); // convert resistance to char
  strcat(radiopacket,res);
  strcat(radiopacket,"Ohm");
  itoa(magstate, mags, 2);
  //Serial.println(mags);
  strcat(radiopacket," M");
  strcat(radiopacket,mags);
  strcat(radiopacket,"%");
  
//  itoa(BTmax, complete_path, 10); //convert int to charr
//  itoa(packetnum++, radiopacket + 2, 3);
//  itoa(h, radiopacket + 8, 2);
  Serial.print("Sending "); Serial.println(radiopacket);

  // Send a message to the DESTINATION!
  if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), DEST_ADDRESS)) {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (rf69_manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
      buf[len] = 0; // zero out remaining string

      Serial.print("Got reply from #"); Serial.print(from);
      Serial.print(" [RSSI :");
      Serial.print(rf69.lastRssi());
      Serial.print("] : ");
      Serial.println((char*)buf);
      Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks
    } else {
      Serial.println("No reply, is anyone listening?");
    }
  } else {
    Serial.println("Sending failed (no ack)");
    //digitalWrite(13, HIGH);
  }

  //Blink(LED,1000,1);
}


void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i = 0; i < loops; i++)  {
    digitalWrite(PIN, HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN, LOW);
    delay(DELAY_MS);
  }
}

float readResistance(int pin, int seriesResistance) {
  // Get ADC value.
  float resistance = analogRead(pin);
  // Convert ADC reading to resistance.
  resistance = (1023.0 / resistance) - 1.0;
  resistance = seriesResistance / resistance;
  return resistance;
}

float resistanceToVolume(float resistance, float zeroResistance, float calResistance, float calVolume) {
  if (resistance > zeroResistance || (zeroResistance - calResistance) == 0.0) {
    // Stop if the value is above the zero threshold, or no max resistance is set (would be divide by zero).
    return 0.0;
  }
  // Compute scale factor by mapping resistance to 0...1.0+ range relative to maxResistance value.
  float scale = (zeroResistance - resistance) / (zeroResistance - calResistance);
  // Scale maxVolume based on computed scale factor.
  return calVolume * scale;
}
