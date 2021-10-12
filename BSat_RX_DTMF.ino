



/* Hamshield
 * Example: DTMF
 * This is a simple example to demonstrate how to use DTMF.
 *
 * Connect the HamShield to your Arduino. Screw the antenna 
 * into the HamShield RF jack. 
 * Connect the Arduino to wall power and then to your computer
 * via USB. After uploading this program to your Arduino, open
 * the Serial Monitor. Press the switch on the HamShield to 
 * begin setup. After setup is complete, type in a DTMF value
 * (0-9, A, B, C, D, *, #) and hit enter. The corresponding
 * DTMF tones will be transmitted. The sketch will also print
 * any received DTMF tones to the screen.
 * 
 * 
 * maybe this tnc could connect to the bluetoothau24 and just print the received nRF24L01 packets.
 * Translates DTMF to nRF24L01 and translates nRF24L01 to the bluetooth to view on your phone
**/

#include <HamShield.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"


#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
//create an RF24 object
RF24 radio(9, 8);  // CE, CSN

// create object for radio
HamShield nradio(A0, A3, A1); // nCS, CLK, DAT
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST); //remember pinout here

#define LED_PIN 13

#define MIC_PIN 3

int count = 0;
uint32_t freq;

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void setup() {
  // NOTE: if not using PWM out, it should be held low to avoid tx noise
  pinMode(MIC_PIN, OUTPUT);
  digitalWrite(MIC_PIN, LOW);
  
  // initialize device
  radio.initialize();
  Serial.begin(9600);
  Serial.println("setting default Radio configuration");

  Serial.println("setting squelch");

  radio.setSQHiThresh(-10);
  radio.setSQLoThresh(-30);
  Serial.print("sq hi: ");
  Serial.println(radio.getSQHiThresh());
  Serial.print("sq lo: ");
  Serial.println(radio.getSQLoThresh());
  radio.setSQOn();
  //radio.setSQOff();

  Serial.println("setting frequency to: ");
  freq = 432100; // 70cm calling frequency
  radio.frequency(freq);
  Serial.print(radio.getFrequency());
  Serial.println("kHz");
  
  // set RX volume to minimum to reduce false positives on DTMF rx
  radio.setVolume1(6);
  radio.setVolume2(0);
  
  // set to receive
  radio.setModeReceive();
  
  radio.setRfPower(0);
    


  // set up DTMF
  radio.enableDTMFReceive();
  
  /* DTMF timing settings are optional.
   * These times are set to default values when the device is started.
   * You may want to change them if you're DTMF receiver isn't detecting
   * codes from the HamShield (or vice versa).
   */
  radio.setDTMFDetectTime(24); // time to detect a DTMF code, units are 2.5ms
  radio.setDTMFIdleTime(50); // time between transmitted DTMF codes, units are 2.5ms
  radio.setDTMFTxTime(60); // duration of transmitted DTMF codes, units are 2.5ms
  //set the address
  nradio.openWritingPipe(address);
  
  //Set module as transmitter


  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

}

char rx_dtmf_buf[255];
int  rx_dtmf_idx = 0;
void loop() {
  
  // look for tone
  nradio.stopListening();
  if (radio.getDTMFSample() != 0) {
    uint16_t code = radio.getDTMFCode();
    if (code == 1) {
      //digitalWrite(OUTPUT_PIN, HIGH);
      const char text[] = "RESET BALLOON";
      radio.write(&text, sizeof(text));
      Serial.print(text);
      ble.print(text);
      count = count - 1;
      Serial.print("It works!");
      delay(1000);
      //digitalWrite(OUTPUT_PIN, LOW);
    }
      if (code == 2) {
      const char text[] = "RELEASE BALLOON";
      radio.write(&text, sizeof(text));
      Serial.print(text);
      ble.print(text);
      count++;
      delay(250);
      Serial.print("It works!");
      delay(1000);
      //digitalWrite(OUTPUT_PIN, LOW);
    }
    rx_dtmf_buf[rx_dtmf_idx++] = code2char(code);

    // reset after this tone
    int j = 0;
    while (j < 4) {
      if (radio.getDTMFSample() == 0) {
        j++;
      }
      delay(10);
    }
  } else if (rx_dtmf_idx > 0) {
  }
  
  // Is it time to send tone?
  //general read
  radio.startListening();
  char text[32] = {0};
  radio.read(&text, sizeof(text));
  ble.print(text); //sniff the radio waves
}

uint8_t char2code(char c) {
    uint8_t code;
    if (c == '#') {
      code = 0xF;
    } else if (c=='*') {
      code = 0xE;
    } else if (c >= 'A' && c <= 'D') {
      code = c - 'A' + 0xA;
    } else if (c >= '0' && c <= '9') {
      code = c - '0';
    } else {
      // invalid code, skip it
      code = 255;
    }

    return code;
}

char code2char(uint16_t code) {
  char c;
  if (code < 10) {
    c = '0' + code;
  } else if (code < 0xE) {
    c = 'A' + code - 10;
  } else if (code == 0xE) {
    c = '*';
  } else if (code == 0xF) {
    c = '#';
  } else {
    c = '?'; // invalid code
  }
 
  return c;
}
