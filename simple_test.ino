#include <lmic.h>
#include <hal/hal.h>
#include <WiFi.h>
#include <CayenneLPP.h>
#include <Tone32.h>

#include "config.h"

#define DEBUG
#define BUILTIN_LED 14
#define THRESHOLD 40 /* Greater the value, more the sensitivity */
#define TOUCH_PIN 15
#define TIME_TO_SLEEP 5
#define US_TO_SECOND_FACTOR 1000000U
#define BATTERY_PIN 35
#define GPS_FIX_RETRY_DELAY 25
#define BUZZER_PIN 14
#define BUZZER_CHANNEL 0

#ifdef DEBUG
#define TX_INTERVAL 60
#else
#define TX_INTERVAL 120
#endif

// Machine state definition
enum state {
  INIT = 0,
  ON = 1,
  OFF = 2,
  ERROR = 3
};

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 23,
  .dio = {26, 33, 32},  // PIN 33 HAS TO BE PHYSICALLY CONNECTED TO PIN Lora1 OF TTGO
};                      // the second connection from Lora2 to pin 32 is not necessary

// Object declaration
CayenneLPP lpp(51);
//gps gps;                                          // class that is encapsulating additional GPS functionality

// Variable declaration
RTC_DATA_ATTR state currentState = state::INIT;   // Default state of the machine
double lat, lon, alt, kmph;                       // GPS data are saved here: Latitude, Longitude, Altitude, Speed in km/h
int sats;                                         // GPS satellite count
char s[32];                                       // used to sprintf for Serial output
float vBat;                                       // battery voltage
long nextPacketTime;
static osjob_t sendjob;                           // callback to LoRa send packet


// Function prototype
void log(const char* i);            // Function used for debugging purposes
void os_getArtEui (u1_t* buf) { }   // Function prototype to get credentials. Declared here to avoid linking errors
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

void do_send(osjob_t* j);   // declaration of send function
void goSleepCallBack();     // Interuption attached to pin TOUCH_PIN when state is ON
void disablePeripherals();  // Power saving
void voidCallback() {}      // Empty function that's used as wakeup isr
void scheduledReboot(int seconds);
void errorMode() {          // Error mode disable everything and
  while (1) {
    tone(BUZZER_PIN, 440, 100, BUZZER_CHANNEL);
    //digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED));
    delay(1000);
  }
}



void onEvent(ev_t ev) {
  #ifdef DEBUG
  log("event");
  #endif
  switch (ev) {
    case EV_RESET:
    case EV_JOINING:
    case EV_RFU1:
    case EV_RXCOMPLETE:
    case EV_TXSTART:
    case EV_LINK_ALIVE:
      // Every previous event are a bit unknown but do not seems to be harmful
      // Skipping them
      break;
    case EV_JOINED:
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time)
      LMIC_setLinkCheckMode(0);
      break;
    case EV_TXCOMPLETE:
      // Schedule next transmission
      #ifdef DEBUG
      log("new message send");
      #endif
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    default:
      // Not handle event can be harmful going into error mode
      currentState = state::ERROR;
      scheduledReboot(2);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
    
  switch (currentState) {
    case state::INIT:
      // First launch
      #ifdef DEBUG
      log("State init");
      #endif
      currentState = state::OFF;
      scheduledReboot(2);
      
      break;
    case state::ON:
      #ifdef DEBUG
      log("State ON: Going to loop.");
      #endif
      tone(BUZZER_PIN, 440, 200, BUZZER_CHANNEL);
      // LMIC init
      os_init();
      LMIC_reset();
      LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
      // Reset the MAC state. Session and pending data transfers will be discarded.
      // Set up the channels used by the Things Network, which corresponds
      // to the defaults of most gateways. Without this, only three base
      // channels from the LoRaWAN specification are used, which certainly
      // works, so it is good for debugging, but can overload those
      // frequencies, so be sure to configure the full frequency range of
      // your network here (unless your network autoconfigures them).
      // Setting up channels should happen after LMIC_setSession, as that
      // configures the minimal channel set.
      // NA-US channels 0-71 are configured automatically
      LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
      LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
      // TTN defines an additional channel at 869.525Mhz using SF9 for class B
      // devices' ping slots. LMIC does not have an easy way to define set this
      // frequency and support for class B is spotty and untested, so this
      // frequency is not configured here.


      // Disable link check validation
      LMIC_setLinkCheckMode(0);

      LMIC.dn2Dr = DR_SF9;

      // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
       LMIC_setDrTxpow(DR_SF7, 14);
      
      touchAttachInterrupt(TOUCH_PIN, goSleepCallBack, THRESHOLD);
      disablePeripherals();
      do_send(&sendjob);
      break;
    case state::OFF:
      #ifdef DEBUG
      log("State OFF");
      #endif
      tone(BUZZER_PIN, 440, 100, BUZZER_CHANNEL);
      delay(50);
      tone(BUZZER_PIN, 440, 100, BUZZER_CHANNEL);
      currentState = state::ON;
      disablePeripherals();
      touchAttachInterrupt(TOUCH_PIN, voidCallback, THRESHOLD);
      esp_sleep_enable_touchpad_wakeup();
      esp_deep_sleep_start();
      break;
    case state::ERROR:
    #ifdef DEBUG
      log("error");
    #endif
      errorMode();
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running to avoid overflow
  if (!(LMIC.opmode & OP_TXRXPEND)) {

    //if (gps.checkGpsFix())
    if (true)
    {
      // Prepare upstream data transmission at the next possible time.
      //gps.getLatLon(&lat, &lon, &alt, &kmph, &sats);

      // we have all the data that we need, let's construct LPP packet for Cayenne
      lpp.reset();
      //lpp.addGPS(1, lat, lon, alt);
      lpp.addGPS(1, 50.8538, 04.478, 0);
      // read LPP packet bytes, write them to FIFO buffer of the LoRa module, queue packet to send to TTN
      LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
    }
    else
    {
      // try again in a few 'GPS_FIX_RETRY_DELAY' seconds...
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(GPS_FIX_RETRY_DELAY), do_send);
    }
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void loop() {
  os_runloop_once();
  //log("Error code reached loop\nRebooting");
  //currentState = state::INIT;
  //scheduledReboot(2);
}

void scheduledReboot(int seconds) {
  esp_sleep_enable_timer_wakeup(seconds * US_TO_SECOND_FACTOR);
  esp_deep_sleep_start();
}

void goSleepCallBack() {
  currentState = state::OFF;
  
  lpp.reset();
  lpp.addAnalogInput(2, analogRead(BATTERY_PIN));
  LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);

  esp_sleep_enable_touchpad_wakeup();
  esp_deep_sleep_start();
}

void disablePeripherals() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();
}

void log(const char* i) {
  Serial.println(i);
}
