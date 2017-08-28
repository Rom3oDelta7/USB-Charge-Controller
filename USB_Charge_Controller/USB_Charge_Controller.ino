/*
USB Charge Controller
A wirelessly-controlled USB power "switch" that cycles charging power to allow a device to periodically,
rather than continuously, charge. This extends battery life and prevents LiPo batteries from swelling
due to overcharging. Especially useful for devices that are always plugged in, such as iPads for a
home automation system.

Based on ESP8266 and Blynk.

Library sources:
 Blynk: https://www.blynk.cc
 OLED: https://github.com/squix78/esp8266-oled-ssd1306
 Time library: https://www.pjrc.com/teensy/td_libs_Time.html
   NTP RTC library "shim": https://github.com/Rom3oDelta7/NTP-RTC 

Copyright 2017 Rob Redford
This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.

*/



#include <ArduinoOTA.h>
//#include <Wire.h>
#include <WiFiUdp.h>
#include <WiFiServer.h>
#include <WiFiClientSecure.h>
#include <WiFiClient.h>
#include <ESP8266WiFiType.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
//#include <brzo_i2c.h>
//#include <SH1106Wire.h>
#include <SH1106Brzo.h>
#include <SH1106.h>
#include <OLEDDisplayUi.h>
#include <OLEDDisplayFonts.h>
#include <OLEDDisplay.h>
#include <TimeLib.h>
#include <WorldTimezones.h>
#include <NTPRTC.h>
#include "Credentials.h"                // your local WiFi SSID & password and Blynk authorization token

//#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG
#include <BlynkSimpleEsp8266.h>

#define DEBUG_INFO
//#define DEBUG_ERROR
#define DEBUG_LOG
#include "DebugLib.h"

// macro for debugging timing issues
//#define DEBUG_TIMING
#ifdef DEBUG_TIMING
   uint32_t lastMillis = millis();
   uint32_t elapsed;

#define ELAPSED(Label, Threshold) \
      elapsed = millis() - lastMillis; \
      lastMillis = millis(); \
      if (elapsed >= Threshold) {SerialIO.print(F("ELAPSED TIME: ")); INFO(Label, elapsed);} 
#else
   #define ELAPSED(...)
#endif



// ============================= Blynk ==================================================

/*
 Blynk customization (setProperty()) is currently not enbaled
 This is becuase the setProperties function for buttons and menus, for example, is taking multiple seconds
 to complete, resulting in Blynk resetting & reconnecting and thus making the UI unresponsive
*/

// dashbord controls
#define STATUS_LED       V0
#define MENU             V1
#define SCHEDULE         V2
#define PROGRESS_BAR     V3
#define MANUAL_OVERRIDE  V4
#define FORCE_DISABLE    V5
#define MANUAL_RUNTIME   V6

// LED status colors - Not currently used
#define BLYNK_RED        "#D3435C"            // off
#define BLYNK_YELLOW     "#ED9D00"            // manually disabled
#define BLYNK_GREEN      "#23C48E"            // charging

#define BLYNK_GREY       "#808080"            // greyed-out menu color
#define BLYNK_BLUE       "#04C0F8"            // active menu color; manual charging outside normal schedule

#define BLYNK_DEBOUNCE_TIME 1000              // delay for buttons pressed in Blynk (longer => software)

WidgetLED statusLED(STATUS_LED);

BlynkTimer timer;


// ============================= PINS ===================================================

#if defined(ESP8266) and !(defined(ARDUINO_ESP8266_NODEMCU) or defined(ARDUINO_ESP8266_WEMOS_D1MINI))
#warning pin symbols were not pre - defined
#define SDA            4    // (NodeMCU D2)
#define SCL            5    // (NodeMCU D1)
#endif

#define ENABLE        14    // D5
#define MANUAL        12    // D6

// software debounce for hardware events
#define DEBOUNCE_TIME 250

volatile bool     debounce = true;
volatile uint32_t	debounceStart = 0;


// ============================= OLED Display ===================================================

#define I2C_OLED      0x3C

SH1106Brzo display(I2C_OLED, SDA, SCL);               // 128 x 64 OLED monochrome screen connected via I2C


// ============================= General ===================================================

#define MY_TIMEZONE    UTC_MST              // select your timezone here (from WorldTimezones.h)

typedef enum:uint8_t { S_MANUAL, S_CHARGING, S_DISABLED, S_QUIESCENT } ChargeState;

void statusUpdate (const ChargeState cstate);

/*
 this class stores the unit config parameters in memory and provides member functions
 to save the configuration in EEPROM
 */
struct {
   int               start = 0;                       // scheduled start time (midnight)
   int               stop = 6 * 3600;                 // scheduled stop time (6 AM)
   uint8_t           menuSelection = 1;               // schedule menu preset selection
   ChargeState       chargeState = S_QUIESCENT;       // current state
   ChargeState       lastChargeState = S_QUIESCENT;   // track previous state to manage Blynk updates
   volatile bool     manual = false;             
   volatile bool     disable = false;
   bool              charging = false;                // controls charging MOSFET
   time_t            runtimeStart;                    // when charging started
   float             chargeProgress = 0.0;            // tracks runtime for progress bars
   uint32_t          savedRuntime = 0;                // save prior runtime if disabled during a scheduled cycle
} state;

// ============================= Functions ===================================================

/*
 ISR handler for manual override tactile momentary switch
*/
void manualOverride (void) {
   if ( !debounce ) {
      debounce = true;
      debounceStart = millis();

      state.manual = !state.manual;
   }
}


/*
 initialize over the air updates
 */
void setupOTA (void) {
   // Port defaults to 8266
   ArduinoOTA.setPort(8266);

   // Hostname defaults to esp8266-[ChipID]
   //ArduinoOTA.setHostname("myesp8266");

   // No authentication by default
   // ArduinoOTA.setPassword((const char *)"123");

   ArduinoOTA.onStart([]() {
      LOG(PSTR("OTA Start"));
   });
   ArduinoOTA.onEnd([]() {
      LOG(PSTR("\nOTA End. Restarting ..."));
      delay(5000);
      ESP.restart();
   });
   ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      LOG(PSTR("OTA Progress: %u%%\r"), (progress / (total / 100)));
   });
   ArduinoOTA.onError([](ota_error_t error) {
      LOG(PSTR("OTA Error[%u]: "), error);
      switch ( error ) {
      case OTA_AUTH_ERROR:
         LOG(PSTR("OTA Auth Failed"));
         break;

      case OTA_BEGIN_ERROR:
         LOG(PSTR("OTA Begin Failed"));
         break;

      case OTA_CONNECT_ERROR:
         LOG(PSTR("OTA Connect Failed"));
         break;

      case OTA_RECEIVE_ERROR:
         LOG(PSTR("OTA Receive Failed"));
         break;

      case OTA_END_ERROR:
         LOG(PSTR("OTA End Failed"));
         break;

      default:
         LOG(PSTR("OTA Unknown error"));
         break;
      }
   });

   ArduinoOTA.begin();
   LOG(PSTR("OTA Ready. IP Address: %s Chip ID %0X\n"), WiFi.localIP().toString().c_str(), ESP.getChipId());
}

/*
 handle power settings - called from timer so we can use Blynk API to update status
 this is called by the timer

 we also update the Blynk dashboard here (buit not the OLED, which is handled by statusUpdate
 we must manipulate the manual override button here since the state may have been changed in the manual button ISR too

 extra calls to yield() are here since the Blynk library can take up a fair amount of elapsed time
*/
void powerManagement (void) {
   static bool counterEnabled = false;         // true when runtimer counter is active
   
   INFO(F(">>>> Power Management"), "");
   INFO(F("\tmanual"), state.manual);
   INFO(F("\tdisable"), state.disable);

   ELAPSED(F("Start of pwrMgt"), 0);
   time_t timeNow = now();    // use a constant time value to avoid corner cases where, for example, the hour changes between function call
   int currentTimeinSecs = ((3600 * hour(timeNow)) + (60 * minute(timeNow)) + second(timeNow));
   if ( !state.disable ) {
      if ( (currentTimeinSecs >= state.start) && (currentTimeinSecs < state.stop) ) {
         // in the charging window
         if ( !counterEnabled ) {
            counterEnabled = true;
            state.runtimeStart = now();
         }
         INFO(F("\tState: charging"), state.runtimeStart);
         state.chargeState = S_CHARGING;
         if ( state.chargeState != state.lastChargeState ) {
            // throttle Blynk traffic if the state has not changed to minize connection resets
            state.lastChargeState = state.chargeState;
            statusLED.setColor(BLYNK_GREEN);
            statusLED.setLabel("ON");
            Blynk.virtualWrite(MANUAL_OVERRIDE, false);
            Blynk.virtualWrite(MANUAL_RUNTIME, " ");
         }
         state.charging = true;
         // the automatic cycle overrides manual setting unless diable has been set
         state.manual = false;
      } else if ( state.manual ) {
         if ( !counterEnabled ) {
            counterEnabled = true;
            state.runtimeStart = now();
            Blynk.virtualWrite(PROGRESS_BAR, 0);                 // progress bar inactive here - use runtime field
         }
         INFO(F("\tState: manual"), state.runtimeStart);
         state.chargeState = S_MANUAL;
         if ( state.chargeState != state.lastChargeState ) {
            state.lastChargeState = state.chargeState;
            statusLED.setColor(BLYNK_BLUE);
            statusLED.setLabel("MAN");
            Blynk.virtualWrite(MANUAL_OVERRIDE, true);
            Blynk.virtualWrite(MANUAL_RUNTIME, " ");             // clear field when manual first set
         }
         state.charging = true;
         state.savedRuntime = 0;                                 // reset anytime we are NOT in the charging window
      } else {
         // outside charging window
         INFO(F("\tState: OFF"), "");
         
         state.chargeState = S_QUIESCENT;
         if ( state.chargeState != state.lastChargeState ) {
            state.lastChargeState = state.chargeState;
            ELAPSED(F("About to set LED options"), 0);
            statusLED.setColor(BLYNK_RED);
            statusLED.setLabel("OFF");
            Blynk.virtualWrite(MANUAL_OVERRIDE, false);
            Blynk.virtualWrite(MANUAL_RUNTIME, " ");
            ELAPSED(F("After setting LED options"), 0);
         }
         // ONLY reset manual flag in the ISR or BLYNK_WRITE function (or S_CHARGING above)
         counterEnabled = false;
         state.charging = false;
         state.savedRuntime = 0;
      }
   } else {
      // forcibly disabled - auto and manual states
      INFO(F("\tState: disabled"), "");
      if ( state.chargeState == S_CHARGING ) {
         // add the current session runtime to the saved value every time user hits disable when charging is active
         state.savedRuntime += (now() - state.runtimeStart);
         INFO(F("saved runtime"), state.savedRuntime);
      }
      state.chargeState = S_DISABLED;
      if ( state.chargeState != state.lastChargeState ) {
         // Blynk dashboard state changed in BLYNK_WRITE for this button, unlike S_MANUAL
         state.lastChargeState = state.chargeState;
         statusLED.setColor(BLYNK_YELLOW);
         statusLED.setLabel("DSBLD");
         Blynk.virtualWrite(MANUAL_OVERRIDE, false);
         Blynk.virtualWrite(MANUAL_RUNTIME, " ");
      }

      counterEnabled = false;
      state.charging = false;
      state.manual = false;
   }

   yield();

   ELAPSED(F("At charging check"), 0);
   if ( state.charging ) {
      // update progress here and refer to this in statusUpdate also
      if ( state.chargeState == S_CHARGING ) {
         // note the progress bar is not cleared after scheduled charging stops to provide a visual record of recent status
         int scheduledSeconds =state.stop - state.start;
         state.chargeProgress = (static_cast<float>(now() - state.runtimeStart + state.savedRuntime) / static_cast<float>(scheduledSeconds)) * 100.0;
         INFO(F("Charge progress bar"), static_cast<int>(state.chargeProgress));
         Blynk.virtualWrite(PROGRESS_BAR, static_cast<int>(state.chargeProgress));
      } else if ( state.chargeState == S_MANUAL ) {
         char buf[64];
         int runtime = now() - state.runtimeStart;
         sprintf(buf, "Elapsed %02d:%02d:%02d", (runtime / 3600), ((runtime % 3600) / 60), (runtime % 60));
         Blynk.virtualWrite(MANUAL_RUNTIME, buf);
      }
   }

   yield();

   ELAPSED(F("END of Blynk dashboard udpate"), 0);
}

/*
 display the status OLED screen for the given unit state
*/
void statusUpdate (const ChargeState cstate) {
   display.clear();
   // common elements
   time_t timeNow = now();
   display.drawString(58, 0, String("WiFi ") + WiFi.RSSI() + "dBm");
   char buf[68];
   sprintf(buf, "%02d/%02d %02d:%02d", month(timeNow), day(timeNow), hour(timeNow), minute(timeNow));
   display.drawString(0, 12, String(buf));

   // time formats are MM:SS
   sprintf(buf, "Start %02d:%02d    Stop %02d:%02d", (state.start / 3600), ((state.start % 3600) / 60),
      (state.stop / 3600), ((state.stop % 3600) / 60));
   display.drawString(0, 24, String(buf));

   int scheduledSeconds = state.stop - state.start;
   sprintf(buf, "Runtime %02d:%02d   Blynk %s", (scheduledSeconds / 3600), ((scheduledSeconds % 3600) / 60), Blynk.connected() ? "OK" : "ERR");
   display.drawString(0, 48, String(buf));

   // state-specific
   int runtime = static_cast<int>(now() - state.runtimeStart);
   sprintf(buf, "Elapsed %02d:%02d:%02d", (runtime / 3600), ((runtime % 3600) / 60), (runtime % 60));

   switch ( cstate ) {
   case S_QUIESCENT:
      display.drawString(0, 0, String("Quiescent"));
      break;

   case S_CHARGING:
      display.drawString(0, 0, String("Charging"));
      display.drawProgressBar(0, 36, 120, 10, static_cast<int>(state.chargeProgress));
      break;

   case S_MANUAL:
      display.drawString(0, 0, String("Manual"));
      display.drawString(0, 36, String(buf));
      break;

   case S_DISABLED:
      display.drawString(0, 0, String("DISABLED"));
      break;

   default:
      break;
   }
   display.display();
}

// ============================= Blynk Actions =====================================
// all-cap functions are called by the dashboard when a button is pressed

/*
 set the start and stop time for a custom schedule
 use the Blynk convention of seconds since midnight

 start and stop times must be on the same day
*/
BLYNK_WRITE ( SCHEDULE ) {
   TimeInputParam schedule(param);
   int            value;

   INFO(F("Setting Schedule"), "");

   if ( schedule.hasStartTime() ) {
      value = param[0].asInt();
      INFO(F("  start raw"), value);
      if ( value != -1 ) {
         state.start = value;
      }
   }
   if ( schedule.hasStopTime() ) {
      value = param[1].asInt();
      INFO(F("  stop raw"), value);
      if ( value != -1 ) {
         state.stop = value;
      }
   }
   // if stop is later then start, then switch them
   if ( state.stop < state.start ) {
      int temp = state.start;
      state.start = state.stop;
      state.start = temp;
   }
   state.menuSelection = 4;
   Blynk.virtualWrite(MENU, 4);
}

/*
 Menu selection for preset or custom schedule
 Blynk uses sezonds since mindnight as the time params
 */
BLYNK_WRITE ( MENU ) {
   INFO(F("Menu selection"), param.asInt());
   state.menuSelection = param.asInt();
   switch ( state.menuSelection ) {
   case 1:
      // 6 hours
      state.start = 0;
      state.stop = (6 * 3600);   // 6 AM

      break;

   case 2:
      // 8 hours
      state.start = 0;
      state.stop = (8 * 3600);   // 8 AM
      break;

   case 3:
      // 10 hours
      state.start = 0;
      state.stop = (10 * 3600);   // 10 AM
      break;

   case 4:
      // custom schedule
      break;

   default:
      break;
   }
   // display the currently selected schedule
   Blynk.virtualWrite(SCHEDULE, state.start, state.stop);
}

/*
 manually enable charging
 press again to go back to normal cycle
 note this has no effect during a scheduled cycle
 */
BLYNK_WRITE ( MANUAL_OVERRIDE ) {
   static int lastPress = 0;
   if ( (millis() - lastPress) > BLYNK_DEBOUNCE_TIME ) {
      lastPress = millis();
      state.manual = !state.manual;
      INFO(F("Manual button pressed in Blynk"), "");
   } else {
      INFO(F("Manual button pressed in Blynk - IGNORED"), "");
   }
}

/*
 force charging to stop
 press again to go back to normal cycle
*/
BLYNK_WRITE (FORCE_DISABLE) {
   static int lastPress = 0;
   if ( (millis() - lastPress) > BLYNK_DEBOUNCE_TIME ) {
      state.disable = !state.disable;
      // we can reset Blynk dashbaord here since there is no physical button as with the manual override
      Blynk.virtualWrite(FORCE_DISABLE, state.disable);
      lastPress = millis();
      INFO(F("Disable button pressed in Blynk"), "");
   } else {
      INFO(F("Disable button pressed in Blynk - IGNORED"), "");
   }
}

BLYNK_CONNECTED () {
   Blynk.syncAll();
}
   

// ================================== Arduino section =====================================

void setup(void) {
   Serial.begin(115200);

   // OLED display
   INFO("Setup ...", "");
   display.init();
   display.flipScreenVertically();
   display.setFont(ArialMT_Plain_10);
   display.drawString(0, 0, String("Initializing ..."));

   display.drawProgressBar(0, 32, 120, 10, 0);
   display.display();

   // manual override pin
   pinMode(MANUAL, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(MANUAL), &manualOverride, FALLING);

   // enable pin (for MOSFET)
   pinMode(ENABLE, OUTPUT);
   digitalWrite(ENABLE, LOW);

   display.drawProgressBar(0, 32, 120, 10, 20);
   display.display();

   // Blynk dashboard
   INFO(F("Blynk init ..."), "");
   Blynk.begin(auth, ssid, pass);
   while ( !Blynk.connect() ) {							// wait for connection
      Blynk.run();
   }
   INFO(F("Blynk connected"), "");
   statusLED.setColor(BLYNK_RED);
   statusLED.setLabel("OFF");
   statusLED.on();

   display.drawProgressBar(0, 32, 120, 10, 40);
   display.display();

   // NTP source for RTC
   NTP_UTC_Timezone(MY_TIMEZONE);
   NTP_Init();

   display.drawProgressBar(0, 32, 120, 10, 60);
   display.display();

   // OTA
   setupOTA();
   yield();

   display.drawProgressBar(0, 32, 120, 10, 80);
   display.display();

   // restore server state for menus (async)
   Blynk.syncVirtual(SCHEDULE, MENU);

   // clear any lingering previous state that is now stale
   Blynk.virtualWrite(MANUAL_OVERRIDE, false);
   Blynk.virtualWrite(MANUAL_RUNTIME, " ");                     // strangeness ensues when a null string is passed, so send a space
   Blynk.virtualWrite(FORCE_DISABLE, false);
   Blynk.virtualWrite(PROGRESS_BAR, 0);
   Blynk.run();


   display.drawProgressBar(0, 32, 120, 10, 100);
   display.display();

   // need to balance UI responsiveness with Blynk - too fast and Blynk constantly reconnects
   timer.setInterval(5000, &powerManagement);
}



void loop(void) {

   // manage debounce window
   if ( debounce && ((millis() - debounceStart) > DEBOUNCE_TIME) ) {
      debounce = false;
   }

   timer.run();
   ELAPSED(F("After timer.run"), 500);
   Blynk.run();
   ELAPSED(F("After Blynk.run"), 200);

   // toggle the USB power MOSFET
   if ( state.charging ) {
      digitalWrite(ENABLE, HIGH);
   } else {
      digitalWrite(ENABLE, LOW);
   }

   // power_management called by timer; also udpates Blynk dashboard
   statusUpdate(state.chargeState);
   ELAPSED(F("After statusUpdate"), 100);
   
   ArduinoOTA.handle();
   ELAPSED(F("After OTA"), 100);
   yield();
}
