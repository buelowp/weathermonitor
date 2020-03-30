/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/work/Projects/particle.io/weathermonitor/src/weathermonitor.ino"
/*
  This example sketch will walk you through how to tune the resonance frequency
  of the antenna using the IC's internal tuning caps. You'll need a logic analyzer, 
  oscillscope, or some method of reading a square wave of at least 4kHz but up to 32kHz.
  A note on what you can expect from a board fresh from SparkFun.
  The resonance frequency of a freshly manufactured SparkFun AS3935 Lightning
  Detectorw has been ~496kHz which is less then one percent deviation from
  perfect resonance. This falls well within the optimal value of 3.5 percent
  suggested in the datasheet on page 35. Again, 3.5 percent is OPTIMAL so try
  not to tear your hair out if it's not perfect. 
  By: Elias Santistevan
  SparkFun Electronics
  Date: April, 2019
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
*/

#include <SPI.h>
#include <Wire.h>
#include <SHT1x.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include <SparkFun_AS3935_Lightning_Detector_Arduino_Library.h>

int currentTimeZone();
void returnTunables();
void as3935Interrupt();
void calibrate();
int setMaskValue(String v);
int setSpikeRejectionValue(String v);
int setNoiseFloorValue(String v);
int setIndoor(String v);
void applicationSetup();
bool startupLightningDetector();
void setup();
void loop();
#line 24 "/work/Projects/particle.io/weathermonitor/src/weathermonitor.ino"
void mqttCallback(char* topic, byte* payload, unsigned int length);

#define APP_ID              59

#define dataPin             D2         // Yellow       // Brown is power, black is ground
#define clockPin            D3         // Blue
#define CALIBRATE_F         0
#define CALIBRATE_C         0
#define ANTFREQ             3
#define INDOOR              0x12 
#define OUTDOOR             0xE
#define LIGHTNING_INT       0x08
#define DISTURBER_INT       0x04
#define NOISE_INT           0x01
#define INTERRUPT_PIN       D5
#define CS_PIN              A2
#define AS3935_PWR          A0
#define NOISE_INTTERUPT_MAX_COUNT   2
#define ONE_SECOND          1000
#define FIVE_SECONDS        (5 * ONE_SECOND)
#define ONE_MINUTE          (60 * ONE_SECOND)
#define ONE_HOUR            (60 * ONE_MINUTE)
#define SIX_HOURS           (6 * ONE_HOUR)
#define CST_OFFSET          -6
#define DST_OFFSET          (CST_OFFSET + 1)
#define TIME_BASE_YEAR		2019

// Chip select for SPI on pin ten.
double g_tempc;
double g_tempf;
double g_humidity;
int g_appid;
int g_count;
int g_noiseCount;
int g_maskValue;
int g_spikeRejection;
int g_threshold;
int g_connected;
int g_timeZone;
int g_delay;
bool g_indoor;
bool g_lastCalibration;
int g_lastDistance;
long g_lastEnergy;
unsigned long g_lastTimeFix;
byte g_watchDogValue;
int g_tuneValue;
unsigned long g_lastMillis;
unsigned long g_lastLoop;
unsigned long g_lastNoiseEvent;
unsigned long g_lastReading;
int g_noiseFloor = 2;
String g_lastEventTime;
String g_name = "weathermonitor-";
String g_mqttName = g_name + System.deviceID().substring(0, 8);
String g_tunablesMessage = "weather/request/tunables";
byte mqttServer[] = {172, 24, 1, 13};
MQTT client(mqttServer, 1883, mqttCallback);

const uint8_t _usDSTStart[22] = { 10, 8,14,13,12,10, 9, 8,14,12,11,10, 9,14,13,12,11, 9};
const uint8_t _usDSTEnd[22]   = { 3, 1, 7, 6, 5, 3, 2, 1, 7, 5, 4, 3, 2, 7, 6, 5, 4, 2};

// SPI
SparkFun_AS3935 lightning;
SHT1x sht1x(dataPin, clockPin);

STARTUP(WiFi.selectAntenna(ANT_INTERNAL));

int currentTimeZone()
{
    g_timeZone = DST_OFFSET;
    if (Time.month() > 3 && Time.month() < 11) {
        return DST_OFFSET;
    }
    if (Time.month() == 3) {
        if ((Time.day() == _usDSTStart[Time.year() - TIME_BASE_YEAR]) && Time.hour() >= 2)
            return DST_OFFSET;
        if (Time.day() > _usDSTStart[Time.year() - TIME_BASE_YEAR])
            return DST_OFFSET;
    }
    if (Time.month() == 11) {
        if ((Time.day() == _usDSTEnd[Time.year() - TIME_BASE_YEAR]) && Time.hour() <=2)
            return DST_OFFSET;
        if (Time.day() < _usDSTEnd[Time.year() - TIME_BASE_YEAR])
            return DST_OFFSET;
    }
    g_timeZone = CST_OFFSET;
    return CST_OFFSET;
}

void returnTunables()
{
    StaticJsonDocument<500> json;
 
    g_maskValue = lightning.readMaskDisturber();
    Serial.print("Are disturbers being masked: "); 
    if (g_maskValue == 1) {
        Serial.println("YES");
        json["device"]["AS3935"]["disturbers"] = "MASKED";
    } 
    else if (g_maskValue == 0) {
        Serial.println("NO"); 
        json["device"]["AS3935"]["disturbers"] = "UNMASKED";
    }

    int enviVal = lightning.readIndoorOutdoor();
    Serial.print("Are we set for indoor or outdoor: ");  
    if(enviVal == INDOOR) {
        Serial.println("Indoor.");
        g_indoor = true;
        json["device"]["AS3935"]["indoor"] = "TRUE";
    }  
    else if( enviVal == OUTDOOR ) {
        Serial.println("Outdoor.");
        g_indoor = false;
        json["device"]["AS3935"]["indoor"] = "FALSE";
    }  
    else { 
        Serial.println(enviVal, BIN);
        json["device"]["AS3935"]["indoor"] = "ERROR";
    }

    g_noiseFloor = lightning.readNoiseLevel();
    Serial.print("Noise Level is set at: ");
    Serial.println(g_noiseFloor);
    json["device"]["AS3935"]["noisefloor"] = g_noiseFloor;

    g_watchDogValue = lightning.readWatchdogThreshold();
    Serial.print("Watchdog Threshold is set to: ");
    Serial.println(g_watchDogValue);
    json["device"]["AS3935"]["watchdog"] = g_watchDogValue;
    
    g_spikeRejection = lightning.readSpikeRejection();
    Serial.print("Spike Rejection is set to: ");
    Serial.println(g_spikeRejection);
    json["device"]["AS3935"]["spikereject"] = g_spikeRejection;

    g_threshold = lightning.readLightningThreshold();
    Serial.print("The number of strikes before interrupt is triggerd: "); 
    json["device"]["AS3935"]["threshold"] = g_threshold;
    Serial.println(g_threshold);

    json["time"]["timezone"] = Time.zone();
    json["time"]["now"] = Time.local();
    json["network"]["ssid"] = WiFi.SSID();
//    json["device"]["AS3935"]["startup"] = g_lastCalibration;
    json["photon"]["id"] = System.deviceID();
    json["photon"]["version"] = System.version();
    json["photon"]["appid"] = g_appid;
        
    char buffer[512];
    serializeJson(json, buffer);
    String msg(buffer);
    client.publish("weather/device", msg.trim(), 1);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) 
{
    /*
    if (length != 0) {
        char p[length + 1];
        memcpy(p, payload, length);
        p[length + 1] = '\0';
    }
    */
    String msg(topic);

    Serial.print("Got message on topic ");
    Serial.println(msg);

    if (msg == g_tunablesMessage) {
        returnTunables();
    }
}

void as3935Interrupt()
{
    g_count++;
}

void calibrate()
{
    int16_t errorVal = 0;
	int16_t bestError = 0x7FFF;

    attachInterrupt(INTERRUPT_PIN, as3935Interrupt, RISING);

    for (int tuning = 0; tuning < 16; tuning++) {
        noInterrupts();
        g_count = 0;
        interrupts();
        delay(40);
        noInterrupts();
		errorVal = g_count - 1250;
		interrupts();

		Serial.printf("Tuning value: %d, error: %d\n", tuning, errorVal);

		if (errorVal < 0)
			errorVal = -errorVal;
		if (errorVal < bestError) {
			bestError = errorVal;
			g_tuneValue = tuning;
		}
        lightning.tuneCap(g_tuneValue);
    }
    Serial.printf("Selected %d as best tuning value\n", g_tuneValue);
    lightning.tuneCap(g_tuneValue);
    detachInterrupt(INTERRUPT_PIN);
    delay(g_delay);
}

int setMaskValue(String v)
{
    if (v == "0") {
        lightning.maskDisturber(false);
        g_maskValue = 0;
        return 0;
    }
    else
        lightning.maskDisturber(true);

    delay(g_delay);
    g_maskValue = lightning.readMaskDisturber();

    StaticJsonDocument<200> json;
    json["device"]["name"] = "AS3935";
    json["device"]["attribute"]["disturbers"] = g_maskValue;

    char buffer[201];
    serializeJson(json, buffer);
    String msg(buffer);
    client.publish("weather/device/change", msg.trim(), 0);

    return g_maskValue;
}

int setSpikeRejectionValue(String v)
{
    int reject = v.toInt();
    lightning.spikeRejection(reject);
    delay(g_delay);
    g_spikeRejection = reject;

    StaticJsonDocument<200> json;
    json["device"]["name"] = "AS3935";
    json["device"]["attribute"]["spikereject"] = g_spikeRejection;

    char buffer[201];
    serializeJson(json, buffer);
    String msg(buffer);
    client.publish("weather/device/change", msg.trim(), 0);

    return reject;
}

int setNoiseFloorValue(String v)
{
    int floor = v.toInt();
    lightning.setNoiseLevel(floor);
    delay(g_delay);
    g_noiseFloor = lightning.readNoiseLevel();

    StaticJsonDocument<200> json;
    json["device"]["name"] = "AS3935";
    json["device"]["attribute"]["noisefloor"] = g_noiseFloor;

    char buffer[201];
    serializeJson(json, buffer);
    String msg(buffer);
    client.publish("weather/device/change", msg.trim(), 0);

    return g_noiseFloor;
}

int setIndoor(String v)
{
    if (v == "0")
        lightning.setIndoorOutdoor(OUTDOOR);
    else
        lightning.setIndoorOutdoor(INDOOR);

    delay(g_delay);
    int enviVal = lightning.readIndoorOutdoor();
 
    if(enviVal == INDOOR) {
        g_indoor = true;
    }  
    else if( enviVal == OUTDOOR ) {
        g_indoor = false;
    }  
    else 
        Serial.println(enviVal, BIN); 
    
    StaticJsonDocument<200> json;
    json["device"]["name"] = "AS3935";
    json["device"]["attribute"]["indoor"] = g_indoor;

    char buffer[201];
    serializeJson(json, buffer);
    String msg(buffer);
    client.publish("weather/device/change", msg.trim(), 0);

    return g_indoor;
}

void applicationSetup()
{
    StaticJsonDocument<200> json;
 
    json["device"]["name"] = "AS3935";

    lightning.maskDisturber(true); 
    delay(g_delay);
    g_maskValue = lightning.readMaskDisturber();
    Serial.print("Are disturbers being masked: "); 
    if (g_maskValue == 1) {
        Serial.println("YES");
        json["device"]["AS3935"]["disturbers"] = "MASKED";
    } 
    else if (g_maskValue == 0) {
        Serial.println("NO"); 
        json["device"]["AS3935"]["disturbers"] = "UNMASKED";
    }

    // The lightning detector defaults to an indoor setting (less
    // gain/sensitivity), if you plan on using this outdoors 
    // uncomment the following line:
    lightning.setIndoorOutdoor(g_indoor); 
    delay(g_delay);
    int enviVal = lightning.readIndoorOutdoor();
    Serial.print("Are we set for indoor or outdoor: ");  
    if(enviVal == INDOOR) {
        Serial.println("Indoor.");
        g_indoor = true;
        json["device"]["AS3935"]["indoor"] = "TRUE";
    }  
    else if( enviVal == OUTDOOR ) {
        Serial.println("Outdoor.");
        g_indoor = false;
        json["device"]["AS3935"]["indoor"] = "FALSE";
    }  
    else { 
        Serial.println(enviVal, BIN);
        json["device"]["AS3935"]["indoor"] = "ERROR";
    }

    // Noise floor setting from 1-7, one being the lowest. Default setting is
    // two. If you need to check the setting, the corresponding function for
    // reading the function follows.    
    lightning.setNoiseLevel(g_noiseFloor);  
    delay(g_delay);
    g_noiseFloor = lightning.readNoiseLevel();
    Serial.print("Noise Level is set at: ");
    Serial.println(g_noiseFloor);
    json["device"]["AS3935"]["noisefloor"] = g_noiseFloor;

    // Watchdog threshold setting can be from 1-10, one being the lowest. Default setting is
    // two. If you need to check the setting, the corresponding function for
    // reading the function follows.    
    lightning.watchdogThreshold(g_watchDogValue); 
    delay(g_delay);
    g_watchDogValue = lightning.readWatchdogThreshold();
    Serial.print("Watchdog Threshold is set to: ");
    Serial.println(g_watchDogValue);
    json["device"]["AS3935"]["watchdog"] = g_watchDogValue;
    
    // Spike Rejection setting from 1-11, one being the lowest. Default setting is
    // two. If you need to check the setting, the corresponding function for
    // reading the function follows.    
    // The shape of the spike is analyzed during the chip's
    // validation routine. You can round this spike at the cost of sensitivity to
    // distant events. 
    lightning.spikeRejection(g_spikeRejection); 
    delay(g_delay);
    g_spikeRejection = lightning.readSpikeRejection();
    Serial.print("Spike Rejection is set to: ");
    Serial.println(g_spikeRejection);
    json["device"]["AS3935"]["spikereject"] = g_spikeRejection;

    // This setting will change when the lightning detector issues an interrupt.
    // For example you will only get an interrupt after five lightning strikes
    // instead of one. Default is one, and it takes settings of 1, 5, 9 and 16.   
    // Followed by its corresponding read function. Default is zero. 
    lightning.lightningThreshold(g_threshold); 
    delay(g_delay);
    g_threshold = lightning.readLightningThreshold();
    Serial.print("The number of strikes before interrupt is triggerd: "); 
    json["device"]["AS3935"]["threshold"] = g_threshold;
    Serial.println(g_threshold);
        
    char buffer[201];
    serializeJson(json, buffer);
    String msg(buffer);
    client.publish("weather/device", msg.trim(), 1);
}

bool startupLightningDetector()
{
    bool rval = false;

    SPI.begin(); // For SPI
    Serial.println("AS3935 Franklin Lightning Detector"); 
    if (!lightning.beginSPI(CS_PIN, 2000000) ) { 
        Serial.println ("Lightning Detector did not start up, freezing!"); 
        while(1); 
    }
    else
        Serial.println("Ready to tune antenna!");

    lightning.resetSettings();
    delay(g_delay);

    // The frequency of the antenna is divided by the "division ratio" which is
    // set to 16 by default. This can be changed to 32, 64, or 128 using the 
    // function call below. As an example when reading the frequency of a 
    // new board, the frequency 31.04kHz. Multiplying that frequency by the
    // division ratio gives 496.64kHz, which is less than 1 percent within the
    // OPTIMAL range of 3.5 percent specified in the datasheet.
    //lightning.changeDivRatio(32);

    // Read the division ratio - 16 is default.  
    byte divVal = lightning.readDivRatio(); 
    Serial.print("Division Ratio is set to: "); 
    Serial.println(divVal);

    // Here you give the value of the capacitor you want turned on. It accepts up
    // to 120pF in steps of 8pF: 8, 16, 24, 32 etc.The change in frequency is
    // somewhat modest. At the maximum value you can lower the frequency up to 22kHz. 
    // As a starting point, the products designed in house ship around 496kHz
    // (though of course every board is different) putting you within one percent
    // of a perfect resonance; the datasheet specifies being within 3.5 percent as
    // optimal. 

    //lightning.tuneCap(8); 

    // When reading the internal capcitor value, it will return the value in pF.
    int tuneVal = lightning.readTuneCap();
    Serial.print("Internal Capacitor is set to: "); 
    Serial.println(tuneVal);

    // This will tell the IC to display the resonance frequncy as a digital
    // signal on the interrupt pin. There are two other internal oscillators 
    // within the chip that can also be displayed on this line but is outside the
    // scope of this example, see page 35 of the datsheet for more information.
    Serial.println("\n----Displaying oscillator on INT pin.----\n"); 
    lightning.displayOscillator(true, ANTFREQ); 
    calibrate();
    // To stop displaying the frequncy on the interrupt line, give "false" as a
    // parameter or power down your lightning detector.
    lightning.displayOscillator(false, ANTFREQ); 

    // You can now calibrate the internal oscillators of the IC - given that the
    // resonance frequency of the antenna is tightly trimmed. They are calibrated
    // off of the antenna frequency. 
    if(lightning.calibrateOsc()) {
        Serial.println("Successfully Calibrated!");
        rval = true;
    }
    else {
        Serial.println("Not Successfully Calibrated!");
        rval = false;
    }
    return rval;
}

void setup()
{
    g_appid = APP_ID;
    g_count = 0;
    g_noiseCount = 0;
    g_noiseFloor = 1;
    g_spikeRejection = 8;
    g_threshold = 5;
    g_watchDogValue = 2;
    g_indoor = false;
    g_lastMillis = 0;
    g_connected = 0;
    g_lastLoop = 0;
    g_lastDistance = 0;
    g_noiseFloor = 0;
    g_tuneValue = 0;
    g_lastNoiseEvent = 0;
    g_lastReading = 0;
    g_watchDogValue = 2;
    g_delay = 10;
    g_lastCalibration = false;

    Particle.variable("appid", g_appid);
    Particle.variable("calibrated", g_tuneValue);
    Particle.variable("noisefloor", g_noiseFloor);
    Particle.variable("threshold", g_threshold);
    Particle.variable("distance", g_lastDistance);
    Particle.variable("indoor", g_indoor);
    Particle.variable("conn_mqtt", g_connected);
    Particle.variable("last_event", g_lastEventTime);
    Particle.variable("humidity", g_humidity);
    Particle.variable("farenheit", g_tempf);
    Particle.function("setmask", setMaskValue);
    Particle.function("setspike", setSpikeRejectionValue);
    Particle.function("setnoise", setNoiseFloorValue);
    Particle.function("setindoor", setIndoor);

    Serial.begin(115200);

    pinMode(D7, OUTPUT);

    delay(2000);        // Give our devices a bit of time to stabilize

    StaticJsonDocument<250> json;
    json["photon"]["id"] = System.deviceID();
    json["photon"]["version"] = System.version();
    json["photon"]["appid"] = g_appid;
    json["reset"]["reason"] = System.resetReason();

    client.connect(g_mqttName.c_str());
    if (client.isConnected()) {
        g_connected = 1;
        digitalWrite(D7, LOW);
        client.subscribe("weather/request/tunables");
    }

    g_lastCalibration = startupLightningDetector();

    Time.zone(currentTimeZone());
    json["time"]["timezone"] = Time.zone();
    json["time"]["now"] = Time.local();
    json["network"]["ssid"] = WiFi.SSID();
    if (g_lastCalibration) {
        json["device"]["AS3935"]["startup"] = "calibrated";
    }
    else
    {
        json["device"]["AS3935"]["startup"] = "notcalibrated";
    }
    

    char buffer[251];
    serializeJson(json, buffer);
    String msg(buffer);
    client.publish("weather/startup", msg.trim(), 1);
    applicationSetup();
}

void loop() 
{
    /*
     * If the number of noise events in a one minute period is more than max
     * bump the noise floor up one and see if it stabilizes
     */
    if (g_noiseCount >= NOISE_INTTERUPT_MAX_COUNT) {
        if (millis() <= (g_lastNoiseEvent + ONE_MINUTE)) {
            g_noiseCount = 0;
            lightning.setNoiseLevel(++g_noiseFloor);

            StaticJsonDocument<200> json;
            json["device"]["name"] = "AS3935";
            json["device"]["attribute"]["noisefloor"] = g_noiseFloor;

            char buffer[201];
            serializeJson(json, buffer);
            String msg(buffer);
            client.publish("weather/device/change", msg.trim(), 0);
        }
    }

    /*
     * Set time on the device every 6 hours to keep it up
     * to date for event logging. Also correctly sets the timezone then.
     */
    if ((g_lastTimeFix + SIX_HOURS) <= millis()) {
        Particle.syncTime();
        g_lastTimeFix = millis();
        Time.zone(currentTimeZone());
        StaticJsonDocument<200> json;
        json["time"]["now"] = Time.now();
        json["time"]["timezone"] = currentTimeZone();

        char buffer[201];
        serializeJson(json, buffer);
        String msg(buffer);
        client.publish("weather/timestamp", msg.trim(), 0);
    }

    /*
     * Handle the MQTT loop
     */
    if (client.isConnected()) {
        if (millis() > (g_lastLoop + FIVE_SECONDS)) {
            client.loop();
            g_lastLoop = millis();
        }   
    }
    else {
        Serial.println("We're not looping");
        g_connected = 0;
        digitalWrite(D7, HIGH);
        client.connect(g_mqttName.c_str());
        if (client.isConnected()) {
            g_connected = 1;
            digitalWrite(D7, LOW);
            client.subscribe("weather/request/#");
        }
    }

    /*
     * If we go more than a minute without noise, reset the count
     * which will allow us to more easily handle noise coming and
     * going.
     */
    if ((millis() > (g_lastNoiseEvent + ONE_MINUTE)) && g_noiseCount) {
        g_noiseCount = 0;
    }

    /*
     * If we go more than 6 hours without a noise event, try
     * to reduce the noise floor.
     */
    if (millis() > (g_lastNoiseEvent + SIX_HOURS)) {
        if (g_noiseFloor > 1) {
            g_noiseFloor--;
            lightning.setNoiseLevel(g_noiseFloor);
            
            StaticJsonDocument<200> json;
            json["device"]["name"] = "AS3935";
            json["device"]["attribute"]["noisefloor"] = g_noiseFloor;

            char buffer[201];
            serializeJson(json, buffer);
            String msg(buffer);
            client.publish("weather/device/change", msg.trim(), 0);
        }
    }

    if (digitalRead(INTERRUPT_PIN) == HIGH) {
        // Hardware has alerted us to an event, now we read the interrupt register
        // to see exactly what it is. 
        int intVal = lightning.readInterruptReg();
        if (intVal == NOISE_INT) {
            g_noiseCount++;
            g_lastNoiseEvent = millis();
        }
        else if (intVal == LIGHTNING_INT) {
            // Lightning! Now how far away is it? Distance estimation takes into
            // account previously seen events. 
            g_lastDistance = lightning.distanceToStorm(); 

            // "Lightning Energy" and I do place into quotes intentionally, is a pure
            // number that does not have any physical meaning. 
            g_lastEnergy = lightning.lightningEnergy(); 

            g_lastEventTime = Time.timeStr();
            StaticJsonDocument<100> json;
            json["event"]["lightning"]["distance"] = g_lastDistance;
            json["event"]["lightning"]["timestamp"] = Time.local();
            char buffer[101];
            serializeJson(json, buffer);
            String msg(buffer);
            client.publish("weather/lightning", msg.trim(), 0);
        }
    }

    if (millis() > (g_lastReading + ONE_MINUTE)) {
        g_lastReading = millis();
        // Read values from the sensor
        g_tempc = static_cast<double>(sht1x.readTemperatureC() + CALIBRATE_C);
        g_tempf = static_cast<double>(sht1x.readTemperatureF() + CALIBRATE_F);
        g_humidity = static_cast<double>(sht1x.readHumidity());
            
        StaticJsonDocument<200> json;
        json["environment"]["celsius"] = g_tempc;
        json["environment"]["farenheit"] = g_tempf;
        json["environment"]["humidity"] = g_humidity;
        json["time"] = Time.now();

        char buffer[201];
        serializeJson(json, buffer);
        String msg(buffer);
        client.publish("weather/conditions", msg.trim(), 0);
    }
}
