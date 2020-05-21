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

#include <cmath>

#include <SPI.h>
#include <Wire.h>
#include <SHT1x.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include <SparkFun_AS3935_Lightning_Detector_Arduino_Library.h>
#include <Adafruit_MQTT_SPARK.h>
#include <Adafruit_MQTT.h>

void mqttCallback(char* topic, byte* payload, unsigned int length);

#define APP_ID              99

#define dataPin             D2         // Yellow       // Brown is power, black is ground
#define clockPin            D3         // Blue
#define TIPGUAGE_PIN        D6          // Rain Tipguage (might use D4)
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
#define FIVE_MINUTES        (5 * ONE_MINUTE)
#define FIFTEEN_MINUTES     (15 * ONE_MINUTE)
#define ONE_HOUR            (60 * ONE_MINUTE)
#define SIX_HOURS           (6 * ONE_HOUR)
#define ONE_DAY             (24 * ONE_HOUR)
#define FIVE_DAYS           (5 * ONE_DAY)
#define CST_OFFSET          -6
#define DST_OFFSET          (CST_OFFSET + 1)
#define TIME_BASE_YEAR		2019
#define AIO_SERVER          "io.adafruit.com"
#define AIO_SERVERPORT      1883                   // use 8883 for SSL
#define AIO_USERNAME        ""
#define AIO_KEY             ""
#define TOTAL_RAIN_ADDR     4
#define DAY_RAIN_ADDR       8

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
int g_envCount;
bool g_rainNotCleared;
bool g_indoor;
bool g_lastCalibration;
bool g_published;
int g_lastDistance;
long g_lastEnergy;
uint16_t g_rainTickToday;
uint16_t g_rainTickTotal;
int g_tuneValue;
int g_noiseFloor;
int g_recentUploadCount;
byte g_watchDogValue;
system_tick_t g_lastSystemUpdate;
system_tick_t g_lastTimeFix;
system_tick_t g_lastMillis;
system_tick_t g_lastLoop;
system_tick_t g_lastNoiseEvent;
system_tick_t g_lastReading;
system_tick_t g_rateLimit;
system_tick_t g_lastTickEvent;
String g_name = "weathermonitor-";
String g_mqttName = g_name + System.deviceID().substring(0, 8);
String g_tunablesMessage = "weather/request/tunables";
byte mqttServer[] = {172, 24, 1, 13};
MQTT client(mqttServer, 1883, mqttCallback);
char mqttBuffer[512];

const uint8_t _usDSTStart[22] = { 10, 8,14,13,12,10, 9, 8,14,12,11,10, 9,14,13,12,11, 9};
const uint8_t _usDSTEnd[22]   = { 3, 1, 7, 6, 5, 3, 2, 1, 7, 5, 4, 3, 2, 7, 6, 5, 4, 2};

// SPI
SparkFun_AS3935 lightning;
SHT1x sht1x(dataPin, clockPin);

/************ Global State (you don't need to change this!) ******************/
TCPClient TheClient;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_SPARK aioClient(&TheClient, AIO_SERVER, AIO_SERVERPORT, g_mqttName.c_str(), AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/
Adafruit_MQTT_Publish g_lightningFeed = Adafruit_MQTT_Publish(&aioClient, AIO_USERNAME "/feeds/weather.lightning");
Adafruit_MQTT_Publish g_temperatureFeed = Adafruit_MQTT_Publish(&aioClient, AIO_USERNAME "/feeds/weather.temperature");
Adafruit_MQTT_Publish g_humidityFeed = Adafruit_MQTT_Publish(&aioClient, AIO_USERNAME "/feeds/weather.humidity");

ApplicationWatchdog wd(60000, System.reset);

STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));

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

    json["time"]["timezone"] = Time.zone();
    json["time"]["now"] = Time.local();
    json["photon"]["id"] = System.deviceID();
    json["photon"]["version"] = System.version();
    json["photon"]["appid"] = g_appid;
        
    memset(mqttBuffer, '\0', 512);
    serializeJson(json, mqttBuffer);

    client.publish("weather/event/tunables", mqttBuffer);
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
    json["disturbers"]["update"] = g_maskValue;
    json["disturbers"]["timestamp"] = Time.local();
    json["noisefloor"]["appid"] = g_appid;

    memset(mqttBuffer, '\0', 512);
    serializeJson(json, mqttBuffer);
    client.publish("weather/event/disturber", mqttBuffer);

    return g_maskValue;
}

int setSpikeRejectionValue(String v)
{
    int reject = v.toInt();
    lightning.spikeRejection(reject);
    delay(g_delay);
    g_spikeRejection = reject;

    StaticJsonDocument<200> json;
    json["spikereject"]["update"] = g_noiseFloor;
    json["spikereject"]["timestamp"] = Time.local();
    json["noisefloor"]["appid"] = g_appid;

    memset(mqttBuffer, '\0', 512);
    serializeJson(json, mqttBuffer);
    client.publish("weather/event/spikereject", mqttBuffer);

    return reject;
}

int setNoiseFloorValue(String v)
{
    int floor = v.toInt();
    lightning.setNoiseLevel(floor);
    delay(g_delay);
    g_noiseFloor = lightning.readNoiseLevel();

    StaticJsonDocument<200> json;
    json["noisefloor"]["update"] = g_noiseFloor;
    json["noisefloor"]["timestamp"] = Time.local();
    json["noisefloor"]["appid"] = g_appid;

    memset(mqttBuffer, '\0', 512);
    serializeJson(json, mqttBuffer);
    client.publish("weather/event/noisefloor", mqttBuffer);

    return g_noiseFloor;
}

int setIndoor(String v)
{
    StaticJsonDocument<200> json;

    if (v == "0")
        lightning.setIndoorOutdoor(OUTDOOR);
    else
        lightning.setIndoorOutdoor(INDOOR);

    delay(g_delay);
    int enviVal = lightning.readIndoorOutdoor();
 
    if(enviVal == INDOOR) {
        json["indoor"]["update"] = "indoor";
        g_indoor = true;
    }  
    else if( enviVal == OUTDOOR ) {
        json["indoor"]["update"] = "outdoor";
        g_indoor = false;
    }  
    else 
        Serial.println(enviVal, BIN); 
    
    json["indoor"]["timestamp"] = Time.local();
    json["noisefloor"]["appid"] = g_appid;

    memset(mqttBuffer, '\0', 512);
    serializeJson(json, mqttBuffer);
    client.publish("weather/event/indoor", mqttBuffer);

    return g_indoor;
}

void sendSystemData(bool force)
{
    if ((millis() > (g_lastSystemUpdate + FIVE_MINUTES)) || force) {
        StaticJsonDocument<250> json;
        json["network"]["ssid"] = WiFi.SSID();
        json["network"]["signalquality"] = (int8_t) WiFi.RSSI();
        json["photon"]["freemem"] = System.freeMemory();
        json["photon"]["uptime"] = System.uptime();
        json["photon"]["appid"] = g_appid;
        json["photon"]["version"] = System.version();
        json["device"]["noisefloor"] = g_noiseFloor;

        memset(mqttBuffer, '\0', 512);
        serializeJson(json, mqttBuffer);
        client.publish("weather/event/system", mqttBuffer);
        g_lastSystemUpdate = millis();
    }
}

void readLightning()
{
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

        StaticJsonDocument<100> json;
        json["lightning"]["kilometers"] = g_lastDistance;
        json["lightning"]["miles"] = g_lastDistance * .621371;
        json["lightning"]["timestamp"] = Time.local();
        json["lightning"]["appid"] = g_appid;

        memset(mqttBuffer, '\0', 512);
        serializeJson(json, mqttBuffer);
        client.publish("weather/event/lightning", mqttBuffer);

        g_lightningFeed.publish((g_lastDistance * .621371));
    }
}

/**
 * Broadcast the environment every minute
 * and every 30, send it to AIO
 */
void readEnvironment()
{
    if (millis() > (g_lastReading + ONE_MINUTE)) {
        g_lastReading = millis();
        // Read values from the sensor
        g_tempc = static_cast<double>(sht1x.readTemperatureC() + CALIBRATE_C);
        g_tempf = static_cast<double>(sht1x.readTemperatureF() + CALIBRATE_F);
        g_humidity = static_cast<double>(sht1x.readHumidity());
        
        //Tdf= ((((Tf-32)/1.8)-(14.55+0.114*((Tf-32)/1.8))*(1-(0.01*RH))-((2.5+0.007*((Tf-32)/1.8))*(1-(0.01*RH)))^3-(15.9+0.117*((Tf-32)/1.8))*(1-(0.01*RH))^14)*1.8)+32
        double tdpfc =  (g_tempc - (14.55 + 0.114 * g_tempc) * (1 - (0.01 * g_humidity)) - pow(((2.5 + 0.007 * g_tempc) * (1 - (0.01 * g_humidity))),3) - (15.9 + 0.117 * g_tempc) * pow((1 - (0.01 * g_humidity)), 14));
        double tdpff = tdpfc * 1.8 + 32;
        StaticJsonDocument<200> json;
        json["environment"]["celsius"] = g_tempc;
        json["environment"]["farenheit"] = g_tempf;
        json["environment"]["humidity"] = g_humidity;
        json["environment"]["dewpointc"] = tdpfc;
        json["environment"]["dewpointf"] = tdpff;
        json["environment"]["raintoday"] = g_rainTickToday;
        json["environment"]["raintotal"] = g_rainTickTotal;
        json["appid"] = g_appid;
        json["time"] = Time.now();

        EEPROM.put(DAY_RAIN_ADDR, g_rainTickToday);
        EEPROM.put(TOTAL_RAIN_ADDR, g_rainTickTotal);

        if ((Time.minute() == 0 || Time.minute() == 30) && !g_published) {
            g_temperatureFeed.publish(g_tempf);
            g_humidityFeed.publish(g_humidity);
            g_published = true;
        }
        else {
            g_published = false;
        }

        memset(mqttBuffer, '\0', 512);
        serializeJson(json, mqttBuffer);
        client.publish("weather/conditions", mqttBuffer);
    }
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
    if (g_indoor)
        lightning.setIndoorOutdoor(INDOOR);
    else
        lightning.setIndoorOutdoor(OUTDOOR);

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
        
    memset(mqttBuffer, '\0', 512);
    serializeJson(json, mqttBuffer);
    client.publish("weather/event/setup", mqttBuffer);
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

void handleDeviceRequest(byte *payload, unsigned int length)
{
    StaticJsonDocument<256> doc;
    deserializeJson(doc, payload, length);
    
    if (!doc.isNull()) {
        if (doc["device"]["id"] == System.deviceID()) {
            if (doc["device"]["command"] == "reboot") {
                System.reset();
            }
        }
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) 
{
    Serial.print("Got message on topic ");
    Serial.println(topic);

    if (strcmp(topic, "weather/request/tunables") == 0) {
        returnTunables();
    }
    else if (strcmp(topic, "weather/request/status") == 0) {
        sendSystemData(true);
    }
    else if (strcmp(topic, "device/request") == 0) {
        handleDeviceRequest(payload, length);
    }
    else {
        return;
    }
}

void updateRainTick()
{
    if ((g_lastTickEvent + 250) <= millis()) {
        g_rainTickToday++;
        g_rainTickTotal++;
        g_lastTickEvent = millis();
    }
}

void setup()
{
    g_appid = APP_ID;
    g_count = 0;
    g_noiseCount = 0;
    g_noiseFloor = 0;
    g_spikeRejection = 8;
    g_threshold = 5;
    g_watchDogValue = 2;
    g_indoor = false;
    g_lastMillis = 0;
    g_connected = 0;
    g_lastLoop = 0;
    g_lastDistance = 0;
    g_tuneValue = 0;
    g_lastNoiseEvent = 0;
    g_lastReading = 0;
    g_watchDogValue = 2;
    g_delay = 10;
    g_lastCalibration = false;
    g_lastSystemUpdate = millis();
    g_envCount = 0;
    g_published = false;
    g_rainTickTotal = 0;
    g_rainTickToday = 0;
    g_lastTickEvent = 0;
    g_rainNotCleared = false;

    Particle.variable("appid", g_appid);
    Particle.variable("antennafreq", g_tuneValue);
    Particle.variable("noisefloor", g_noiseFloor);
    Particle.variable("distance", g_lastDistance);
    Particle.variable("conn_mqtt", g_connected);
    Particle.variable("humidity", g_humidity);
    Particle.variable("farenheit", g_tempf);
    Particle.variable("calibrated", g_lastCalibration);
    Particle.variable("ticks", g_rainTickToday);
    Particle.function("setmask", setMaskValue);
    Particle.function("setspike", setSpikeRejectionValue);
    Particle.function("setnoise", setNoiseFloorValue);

    Serial.begin(115200);

    EEPROM.get(DAY_RAIN_ADDR, g_rainTickToday);
    if (g_rainTickToday == 0xFFFF)
        g_rainTickToday = 0;

    EEPROM.get(TOTAL_RAIN_ADDR, g_rainTickTotal);
    if (g_rainTickTotal == 0xFFFF)
        g_rainTickTotal = 0;

    pinMode(D7, OUTPUT);
    pinMode(TIPGUAGE_PIN, INPUT_PULLUP);

    delay(2000);        // Give our devices a bit of time to stabilize

    attachInterrupt(TIPGUAGE_PIN, updateRainTick, CHANGE);

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
    if (g_lastCalibration) {
        json["device"]["AS3935"] = "calibrated";
    }
    else
    {
        json["device"]["AS3935"] = "notcalibrated";
    }
    
    memset(mqttBuffer, '\0', 512);
    serializeJson(json, mqttBuffer);
    client.publish("weather/event/startup", mqttBuffer);
    applicationSetup();
    sendSystemData(true);
}

void loop() 
{
    char mqttBuffer[512];

    if ((Time.hour() == 0) && (Time.minute() == 0)) {
        if (g_rainNotCleared) {
            g_rainTickToday = 0;
            g_rainNotCleared = false;
        }
    }
    else {
        g_rainNotCleared = true;
    }

    if (System.uptime() == (FIVE_DAYS / 1000)) {
        System.reset();
    }
    
    /*
     * If the number of noise events in a one minute period is more than max
     * bump the noise floor up one and see if it stabilizes
     */
    if (g_noiseCount >= NOISE_INTTERUPT_MAX_COUNT) {
        if (millis() <= (g_lastNoiseEvent + ONE_MINUTE)) {
            g_noiseCount = 0;
            lightning.setNoiseLevel(++g_noiseFloor);

            StaticJsonDocument<200> json;
            json["noisefloor"]["update"] = g_noiseFloor;
            json["noisefloor"]["timestamp"] = Time.local();
            json["photon"]["appid"] = g_appid;

            memset(mqttBuffer, '\0', 512);
            serializeJson(json, mqttBuffer);
            
            client.publish("weather/event/noisefloor", mqttBuffer);
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
        json["time"]["local"] = Time.local();
        json["time"]["timezone"] = currentTimeZone();
        json["photon"]["appid"] = g_appid;

        memset(mqttBuffer, '\0', 512);
        serializeJson(json, mqttBuffer);
        client.publish("weather/event/timestamp", mqttBuffer);
    }

    /*
     * Handle the MQTT loop
     */
    if (client.isConnected()) {
        if (millis() > (g_lastLoop + FIVE_SECONDS)) {
            client.loop();
            if (!aioClient.ping())
                aioClient.Update();
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
            json["noisefloor"]["update"] = g_noiseFloor;
            json["noisefloor"]["timestamp"] = Time.local();
            json["photon"]["appid"] = g_appid;

            memset(mqttBuffer, '\0', 512);
            serializeJson(json, mqttBuffer);
            client.publish("weather/event/noisefloor", mqttBuffer);
        }
    }

    if (digitalRead(INTERRUPT_PIN) == HIGH) {
        readLightning();
    }

    readEnvironment();
    sendSystemData(false);
}
