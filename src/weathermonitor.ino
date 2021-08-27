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
#include <SHT30.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_VEML6070.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include <SparkFun_AS3935_Lightning_Detector_Arduino_Library.h>
#include <Adafruit_MQTT_SPARK.h>
#include <Adafruit_MQTT.h>

void mqttCallback(char* topic, byte* payload, unsigned int length);

#define APP_ID              108

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
#define DAY_RAIN_ADDR       (TOTAL_RAIN_ADDR + sizeof(uint32_t))

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
bool g_rainCleared;
bool g_hourlyRainCleared;
bool g_indoor;
bool g_lastCalibration;
bool g_published;
bool g_getLightningData;
float g_lux;
uint16_t g_uv;
int g_lastDistance;
long g_lastEnergy;
uint32_t g_rainTickToday;
uint32_t g_rainTickTotal;
uint32_t g_rainTickLastHour;
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
SHT30 sensor;
Adafruit_VEML6070 uv = Adafruit_VEML6070();
Adafruit_VEML7700 veml;

/************ Global State (you don't need to change this!) ******************/
TCPClient TheClient;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_SPARK aioClient(&TheClient, AIO_SERVER, AIO_SERVERPORT, g_mqttName.c_str(), AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/
Adafruit_MQTT_Publish g_lightningFeed = Adafruit_MQTT_Publish(&aioClient, AIO_USERNAME "/feeds/weather.lightning");
Adafruit_MQTT_Publish g_temperatureFeed = Adafruit_MQTT_Publish(&aioClient, AIO_USERNAME "/feeds/weather.temperature");
Adafruit_MQTT_Publish g_humidityFeed = Adafruit_MQTT_Publish(&aioClient, AIO_USERNAME "/feeds/weather.humidity");
Adafruit_MQTT_Publish g_rainFallFeed = Adafruit_MQTT_Publish(&aioClient, AIO_USERNAME "/feeds/weather.rainfall");
Adafruit_MQTT_Publish g_hourlyRainFallFeed = Adafruit_MQTT_Publish(&aioClient, AIO_USERNAME "/feeds/weather.hourlyrainfall");
Adafruit_MQTT_Publish g_totalRainFallFeed = Adafruit_MQTT_Publish(&aioClient, AIO_USERNAME "/feeds/weather.total-rainfall");
Adafruit_MQTT_Publish g_luxFeed = Adafruit_MQTT_Publish(&aioClient, AIO_USERNAME "/feeds/weather.lux");
Adafruit_MQTT_Publish g_uvFeed = Adafruit_MQTT_Publish(&aioClient, AIO_USERNAME "/feeds/weather.uvindex");

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
    memset(mqttBuffer, '\0', 512);
    JSONBufferWriter json(mqttBuffer, sizeof(mqttBuffer));
    json.beginObject();
    json.name("device");
        json.beginObject();
        json.name("AS3935");

    g_maskValue = lightning.readMaskDisturber();
    Serial.print("Are disturbers being masked: "); 
    if (g_maskValue == 1) {
        Serial.println("YES");
        json.name("disturbers").value("MASKED");
    } 
    else if (g_maskValue == 0) {
        Serial.println("NO"); 
        json.name("disturbers").value("UNMASKED");
    }

    int enviVal = lightning.readIndoorOutdoor();
    Serial.print("Are we set for indoor or outdoor: ");  
    if(enviVal == INDOOR) {
        Serial.println("Indoor.");
        g_indoor = true;
        json.name("indoor").value("TRUE");
    }  
    else if( enviVal == OUTDOOR ) {
        Serial.println("Outdoor.");
        g_indoor = false;
        json.name("indoor").value("FALSE");
    }  
    else { 
        Serial.println(enviVal, BIN);
        json.name("indoor").value("ERROR");
    }

    g_noiseFloor = lightning.readNoiseLevel();
    Serial.print("Noise Level is set at: ");
    Serial.println(g_noiseFloor);
    json.name("noisefloor").value(g_noiseFloor);
    json.endObject();
    json.name("time");
    json.beginObject();
        json.name("timezone").value(Time.zone());
        json.name("now").value(Time.timeStr());
    json.endObject();
    json.name("photon");
    json.beginObject();
        json.name("appid").value(g_appid);
        json.name("version").value(System.version());
        json.name("appid").value(g_appid);
    json.endObject();
    json.endObject();
        
    json.buffer()[std::min(json.bufferSize(), json.dataSize())] = 0;
    client.publish("weather/event/tunables", json.buffer());
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

    memset(mqttBuffer, '\0', 512);
    JSONBufferWriter json(mqttBuffer, sizeof(mqttBuffer));
    json.beginObject();
    json.name("disturbers").value(g_maskValue);
    json.name("timestamp").value(Time.timeStr());
    json.name("appid").value(g_appid);
    json.endObject();

    json.buffer()[std::min(json.bufferSize(), json.dataSize())] = 0;
    client.publish("weather/event/disturber", json.buffer());

    return g_maskValue;
}

int setSpikeRejectionValue(String v)
{
    int reject = v.toInt();
    lightning.spikeRejection(reject);
    delay(g_delay);
    g_spikeRejection = reject;

    memset(mqttBuffer, '\0', 512);
    JSONBufferWriter json(mqttBuffer, sizeof(mqttBuffer));
    json.beginObject();
    json.name("spikereject");
    json.beginObject();
        json.name("update").value(g_noiseFloor);
    json.endObject();
    json.name("timestamp").value(Time.timeStr());
    json.name("appid").value(g_appid);
    json.endObject();

    json.buffer()[std::min(json.bufferSize(), json.dataSize())] = 0;
    client.publish("weather/event/spikereject", json.buffer());

    return reject;
}

int setNoiseFloorValue(String v)
{
    int floor = v.toInt();
    lightning.setNoiseLevel(floor);
    delay(g_delay);
    g_noiseFloor = lightning.readNoiseLevel();

    memset(mqttBuffer, '\0', 512);
    JSONBufferWriter json(mqttBuffer, sizeof(mqttBuffer));
    json.beginObject();
    json.name("noisefloor");
    json.beginObject();
        json.name("update").value(g_noiseFloor);
        json.name("timestamp").value(Time.timeStr());
        json.name("appid").value(g_appid);
    json.endObject();
    json.endObject();

    json.buffer()[std::min(json.bufferSize(), json.dataSize())] = 0;
    client.publish("weather/event/noisefloor", json.buffer());

    return g_noiseFloor;
}

int setIndoor(String v)
{
    memset(mqttBuffer, '\0', 512);
    JSONBufferWriter json(mqttBuffer, sizeof(mqttBuffer));
    json.beginObject();
    json.name("location");
    json.beginObject();

    if (v == "0")
        lightning.setIndoorOutdoor(OUTDOOR);
    else
        lightning.setIndoorOutdoor(INDOOR);

    delay(g_delay);
    int enviVal = lightning.readIndoorOutdoor();
 
    if(enviVal == INDOOR) {
        json.name("update").value("indoor");
        g_indoor = true;
    }  
    else if( enviVal == OUTDOOR ) {
        json.name("update").value("outdoor");
        g_indoor = false;
    }  
    else 
        Serial.println(enviVal, BIN); 
    
    json.endObject();
    json.name("timestamp").value(Time.timeStr());
    json.name("appid").value(g_appid);
    json.endObject();

    json.buffer()[std::min(json.bufferSize(), json.dataSize())] = 0;
    client.publish("weather/event/indoor", json.buffer());

    return g_indoor;
}

void sendSystemData(bool force)
{
    if ((millis() > (g_lastSystemUpdate + FIVE_MINUTES)) || force) {
        memset(mqttBuffer, '\0', 512);
        JSONBufferWriter json(mqttBuffer, sizeof(mqttBuffer));
        json.beginObject();
            json.name("network");
            json.beginObject();
                json.name("ssid").value(WiFi.SSID());
                json.name("signalquality").value((int8_t)WiFi.RSSI());
            json.endObject();
            json.name("photon");
            json.beginObject();
                json.name("freemem").value(System.freeMemory());
                json.name("uptime").value(System.uptime());
                json.name("appid").value(g_appid);
                json.name("version").value(System.version());
            json.endObject();
            json.name("AS3935");
            json.beginObject();
                json.name("noisefloor").value(g_noiseFloor);
            json.endObject();
        json.endObject();
        json.buffer()[std::min(json.bufferSize(), json.dataSize())] = 0;
        client.publish("weather/event/system", json.buffer());
        g_lastSystemUpdate = millis();
    }
}

void readLightning()
{
    // We got notified, make sure we reset this so if a notification comes in again, we get it while in this function
    g_getLightningData = false;

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

        memset(mqttBuffer, '\0', 512);
        JSONBufferWriter json(mqttBuffer, sizeof(mqttBuffer));
        json.beginObject();
            json.name("appid").value(g_appid);
            json.name("timestamp").value(Time.local());
            json.name("lightning");
            json.beginObject();
                json.name("kilometers").value(g_lastDistance);
                json.name("miles").value(g_lastDistance * .621371);
            json.endObject();
        json.endObject();
        json.buffer()[std::min(json.bufferSize(), json.dataSize())] = 0;
        client.publish("weather/event/lightning", json.buffer());

        while (!aioClient.connected()) {
            Log.warn("Not connected, trying to connect...");
            if (aioClient.connectServer())
                break;

            delay(100);
        }
        g_lightningFeed.publish((g_lastDistance * .621371));
        aioClient.disconnectServer();
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
        if(sensor.update()) {
            g_tempc = sensor.humidity;
            g_tempf = g_tempc * 1.8 + 32;
            g_humidity = sensor.humidity;
            g_lux = veml.readLux();
            g_uv = uv.readUV();
        }
        else {
            Log.error("Unable to read from SHT20");
            return;
        }
        
        double tdpfc =  (g_tempc - (14.55 + 0.114 * g_tempc) * (1 - (0.01 * g_humidity)) - pow(((2.5 + 0.007 * g_tempc) * (1 - (0.01 * g_humidity))),3) - (15.9 + 0.117 * g_tempc) * pow((1 - (0.01 * g_humidity)), 14));
        double tdpff = tdpfc * 1.8 + 32;
        memset(mqttBuffer, '\0', 512);
        JSONBufferWriter json(mqttBuffer, sizeof(mqttBuffer));
        json.beginObject();
            json.name("appid").value(g_appid);
            json.name("timestamp").value(Time.local());
            json.name("environment");
            json.beginObject();
                json.name("celsius").value(g_tempc);
                json.name("farenheit").value(g_tempf);
                json.name("humidity").value(g_humidity);
                json.name("dewpointc").value(tdpfc);
                json.name("dewpointf").value(tdpff);
                json.name("hourlyrain").value(g_rainTickLastHour);
                json.name("railtoday").value(g_rainTickToday);
                json.name("raintotal").value(g_rainTickTotal);
                json.name("lux").value(g_lux);
                json.name("uv").value(g_uv);
            json.endObject();
        json.endObject();

        EEPROM.put(DAY_RAIN_ADDR, g_rainTickToday);
        EEPROM.put(TOTAL_RAIN_ADDR, g_rainTickTotal);

        json.buffer()[std::min(json.bufferSize(), json.dataSize())] = 0;
        client.publish("weather/conditions", json.buffer());

        if ((Time.minute() == 0 || Time.minute() == 30) && !g_published) {
            while (!aioClient.connected()) {
                Log.warn("Not connected, trying to connect...");
                if (aioClient.connectServer())
                    break;

                delay(100);
            }
            
            g_temperatureFeed.publish(g_tempf);
            g_humidityFeed.publish(g_humidity);
            g_uvFeed.publish(g_uv);
            g_luxFeed.publish(g_lux);
            g_published = true;
        }
        else {
            g_published = false;
        }
    }
}

void applicationSetup()
{
    memset(mqttBuffer, '\0', 512);
    JSONBufferWriter json(mqttBuffer, sizeof(mqttBuffer));
    json.beginObject();
        json.name("device");
        json.beginObject();
            json.name("name").value("AS3935");

            json.name("AS3935");
                json.beginObject();
                json.name("calibrated").value(g_lastCalibration);

    lightning.maskDisturber(true); 
    delay(g_delay);
    g_maskValue = lightning.readMaskDisturber();
    Serial.print("Are disturbers being masked: "); 
    if (g_maskValue == 1) {
        Serial.println("YES");
                json.name("disturbers").value("MASKED");
    } 
    else if (g_maskValue == 0) {
        Serial.println("NO"); 
                json.name("disturbers").value("UNMASKED");
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
                json.name("indoor").value("TRUE");
    }  
    else if( enviVal == OUTDOOR ) {
        Serial.println("Outdoor.");
        g_indoor = false;
                json.name("indoor").value("FALSE");
    }  
    else { 
        Serial.println(enviVal, BIN);
                json.name("indoor").value("ERROR");
    }

    // Noise floor setting from 1-7, one being the lowest. Default setting is
    // two. If you need to check the setting, the corresponding function for
    // reading the function follows.    
    lightning.setNoiseLevel(g_noiseFloor);  
    delay(g_delay);
    g_noiseFloor = lightning.readNoiseLevel();
    Serial.print("Noise Level is set at: ");
    Serial.println(g_noiseFloor);
            json.name("noisefloor").value(g_noiseFloor);

    // Watchdog threshold setting can be from 1-10, one being the lowest. Default setting is
    // two. If you need to check the setting, the corresponding function for
    // reading the function follows.    
    lightning.watchdogThreshold(g_watchDogValue); 
    delay(g_delay);
    g_watchDogValue = lightning.readWatchdogThreshold();
    Serial.print("Watchdog Threshold is set to: ");
    Serial.println(g_watchDogValue);
            json.name("watchdog").value(g_watchDogValue);
    
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
            json.name("spikereject").value(g_spikeRejection);

    // This setting will change when the lightning detector issues an interrupt.
    // For example you will only get an interrupt after five lightning strikes
    // instead of one. Default is one, and it takes settings of 1, 5, 9 and 16.   
    // Followed by its corresponding read function. Default is zero. 
    lightning.lightningThreshold(g_threshold); 
    delay(g_delay);
    g_threshold = lightning.readLightningThreshold();
    Serial.print("The number of strikes before interrupt is triggerd: ");
            json.name("threshold").value(g_threshold);
            json.endObject();   // AS3935
        json.endObject();       // device
        json.name("appid").value(g_appid);
        json.name("timestamp").value(Time.timeStr());
    json.endObject();

    Serial.println(g_threshold);
        
    json.buffer()[std::min(json.bufferSize(), json.dataSize())] = 0;
    client.publish("weather/event/setup", json.buffer());
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
        g_rainTickLastHour++;
        g_lastTickEvent = millis();
    }
}

int setTotalRainFall(String total)
{
    g_rainTickTotal = total.toInt();
    EEPROM.put(TOTAL_RAIN_ADDR, g_rainTickTotal);
    return g_rainTickTotal;
}

int setDayRainFall(String total)
{
    g_rainTickToday = total.toInt();
    EEPROM.put(DAY_RAIN_ADDR, g_rainTickToday);
    return g_rainTickToday;
}

void updateLightning()
{
    g_getLightningData = true;
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
    g_rainCleared = true;
    g_hourlyRainCleared = true;
    g_rainTickLastHour = 0;

    Particle.variable("appid", g_appid);

    Particle.variable("distance", g_lastDistance);
    Particle.variable("conn_mqtt", g_connected);
    Particle.variable("humidity", g_humidity);
    Particle.variable("farenheit", g_tempf);
    Particle.variable("calibrated", g_lastCalibration);
    Particle.variable("today", g_rainTickToday);
    Particle.variable("total", g_rainTickTotal);
    Particle.function("setmask", setMaskValue);
    Particle.function("setspike", setSpikeRejectionValue);
    Particle.function("setnoise", setNoiseFloorValue);
    Particle.function("setrain", setTotalRainFall);
    Particle.function("settoday", setDayRainFall);

    Serial.begin(115200);

    EEPROM.get(DAY_RAIN_ADDR, g_rainTickToday);
    if (g_rainTickToday == 0xFFFFFFFF)
        g_rainTickToday = 0;

    EEPROM.get(TOTAL_RAIN_ADDR, g_rainTickTotal);
    if (g_rainTickTotal == 0xFFFFFFFF)
        g_rainTickTotal = 0;

    pinMode(D7, OUTPUT);
    pinMode(TIPGUAGE_PIN, INPUT_PULLUP);

    delay(2000);        // Give our devices a bit of time to stabilize

    uv.begin(VEML6070_1_T);
    veml.begin();
    veml.setGain(VEML7700_GAIN_1);
    veml.setIntegrationTime(VEML7700_IT_800MS);

    attachInterrupt(TIPGUAGE_PIN, updateRainTick, CHANGE);
    attachInterrupt(INTERRUPT_PIN, updateLightning, RISING);
    g_getLightningData = false;

    memset(mqttBuffer, '\0', 512);
    JSONBufferWriter json(mqttBuffer, sizeof(mqttBuffer));
    json.beginObject();
    json.name("photon");
        json.beginObject();
        json.name("deviceid").value(System.deviceID());
        json.name("version").value(System.version());
        json.name("appid").value(g_appid);
        json.endObject();
    json.name("reset");
        json.beginObject();
        json.name("reason").value(System.resetReason());
        json.endObject();

    client.connect(g_mqttName.c_str());
    if (client.isConnected()) {
        g_connected = 1;
        digitalWrite(D7, LOW);
        client.subscribe("weather/request/tunables");
    }

    g_lastCalibration = startupLightningDetector();

    Time.zone(currentTimeZone());
    json.name("time");
        json.beginObject();
        json.name("timezone").value(Time.zone());
        json.name("timestamp").value(Time.timeStr());
        json.endObject();
    json.endObject();
    
    sensor.setAddress(0);

    json.buffer()[std::min(json.bufferSize(), json.dataSize())] = 0;
    client.publish("weather/event/startup", json.buffer());
    applicationSetup();
    sendSystemData(true);
}

void loop() 
{
    char mqttBuffer[512];

    if (Time.minute() == 0) {
        if (!g_rainCleared && Time.hour() == 0) {
            g_rainFallFeed.publish(g_rainTickToday * .01);
            g_rainTickToday = 0;
            g_rainCleared = true;
            EEPROM.put(DAY_RAIN_ADDR, g_rainTickToday);
        }
        if (!g_hourlyRainCleared) {
            g_hourlyRainFallFeed.publish(g_rainTickLastHour * .01);
            g_totalRainFallFeed.publish(g_rainTickTotal * .01);
            g_rainTickLastHour = 0;
            g_hourlyRainCleared = true;
        }
    }
    else {
        g_rainCleared = false;
        g_hourlyRainCleared = false;
    }

    if (System.uptime() == (FIVE_DAYS / 1000)) {
        EEPROM.put(DAY_RAIN_ADDR, g_rainTickToday);
        EEPROM.put(TOTAL_RAIN_ADDR, g_rainTickTotal);
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

            memset(mqttBuffer, '\0', 512);
            JSONBufferWriter json(mqttBuffer, sizeof(mqttBuffer));
            json.beginObject();
                json.name("noisefloor").value(g_noiseFloor);
                json.name("timestamp").value(Time.timeStr());
                json.name("appid").value(g_appid);
            json.endObject();

            json.buffer()[std::min(json.bufferSize(), json.dataSize())] = 0;
            client.publish("weather/event/noisefloor", json.buffer());
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
        if (g_noiseFloor >= 1) {
            g_noiseFloor--;
            lightning.setNoiseLevel(g_noiseFloor);
            
            memset(mqttBuffer, '\0', 512);
            JSONBufferWriter json(mqttBuffer, sizeof(mqttBuffer));
            json.beginObject();
                json.name("noisefloor").value(g_noiseFloor);
                json.name("timestamp").value(Time.timeStr());
                json.name("appid").value(g_appid);
            json.endObject();

            json.buffer()[std::min(json.bufferSize(), json.dataSize())] = 0;
            client.publish("weather/event/noisefloor", json.buffer());
        }
    }

    if (g_getLightningData) {
        readLightning();
    }

    readEnvironment();
    sendSystemData(false);
}
