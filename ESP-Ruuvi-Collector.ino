// Gateway running on an esp32 for BTLE Ruuvi tags to influx database

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <WiFi.h>
#include <esp_system.h>
#include "InfluxArduino.hpp"

const int wdtTimeout = 30000;  // time in ms to trigger the watchdog
                               // The watchdog should mainly trigger, if the wifi is not connecting and, if the ruuvi process gets stuck
hw_timer_t *timer = NULL;

void IRAM_ATTR resetModule() {
  ets_printf("reboot\n");
  esp_restart();
}

InfluxArduino influx;

const char WIFI_NAME[] = "YOUR_WIFI_SSID";
const char WIFI_PASS[] = "YOUR_PASSWORD";

const char INFLUX_DATABASE[] = "ruuvi_esp32";  // influx database name
const char INFLUX_IP[] = "192.168.1.60"; // influx server ip
const char INFLUX_USER[] = "ruuvi"; // influx username
const char INFLUX_PASS[] = "ruuvi"; // influx password

const char INFLUX_MEASUREMENT[] = "ruuvi"; // influx measurment name
const char INFLUX_MEASUREMENT_FREEZER[] = "ruuviFreezer";
const char INFLUX_MEASUREMENT_BATH[] = "ruuviBath";
const char INFLUX_MEASUREMENT_RUUVI[] = "ruuviRuuvi";

char formatStringFive[] = "temperature=%0.3f,pressure=%i,humidity=%0.3f,accel_x=%i,accel_y=%i,accel_z=%i,milivolts=%i"; // this is for digital format v5 from ruuvi tags

// a bit of waste of memory but it was easier to debug for me
char formatStringThree_freezer[] = "temperature=%0.3f,pressure=%i,humidity=%i,milivolts=%i";
char formatStringThree_bath[] = "temperature=%0.3f,pressure=%i,humidity=%i,milivolts=%i";
char formatStringThree_ruuvi[] = "temperature=%0.3f,pressure=%i,humidity=%i,milivolts=%i";

// 16 bit
short getShort(char* data, int index)
{
  return (short)((data[index] << 8) + (data[index + 1]));
}

short getShortone(char* data, int index)
{
  return (short)((data[index]));
}

// 16 bit
unsigned short getUShort(char* data, int index)
{
  return (unsigned short)((data[index] << 8) + (data[index + 1]));
}

unsigned short getUShortone(char* data, int index)
{
  return (unsigned short)((data[index]));
}

  void DecodeV5(char* data)
  {
  timerWrite(timer, 0); //reset timer (feed watchdog)
  digitalWrite(22, LOW);
  short tempRaw = getShort(data, 3);
  double temperature = (double)tempRaw * 0.005;
  unsigned short humRaw = getUShort(data, 5);
  double humidity = (double)humRaw * 0.0025;
  unsigned int pressure = (getUShort(data, 7) + 50000);
  short accelX = getShort(data, 9);
  short accelY = getShort(data, 11);
  short accelZ = getShort(data, 13);

  unsigned short voltRaw = data[15] << 3 | data[16] >> 5;
  unsigned char tPowRaw = data[16] && 0x1F;
  unsigned short voltage = voltRaw + 1600;
  char power = tPowRaw* 2 - 40;

  char fields[256];
  sprintf(fields, formatStringFive, temperature, pressure, humidity, accelX, accelY, accelZ, voltage);
  bool writeSuccessful = influx.write(INFLUX_MEASUREMENT, "device=esp", fields);
  if (!writeSuccessful)
  {
    Serial.print("error: ");
    Serial.println(influx.getResponse());

  }
}

// every ruuvi gets its own decode function. It would be much better, to just use a function input parameter to select the right output to influx.

void DecodeV3_freezer(char* data)
{
  timerWrite(timer, 0); //reset timer (feed watchdog)
  digitalWrite(22, HIGH);
  short tempRaw = getUShortone(data, 4) & 0b01111111;
  short tempRawsign = getUShortone(data, 4) & 0b10000000;
  short tempRawdec = getUShortone(data, 5);
  double temperature = (double)tempRaw + (double)tempRawdec / 100;
  if (tempRawsign==128){
  temperature = temperature * -1;
  }
  byte humRaw = getUShortone(data, 3);
  short humidity = humRaw / 2;
  unsigned int pressure = (getUShort(data, 6) + 50000);
  unsigned int voltageraw = getUShort(data, 14);
  short voltage = (short)voltageraw;

  char fields[256];
  sprintf(fields, formatStringThree_freezer, temperature, pressure, humidity, voltage);
  //char mDevice[] = advertisedDevice.getAddress().toString().c_str();
  //bool writeSuccessful = influx.write(INFLUX_MEASUREMENT, ("device=%s",mDevice), fields);
  bool writeSuccessful = influx.write(INFLUX_MEASUREMENT_FREEZER, "device=freezer", fields);
  if (!writeSuccessful)
  {
    Serial.print("error: ");
    Serial.println(influx.getResponse());
  }
  timerWrite(timer, 0); //reset timer (feed watchdog)
}

void DecodeV3_bath(char* data)
{
  timerWrite(timer, 0); //reset timer (feed watchdog)
  digitalWrite(22, HIGH);
  short tempRaw = getUShortone(data, 4) & 0b01111111;
  short tempRawsign = getUShortone(data, 4) & 0b10000000;
  short tempRawdec = getUShortone(data, 5);
  double temperature = (double)tempRaw + (double)tempRawdec / 100;
  if (tempRawsign==128){
  temperature = temperature * -1;
  }
  byte humRaw = getUShortone(data, 3);
  short humidity = humRaw / 2;
  unsigned int pressure = (getUShort(data, 6) + 50000);
  unsigned int voltageraw = getUShort(data, 14);
  short voltage = (short)voltageraw;

  char fields[256];
  sprintf(fields, formatStringThree_bath, temperature, pressure, humidity, voltage);
  //char mDevice[] = advertisedDevice.getAddress().toString().c_str();
  //bool writeSuccessful = influx.write(INFLUX_MEASUREMENT, ("device=%s",mDevice), fields);
  bool writeSuccessful = influx.write(INFLUX_MEASUREMENT_BATH, "device=bath", fields);
  if (!writeSuccessful)
  {
    Serial.print("error: ");
    Serial.println(influx.getResponse());
  }
  timerWrite(timer, 0); //reset timer (feed watchdog)
}


void DecodeV3_ruuvi(char* data)
{
  timerWrite(timer, 0); //reset timer (feed watchdog)
  digitalWrite(22, HIGH);
  short tempRaw = getUShortone(data, 4) & 0b01111111;
  short tempRawsign = getUShortone(data, 4) & 0b10000000;
  short tempRawdec = getUShortone(data, 5);
  double temperature = (double)tempRaw + (double)tempRawdec / 100;
  if (tempRawsign==128){
  temperature = temperature * -1;
  }
  byte humRaw = getUShortone(data, 3);
  short humidity = humRaw / 2;
  unsigned int pressure = (getUShort(data, 6) + 50000);
  unsigned int voltageraw = getUShort(data, 14);
  short voltage = (short)voltageraw;

  char fields[256];
  sprintf(fields, formatStringThree_ruuvi, temperature, pressure, humidity, voltage);
  //char mDevice[] = advertisedDevice.getAddress().toString().c_str();
  //bool writeSuccessful = influx.write(INFLUX_MEASUREMENT, ("device=%s",mDevice), fields);
  bool writeSuccessful = influx.write(INFLUX_MEASUREMENT_RUUVI, "device=ruuvi", fields);
  if (!writeSuccessful)
  {
    Serial.print("error: ");
    Serial.println(influx.getResponse());
  }
  timerWrite(timer, 0); //reset timer (feed watchdog)
}


const char* mDevice_char;
String mDevice;
char* mData_byte;
char* mData;
uint8_t blocked = 0;

class AdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
           
      mData_byte = (char*)advertisedDevice.getManufacturerData().data(); // get data of every broadcasting BT device, not only ruuvi tags
      mData = strdup(mData_byte); //copy the pointer content (mData_byte) to another pointer (mData) as an update in the background would mess up the data
      mDevice_char = (advertisedDevice.toString().c_str()); // get MAC or ID of the device
      String mDevice = mDevice_char; //copy this pointer as well
      

 	    // process only ruuvi tags with v3 data protocol further (mData is MSL coded)
      if (mData[0] == 0x99 && mData[1] == 0x04 && mData[2] == 0x03)
      { 
          if (mDevice.indexOf("c5:0e:d3:ce:b4:07") > 0) // search inside mDevice string for ID of ruuvi tag
          {
              DecodeV3_freezer(mData);
              //Serial.println("found freezer");
              //Serial.println(mDevice);
          }
          
          if (mDevice.indexOf("d9:fb:b9:22:bc:f3") > 0) // search inside mDevice string for ID of ruuvi tag
          {
              DecodeV3_bath(mData);
              //Serial.println("found bath");
              //Serial.println(mDevice);              
          }

          if (mDevice.indexOf("d0:2b:fd:a2:9b:19") > 0) // search inside mDevice string for ID of ruuvi tag
          {
              DecodeV3_ruuvi(mData);
              //Serial.println("found ruuvi");
              //Serial.println(mDevice);
          }
      } 

     timerWrite(timer, 0); //reset timer (feed watchdog)
   }
};

BLEScan* pBLEScan;

void setup() {
  
  //WDT Stuff
  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timer);                          //enable interrupt

  pinMode(22, OUTPUT);
  digitalWrite(22, HIGH);
  Serial.begin(115200);

  WiFi.begin(WIFI_NAME, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    digitalWrite(22, LOW);
    Serial.print(".");
    digitalWrite(22, HIGH);
    //timerWrite(timer, 0); // would reset the timer of the watchdog but, if no wifi is found, the device should reboot here
  }
  Serial.println("WiFi connected!");
  influx.configure(INFLUX_DATABASE, INFLUX_IP);
  influx.authorize(INFLUX_USER, INFLUX_PASS);

  //log_i("BLEDevice::init()");
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  //pBLEScan->setInterval(200);
  //pBLEScan->setWindow(40);
}

//long loopTimeMax=0;

void loop() {
  timerWrite(timer, 0); //reset timer (feed watchdog)
  BLEScanResults foundDevices = pBLEScan->start(1);
  digitalWrite(22, LOW);
  /* // checking for loop time to debug watachdog issues
  long loopTime = millis();
    loopTime = millis() - loopTime;
  if (loopTimeMax < loopTime) loopTimeMax=loopTime;
  Serial.print("current loop time (ms): ");
  Serial.print(loopTime); //should be under 20000
  Serial.print(" / MaxTime: ");
  Serial.println(loopTimeMax); //should be under 20000
  */
}
