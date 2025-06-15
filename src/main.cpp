/*********************************
 *
 *  INCLUDE LIBRARY
 *
 *********************************/
#include <Arduino.h>
#include <Wire.h>       // ไลบรารี่สำหรับการสื่อสารแบบ I2C
#include <BH1750FVI.h>  // ไลบรารี่เซนเซอร์วัดแสง BH1750FVI
#include <DHT.h>        // ไลบรารี่เซนเซอร์วัดอุณหภูมิและความชื้น DHT11
#include <WiFi.h>       // ไลบรารี่สำหรับการเชื่อมต่อ WiFi
#include <HTTPClient.h> // ไลบรารี่สำหรับส่งข้อมูล HTTP
#include <WiFiClientSecure.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>

/*********************************
 *
 *  FUNCTION DECLARATIONS
 *
 *********************************/

//  Read and Print Sensors
void readSensors();

//  Send current sensor values (temp, humi, light, tds) to server
void sendToCloud();

//  Update OLED Display
void updateOLED();

//  Tries to connect to wifi
void connectWiFi();

//  Read Value from tds sensor and convert to ppm
void readTDS();

//  Get Median of all numbers in list (used in readTDS)
int getMedianNum(int bArray[], int iFilterLen);

//  Get current hour:minute from internet
void getCurrentTime();

/*********************************
 *
 *  GLOBAL CONSTANTS
 *
 *********************************/
// กำหนดค่าคงที่และพินที่ใช้
#define DHTPIN 13        // กำหนดขา GPIO2 สำหรับเซนเซอร์ DHT11
#define DHTTYPE DHT11    // ระบุประเภทเซนเซอร์เป็น DHT11
#define TDS_PIN 34       // กำหนดขา Analog GPIO34 สำหรับเซนเซอร์ TDS
#define LIGHT_PIN 5      // กำหนดขา GPIO5 สำหรับควบคุมไฟ LED
#define PUMP_PIN 4       // กำหนดขา GPIO4 สำหรับควบคุมปั๊มน้ำ
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
#define VREF 5.0  // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point
#define LIGHT_MIN_THRESHOLD 15000
#define LIGHT_MAX_THRESHOLD 25000
#define PUMP_MIN_THRESHOLD 800
#define PUMP_MAX_THRESHOLD 900

// ตัวแปรเก็บค่าจากเซนเซอร์
float lux, temp, humi, hic;                            // ตัวแปรเก็บค่าความเข้มแสง, อุณหภูมิ, ความชื้น, ค่า TDS
unsigned long lastUpdate = 0;                          // ตัวแปรเก็บเวลาล่าสุดที่อัพเดทข้อมูล
unsigned long screenLastUpdate = 0;                    // ตัวแปรเก็บเวลาล่าสุดที่อัพเดทข้อมูล
const long screenUpdateInterval = 2000;                // กำหนดช่วงเวลาการอัพเดทหน้าจอ OLED ทุก 2 วินาที
const long cloudUpdateInterval = 30000;                // กำหนดช่วงเวลาการอัพเดทข้อมูลทุก 30 วินาที
static unsigned long analogSampleTimepoint = millis(); // กำหนดช่วงเวลาการอ่านค่า tds
static unsigned long printTimepoint = millis();        // กำหนดช่วงเวลาการแสดงค่า tds ทาง Serial Monitor
int analogBuffer[SCOUNT];                              // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0;

//  Value of current hour and minute of the day
int hour = 0;
int minute = 0;

// ประกาศออบเจ็กต์สำหรับเซนเซอร์และอุปกรณ์
BH1750FVI lightMeter(BH1750FVI::k_DevModeContHighRes);                    // ตั้งค่าเซนเซอร์แสงในโหมด High Resolution
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // ตั้งค่าจอ OLED
DHT dht(DHTPIN, DHTTYPE);                                                 // ประกาศออบเจ็กต์เซนเซอร์ DHT11 (Humidity)

// ตั้งค่าการเชื่อมต่อ WiFi
const char *ssid = "POCO F6";      // กำหนดชื่อ WiFi
const char *password = "iamgroot"; // กำหนดรหัส WiFi

/*********************************
 *
 *  MAIN FUNCTION
 *
 *********************************/
void setup()
{
  Serial.begin(9600); // เริ่มการสื่อสาร Serial ที่ความเร็ว 9600 bps

  //  Setup OLED Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  // เริ่มต้นเซนเซอร์และอุปกรณ์
  Wire.begin();       // เริ่มการสื่อสารแบบ I2C
  lightMeter.begin(); // เริ่มการทำงานเซนเซอร์แสง
  dht.begin();        // เริ่มการทำงานเซนเซอร์ DHT11

  // ตั้งโหมดการทำงานของพิน
  pinMode(TDS_PIN, INPUT);    // ตั้งขา TDS เป็น Input
  pinMode(LIGHT_PIN, OUTPUT); // ตั้งขาไฟ LED เป็น Output
  pinMode(PUMP_PIN, OUTPUT);  // ตั้งขาปั๊มน้ำเป็น Output

  // Clear display
  display.display();
  display.clearDisplay();

  connectWiFi(); // เรียกฟังก์ชันเชื่อมต่อ WiFi

  getCurrentTime(); // อัพเดทเวลาปัจจุบัน

  delay(2000); // หน่วงเวลา 2 วินาที
}

void loop()
{
  // อ่านค่าจากเซนเซอร์
  readSensors();

  // แสดงผลบน OLED
  updateOLED();

  // Every 30 sec
  if (millis() - lastUpdate > cloudUpdateInterval)
  {
    // ส่งข้อมูลขึ้นคลาวด์ตามช่วงเวลาที่กำหนด
    sendToCloud();

    // อัพเดทเวลาปัจจุบัน
    getCurrentTime();
  }
}

/*********************************
 *
 *  FUNCTION DEFINITIONS
 *
 *********************************/

void readSensors()
{
  // อ่านค่า TDS
  readTDS();

  // อ่านค่าความเข้มแสงจากเซนเซอร์ BH1750FVI
  lux = lightMeter.GetLightIntensity();

  // เปิดไฟเมื่อค่า lux < 15000 และอยู่ในช่วงเวลา 05:00 - 18.00
  if (lux < LIGHT_MIN_THRESHOLD && hour < 18 && hour > 5)
  {
    digitalWrite(LIGHT_PIN, HIGH);
  }
  // ปิดไฟเมื่ออยู่ในช่วงเวลา 18:00 - 05.00
  else if (hour > 18 || hour < 5)
  {
    digitalWrite(LIGHT_PIN, LOW);
  }
  // ปิดไฟเมื่อค่า lux > 25000
  else if (lux > LIGHT_MAX_THRESHOLD)
  {
    digitalWrite(LIGHT_PIN, LOW);
  }
  // Serial.print("Light: ");
  // Serial.println(lux);

  // อ่านค่าอุณหภูมิและความชื้นจากเซนเซอร์ DHT11
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  humi = dht.readHumidity();
  // Read temperature as Celsius (the default)
  temp = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(humi) || isnan(temp))
  {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Celsius (isFahreheit = false)
  hic = dht.computeHeatIndex(temp, humi, false);

  // Serial.print(F("Humidity: "));
  // Serial.print(humi);
  // Serial.print(F("%  Temperature: "));
  // Serial.print(temp);
  // Serial.print(F("°C  Heat index: "));
  // Serial.print(hic);
  // Serial.print(F("°C "));
}

void updateOLED()
{
  if (millis() - screenLastUpdate >= screenUpdateInterval)
  {
    screenLastUpdate = millis();

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(F("Humidity:"));
    display.setCursor(64, 0);
    display.print(humi);
    display.setCursor(100, 0);
    display.print(F("%"));

    display.setCursor(0, 8);
    display.print(F("Light:"));
    display.setCursor(64, 8);
    display.print(lux);
    display.setCursor(100, 8);
    display.print(F("Lux"));

    display.setCursor(0, 16);
    display.print(F("Temp:"));
    display.setCursor(64, 16);
    display.print(temp);
    display.setCursor(100, 16);
    display.print(F("°C"));

    display.setCursor(0, 24);
    display.print(F("Tds:"));
    display.setCursor(64, 24);
    display.print(tdsValue);
    display.setCursor(100, 24);
    display.print(F("ppm"));

    display.display();
    delay(100);
  }
}

void connectWiFi()
{
  WiFi.begin(ssid, password); // เริ่มการเชื่อมต่อ WiFi

  Serial.print("Connecting WiFi...");

  // รอการเชื่อมต่อ WiFi
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print("."); // แสดงจุดบน Serial Monitor ขณะรอเชื่อมต่อ
  }
}

void getCurrentTime()
{

  //  Set last update time
  lastUpdate = millis();

  // ตรวจสอบการเชื่อมต่อ WiFi ก่อนส่งข้อมูล
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("Connect date time server");
    HTTPClient http;
    WiFiClientSecure client;

    client.setInsecure();

    // สร้าง URL สำหรับรับค่า วัน เวลา จาก server
    String url = "https://timeapi.io/api/time/current/zone?timeZone=Asia%2FBangkok";

    // Send HTTP GET request
    http.begin(client, url);

    int httpResponseCode = http.GET();

    if (httpResponseCode >= 0)
    {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      String payload = http.getString();
      Serial.println(payload);

      //  Convert response to json object
      JsonDocument doc;
      deserializeJson(doc, payload);

      //  Get current hour and minutes
      hour = doc["hour"];
      minute = doc["minute"];

      Serial.println(hour);
      Serial.println(minute);
    }
    else
    {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }

    http.end(); // ปิดการเชื่อมต่อ HTTP
  }
}

void sendToCloud()
{

  //  Set last update time
  lastUpdate = millis();

  // ตรวจสอบการเชื่อมต่อ WiFi ก่อนส่งข้อมูล
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("Connect database server");
    HTTPClient http; // ประกาศออบเจ็กต์ HTTPClient
    WiFiClientSecure client;

    client.setInsecure();

    // สร้าง URL สำหรับส่งข้อมูลไปยัง Server (humi, lux, temp, tds)
    String url = "https://hydroponic-ject.onrender.com/" + String(lux) + "/" + String(temp) + "/" + String(humi) + "/" + String(tdsValue);
    http.begin(client, url); // เริ่มการเชื่อมต่อ HTTP

    // Send HTTP GET request
    int httpResponseCode = http.GET();

    // Show error or success
    if (httpResponseCode >= 0)
    {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
    }
    else
    {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }

    http.end(); // ปิดการเชื่อมต่อ HTTP
  }
}

//  Read data from TDS Sensor
void readTDS()
{
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) // every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TDS_PIN); // read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) // every 800 milliseconds, calculate tds value
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;                                                                                                  // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temp - 25.0);                                                                                                                      // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient;                                                                                                            // temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; // convert voltage value to tds value

    Serial.print("TDS Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");

    // เปิดปั้มเมื่อค่า tds < 800
    if (tdsValue < PUMP_MIN_THRESHOLD)
    {
      digitalWrite(PUMP_PIN, HIGH);
    }
    // ปิดปั้มเมื่อค่า tds > 900
    else if (tdsValue > PUMP_MAX_THRESHOLD)
    {
      digitalWrite(PUMP_PIN, LOW);
    }
  }
}

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}
