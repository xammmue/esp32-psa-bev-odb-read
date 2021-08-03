#define TINY_GSM_MODEM_SIM800
#include <Wire.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <BluetoothSerial.h>
#include "TinyGPS++.h"
#include <HardwareSerial.h>
#include "soc/rtc_wdt.h"

#define SerialMon Serial
#define SerialAT Serial1
#define TINY_GSM_DEBUG SerialMon
#define GSM_PIN ""
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

#define LOG_LEVEL debug


const bool DO_CELLULAR_OUTPUT = true;
const bool LOG_ODB_COMMANDS = false;
const bool LOG_MQTT_MESSAGES = true;
const int debug = 0;
const int info = 1;
const int warning = 2;
const int error = 3;

int lastActive;

const char apn[] = "";
const char gprsUser[] = "";
const char gprsPass[] = "";
const char simPIN[]   = "";

const char* broker = "mqttBrokerURL";
const char* mqttUsername = "mqttUsername";
const char* mqttPassword = "mqttPassword";

uint32_t lastReconnectAttempt = 0;

TwoWire I2CPower = TwoWire(0);

bool dataCollected = false;

String ambTemp;
String batTemp;
String carSpeed;
String batVolt;
String batVMin;
String batVMax;
String socCal;
String kwhAvai;
String soh;
String soc;
String batCurr;
int mqttODBResendInterval = 15;
int lastODBValuesSent = -mqttODBResendInterval;

float latitude = 0;
float longitude = 0;
bool gpsPublished = false;
bool gpsReady = false;
char lastLatString[8];
char lastLngString[8];

hw_timer_t * timer1 = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqttClient(client);
BluetoothSerial ODB;
TinyGPSPlus gps;

void ARDUINO_ISR_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  xSemaphoreGiveFromISR(timerSemaphore, NULL);

  if (gpsReady)
  {
    gpsPublished = false;
    gpsReady = false;
  }

  if (esp_timer_get_time() - lastActive > 19999900)
  {
    log("inactive for too long, rebooting", warning);
    ESP.restart();
  }

}

bool setPowerBoostKeepOn(int en) {
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37);
  } else {
    I2CPower.write(0x35);
  }
  return I2CPower.endTransmission() == 0;
}

void cellularSetup()
{
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);

  bool isOk = setPowerBoostKeepOn(1);
  log(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"), debug);

  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  log("Wait...", debug);

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  log("Initializing modem...", debug);
  modem.restart();

  String modemInfo = modem.getModemInfo();
  log("Modem Info: ", debug);
  log(modemInfo, debug);

  if ( GSM_PIN && modem.getSimStatus() != 3 ) {
    modem.simUnlock(GSM_PIN);
  }

  log("Connecting to APN: ", debug);
  log(apn, debug);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    log("GRPS Connection failed", error);
    ESP.restart();
  }
  else {
    log("Connection established", debug);
  }

  if (modem.isGprsConnected()) {
    log("GPRS connected", info);
  }
}

void mqttSetup()
{
  mqttClient.setServer(broker, 1883);
  mqttReconnect();
  sendMqtt("corsaE", "connected");
}


void mqttReconnect() {
  int countRetry = 0;
  while (!mqttClient.connected()) {
    log("Attempting MQTT connection...", debug);
    if (mqttClient.connect("esp32-psa-bev-odb-reader", mqttUsername, mqttPassword)) {
      log("MQTT connected", info);
    } else {
      logLine("failed, rc=", warning);
      log(String(mqttClient.state()), warning);
      log(" try again in 5 seconds", warning);
      delay(5000);
    }

    if (countRetry >= 5)
    {
      log("Failed too many times", error);
      log("resetting...", error);
      ESP.restart();
    }
    else
    {
      countRetry++;
    }
  }
}
void bluetoothSetup()
{
  log("starting bluetooth", debug);
  ODB.begin("ESP32-ODB-Reader", true);

  uint8_t address[6]  = {0xAB, 0x90, 0x78, 0x56, 0x34, 0x12};
  bool connected = false;
  do
  {
    log("Connecting to ODB...", debug);
    connected = ODB.connect(address);
  } while (!connected);

  log("Connected to ODB", debug);

  sendSetupCommands();
  log("bluetoot setup finished", info);
}
void sendSetupCommands()
{
  char *setupCommands[] = {"ATWS", "ATI", "ATE0", "AT@1", "ATSP6", "ATAT1", "ATL0", "ATS0", "ATH1", "ATCAF1", "ATSH79B", "ATFCSH79B", "ATFCSD300000", "ATFCSM1"};
  for (int i = 0; i < 14; i++)
  {
    sendOdbCommand(setupCommands[i]);
  }
  log("all commands sent", debug);
}

void timerSetup()
{
  sendMqtt("corsaE", "starting timers");
  timerSemaphore = xSemaphoreCreateBinary();
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &onTimer, true);
  timerAlarmWrite(timer1, 10000000, true);
  timerAlarmEnable(timer1);
}

void gpsSetup()
{
  log("setting up GPS", debug);
  Serial2.begin(  9600, SERIAL_8N1, 14, 34);
  log("GPS setup", info);
}



void setup() {
  Serial.begin(115200);
  delay(10);

  cellularSetup();
  mqttSetup();
  gpsSetup();
  bluetoothSetup();
  timerSetup();
  lastActive = esp_timer_get_time();
  log("Setup finished", info);
  sendMqtt("corsaE", "setup finished");
}

void loop() {
  mqttLoop();
  delay(100);
  refreshData();
  gpsLoop();
  sendGpsPosition();
  publishOdbData();

  lastActive = esp_timer_get_time();
}

void gpsLoop()
{
  while (Serial2.available())
  {
    gps.encode(Serial2.read());
  }
  if (gps.location.isUpdated())
  {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    gpsReady = true;

    logLine("Satellite Count: ", debug);
    logLine(String(gps.satellites.value()), debug);
    logLine(" - Latitude: ", debug);
    logLine(String(latitude), debug);
    logLine(" - Longitude: ", debug);
    log(String(longitude), debug);
  }
}

void sendGpsPosition()
{
  if (gpsPublished || (latitude == 0 && longitude == 0))
  {
    return;
  }
  int totalLength = 8;
  int decimals = 5;
  log("publish coordinates", info);
  char latString[10];
  char lngString[10];
  dtostrf(latitude, totalLength, decimals, latString);
  dtostrf(longitude, totalLength, decimals, lngString);
  sendMqtt("corsaE-lat", latString);
  sendMqtt("corsaE-lng", lngString);
  gpsPublished = true;
}

void mqttLoop()
{
  if (!mqttClient.connected()) {
    log("try to reconnect to mqtt server", debug);
    mqttReconnect();
  }
  mqttClient.loop();
}

void publishOdbData()
{
  if (!dataCollected || (getSecondsSinceStartup() - lastODBValuesSent) < mqttODBResendInterval)
  {
    return;
  }

  log("Send collected data", debug);
  String combinedValues = ambTemp + ',' + batTemp + ',' + carSpeed + ',' + batVolt + ',' + batVMin + ',' + batVMax + ',' + socCal + ',' + kwhAvai + ',' + soh + ',' + soc + ',' + batCurr;
  sendMqtt("corsaE/obd-values", combinedValues);

  dataCollected = false;
  lastODBValuesSent = getSecondsSinceStartup();
}

void refreshData()
{
  char *dataSet1Init[] = {"ATSH6A2", "ATFCSH6A2", "ATCRA682"};
  char *dataSet2Init[] = {"ATSH6B4", "ATFCSH6B4", "ATCRA694"};

  for (int i = 0; i < 3; i++)
  {
    sendOdbCommand(dataSet1Init[i]);
  }

  ambTemp = sendOdbCommandAndDisplayResult("22D4341");
  batTemp = sendOdbCommandAndDisplayResult("22D8EF1");
  carSpeed = sendOdbCommandAndDisplayResult("22D4021");

  for (int i = 0; i < 3; i++)
  {
    sendOdbCommand(dataSet2Init[i]);
  }
  delay(500);

  batCurr = sendOdbCommandAndDisplayResult("22D8161");

  batVolt = sendOdbCommandAndDisplayResult("22D8151");
  batVMin = sendOdbCommandAndDisplayResult("22D86F1");
  batVMax = sendOdbCommandAndDisplayResult("22D8701");
  socCal = sendOdbCommandAndDisplayResult("22D4101");
  kwhAvai = sendOdbCommandAndDisplayResult("22D8651");
  soh = sendOdbCommandAndDisplayResult("22D8601");
  soc = sendOdbCommandAndDisplayResult("22D8101");

  dataCollected = true;
}

String sendOdbCommandAndDisplayResult(String command)
{
  String answer = sendOdbCommand(command);
  return answer;
  static char charBuf[20];
  char* charBuff = (char*) malloc(sizeof(char) * 20);
  answer.toCharArray(charBuf, answer.length());
  return charBuf;
}

String sendOdbCommand(String command)
{
  if (LOG_ODB_COMMANDS)
  {
    log("Command: " + command, debug);
  }
  for (int i = 0; i < command.length(); i++ ) {
    ODB.write(command[i]);
  }
  ODB.write('\r');
  delay(200);
  return readElmAnswer();
}

String readElmAnswer()
{
  String response = "";
  while (ODB.available())
  {
    char c = ODB.read();

    if (c != '>')
    {
      response.concat(c);
    }
  }
  if (LOG_ODB_COMMANDS)
  {
    log(response, debug);
  }
  return response;
}

void sendMqtt(char topic[], char message[])
{
  if (LOG_MQTT_MESSAGES)
  {
    logLine("topic: ", debug);
    logLine(topic, debug);
    logLine("  message: ", debug);
    log(message, debug);
  }
  if (DO_CELLULAR_OUTPUT)
  {
    mqttClient.publish(topic, message);
  }
}
void sendMqtt(char topic[], String message)
{
  char buffer[message.length()+1];
  message.toCharArray(buffer, message.length()+1);
  sendMqtt(topic, buffer);
}

void log(String logStatement, int level)
{
  if (level >= LOG_LEVEL)
  {
    Serial.println(logStatement);
  }
}

void logLine(String logStatement, int level)
{
  if (level >= LOG_LEVEL)
  {
    Serial.print(logStatement);
  }
}

int getSecondsSinceStartup()
{
  return esp_timer_get_time() / 1000 / 1000;
}
