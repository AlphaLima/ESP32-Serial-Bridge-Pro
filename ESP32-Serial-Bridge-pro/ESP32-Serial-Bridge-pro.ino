//#define TASK_HANDLER
//#define BT_HANDLER_NEU
#define OTA_HANDLER
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Preferences.h>
//#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <SPIFFSEditor.h>
#include "global.h"
#ifdef OTA_HANDLER
#include <ArduinoOTA.h>
#endif // OTA_HANDLER
#define NMEA_PARSE
#ifdef BT_HANDLER_NEU
  #include "esp_bt.h"
  #include "BluetoothSerial.h"
  #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
  #endif
  BluetoothSerial SerialBT;
#endif // BT_HANDLER
#define HTTP_STATUS_OK 200
#define TMP_BUF_SIZE 127


void TaskHandler(void* PortNumber) ;
#ifdef BT_HANDLER_NEU
  #define NUM_HW_COM 4
#else
  #define NUM_HW_COM 3
#endif
#define BT_IDX     3
#define DEBUG_COM_PORT 0
#define MAX_NO_FILTER 2

/*************************  COM Port 0 *******************************/
#define SERIAL0_RXPIN 21             // receive Pin UART0
#define SERIAL0_TXPIN 1             // transmit Pin UART0

/*************************  COM Port 1 *******************************/
#define SERIAL2_RXPIN 15             // receive Pin UART1
#define SERIAL2_TXPIN 4             // transmit Pin UART1

/*************************  COM Port 2 *******************************/
#define SERIAL1_RXPIN 16            // receive Pin UART2
#define SERIAL1_TXPIN 17            // transmit Pin UART2


/*************************  COM Port 3 (SoftwareSerial) **************/
//#define SERIAL3_RXPIN 21            // receive Pin UART3
//#define SERIAL3_TXPIN 22            // transmit Pin UART3

char szTmp[TMP_BUF_SIZE];

#define bufferSize 127
uint8_t buf1[NUM_HW_COM][bufferSize];
uint16_t i1[NUM_HW_COM] ;

uint8_t buf2[NUM_HW_COM][bufferSize];
uint16_t i2[NUM_HW_COM];


struct UARTParam {
  int Port;
  long Baudrate;
  char Parity;
  long Stop;
  long Data;
  int RxPin;
  int TxPin;
  boolean xCOM[NUM_HW_COM];
  int NoNMEAPass;
  int NoNMEABlock;
  String NMEAPass[MAX_NO_FILTER];
  String NMEABlock[MAX_NO_FILTER];
  TaskHandle_t TaskHandle;
  SemaphoreHandle_t  WriteSemaphore;
  SemaphoreHandle_t  ReadSemaphore ;
};

UARTParam COM[NUM_HW_COM];
UARTParam COM_BT;

#ifdef TASK_HANDLER
SemaphoreHandle_t barrierSemaphore = xSemaphoreCreateCounting( NUM_HW_COM, 0 );
#endif

#define HARDWARE_SERIAL
#define PROTOCOL_TCP


HardwareSerial Serial1(1);
HardwareSerial Serial2(2);
HardwareSerial* HW_SERIAL[3] = {&Serial, &Serial1 , &Serial2};
HardwareSerial* Debug_COM = HW_SERIAL[DEBUG_COM_PORT];
#define MAX_NMEA_CLIENTS 4

#define UART_5_BIT 0x00
#define UART_6_BIT 0x04
#define UART_7_BIT 0x08
#define UART_8_BIT 0x0c

#define UART_1_STOPBIT 0x10
#define UART_2_STOPBIT 0x30

#define UART_NO_PARITY   0x00
#define UART_EVEN_PARITY 0x02
#define UART_ODD_PARITY  0x03

long detRate(int recpin)  // function to return valid received baud rate
// Note that the serial monitor has no 600 baud option and 300 baud
// doesn't seem to work with version 22 hardware serial library
{
  long baud, rate = 10000, x;
  for (int i = 0; i < 10; i++) {
    x = pulseIn(recpin, LOW);  // measure the next zero bit width
    rate = x < rate ? x : rate;
  }

  if (rate < 12)
    baud = 115200;
  else if (rate < 20)
    baud = 57600;
  else if (rate < 29)
    baud = 38400;
  else if (rate < 40)
    baud = 28800;
  else if (rate < 60)
    baud = 19200;
  else if (rate < 80)
    baud = 14400;
  else if (rate < 150)
    baud = 9600;
  else if (rate < 300)
    baud = 4800;
  else if (rate < 600)
    baud = 2400;
  else if (rate < 1200)
    baud = 1200;
  else
    baud = 0;
  return baud;
}

void InitSerial(int No)
{
  pinMode(COM[No].RxPin, INPUT_PULLUP);
  //  long DetBaud = detRate(COM[No].RxPin);
  //  if(Debug_COM != NULL) Debug_COM->printf("detected Baudrate: %ul  ",DetBaud);


  unsigned long  ulParam = 0x8000000;
  switch (COM[No].Data)
  {
    case 5:  ulParam |= UART_5_BIT; break;
    case 6:  ulParam |= UART_6_BIT; break;
    case 7:  ulParam |= UART_7_BIT; break;
    default:
    case 8:  ulParam |= UART_8_BIT; break;
  }
  switch (COM[No].Stop)
  {
    default:
    case 1:  ulParam |= UART_1_STOPBIT; break;
    case 2:  ulParam |= UART_2_STOPBIT; break;
  }
  switch (COM[No].Parity)
  {
    default:
    case 'N':  ulParam |= UART_NO_PARITY;   break;
    case 'O':  ulParam |= UART_EVEN_PARITY; break;
    case 'E':  ulParam |= UART_ODD_PARITY;  break;
  }
  pinMode(COM[No].RxPin, INPUT_PULLUP);
  pinMode(COM[No].TxPin, INPUT_PULLUP);
//  if (Debug_COM != NULL) Debug_COM->printf("Open COM%u: %uBd %1u%c%1u <==> Port:%u\n", No, COM[No].Baudrate, COM[No].Data, COM[No].Parity, COM[No].Stop, COM[No].Port);
  HW_SERIAL[No]->begin(COM[No].Baudrate, ulParam, COM[No].RxPin, COM[No].TxPin );


  if (Debug_COM != NULL) Debug_COM->printf("Open COM%u: %uBd %1u%c%1u <==> Port:%u\n", No, COM[No].Baudrate, COM[No].Data, COM[No].Parity, COM[No].Stop, COM[No].Port);
}

WiFiServer *hwserver[NUM_HW_COM] = {NULL, NULL, NULL};
WiFiClient hwclient[NUM_HW_COM][MAX_NMEA_CLIENTS];




void saveConfig() {

  preferences.begin("LK8000Bridge", false);
  //  preferences.clear();

  // Store to the Preferences
  preferences.putBool("bLoginEnable", bLoginEnable);
  preferences.putString("login_username", login_username);
  preferences.putString("login_password", login_password);
  preferences.putLong("COM0Baudrate", COM[0].Baudrate);
  preferences.putLong("COM1Baudrate", COM[1].Baudrate);
  preferences.putLong("COM2Baudrate", COM[2].Baudrate);

  preferences.putInt("TCPPort0", COM[0].Port);
  preferences.putInt("TCPPort1", COM[1].Port);
  preferences.putInt("TCPPort2", COM[2].Port);
  preferences.putInt("TCPPort_BT", COM_BT.Port);

  preferences.putLong("COM0Data", COM[0].Data);
  preferences.putLong("COM1Data", COM[1].Data);
  preferences.putLong("COM2Data", COM[2].Data);

  preferences.putLong("COM0Stop", COM[0].Stop);
  preferences.putLong("COM1Stop", COM[1].Stop);
  preferences.putLong("COM2Stop", COM[2].Stop);

  preferences.putChar("COM0Parity", COM[0].Parity);
  preferences.putChar("COM1Parity", COM[1].Parity);
  preferences.putChar("COM2Parity", COM[2].Parity);

  preferences.putBool("COM0xCOM0", COM[0].xCOM[0]);
  preferences.putBool("COM0xCOM1", COM[0].xCOM[1]);
  preferences.putBool("COM0xCOM2", COM[0].xCOM[2]);
  preferences.putBool("COM0xCOM_BT", COM[0].xCOM[3]);

  preferences.putBool("COM1xCOM0", COM[1].xCOM[0]);
  preferences.putBool("COM1xCOM1", COM[1].xCOM[1]);
  preferences.putBool("COM1xCOM2", COM[1].xCOM[2]);
  preferences.putBool("COM1xCOM_BT", COM[1].xCOM[3]);

  preferences.putBool("COM2xCOM0", COM[2].xCOM[0]);
  preferences.putBool("COM2xCOM1", COM[2].xCOM[1]);
  preferences.putBool("COM2xCOM2", COM[2].xCOM[2]);
  preferences.putBool("COM2xCOM_BT", COM[2].xCOM[3]);

  preferences.putBool("COM_BTxCOM0", COM_BT.xCOM[0]);
  preferences.putBool("COM_BTxCOM1", COM_BT.xCOM[1]);
  preferences.putBool("COM_BTxCOM2", COM_BT.xCOM[2]);
  preferences.putBool("COM_BTxCOM_BT", COM_BT.xCOM[3]);

  for (int i = 0; i < NUM_HW_COM; i++)
  {
    for (int j = 0; j < MAX_NO_FILTER; j++)
    {
      sprintf(szTmp, "NMEAPass%02u%02u", i, j);
      preferences.putString(szTmp, COM[i].NMEAPass[j]);
      sprintf(szTmp, "NMEABlock%02u%02u", i, j);
      preferences.putString(szTmp, COM[i].NMEABlock[j]);
    }
  }
  for (int j = 0; j < MAX_NO_FILTER; j++)
  {
    sprintf(szTmp, "NMEAPass_BT%02u", j);
    preferences.putString(szTmp, COM_BT.NMEAPass[j]);
    sprintf(szTmp, "NMEABlock_BT%02u", j);
    preferences.putString(szTmp, COM_BT.NMEABlock[j]);
  }

  preferences.putString("receiverName", receiverName);
  preferences.putBool  ("openAP", openAP);
  preferences.putString("AP_pw", AP_pw);
  preferences.putString("WiFi_ssid", WiFi_ssid);
  
  preferences.putString("WiFi_pw", WiFi_pw);
  preferences.putInt   ("iWiFiSearchTime", iWiFiSearchTime);
  preferences.putInt   ("iWiFiTxPower", iWiFiTxPower);

  // Close the Preferences
  preferences.end();
}

void loadConfig() {

  
  preferences.begin("LK8000Bridge", true);

  // Remove all preferences under opened namespace
  //preferences.clear();

  bLoginEnable = preferences.getBool("bLoginEnable", false);

  login_username = preferences.getString("login_username", "");
  login_password = preferences.getString("login_password", "");
  COM[0].Baudrate = preferences.getLong("COM0Baudrate", 115200);
  COM[1].Baudrate = preferences.getLong("COM1Baudrate", 19200);
  COM[2].Baudrate = preferences.getLong("COM2Baudrate", 19200);

  COM[0].Port = preferences.getInt("TCPPort0", 8880);
  COM[1].Port = preferences.getInt("TCPPort1", 8881);
  COM[2].Port = preferences.getInt("TCPPort2", 8882);
  COM_BT.Port = preferences.getInt("TCPPort_BT", 8883);

  COM[0].Data = preferences.getLong("COM0Data", 8);
  COM[1].Data = preferences.getLong("COM1Data", 8);
  COM[2].Data = preferences.getLong("COM2Data", 8);

  COM[0].Stop = preferences.getLong("COM0Stop", 1);
  COM[1].Stop = preferences.getLong("COM1Stop", 1);
  COM[2].Stop = preferences.getLong("COM2Stop", 1);

  COM[0].Parity = preferences.getChar("COM0Parity", 'N');
  COM[1].Parity = preferences.getChar("COM1Parity", 'N');
  COM[2].Parity = preferences.getChar("COM2Parity", 'N');

  COM[0].xCOM[0] = preferences.getBool("COM0xCOM0", false);
  COM[0].xCOM[1] = preferences.getBool("COM0xCOM1", false);
  COM[0].xCOM[2] = preferences.getBool("COM0xCOM2", false);
  COM[0].xCOM[3] = preferences.getBool("COM0xCOM_BT", false);

  COM[1].xCOM[0] = preferences.getBool("COM1xCOM0", false);
  COM[1].xCOM[1] = preferences.getBool("COM1xCOM1", false);
  COM[1].xCOM[2] = preferences.getBool("COM1xCOM2", false);
  COM[1].xCOM[3] = preferences.getBool("COM1xCOM_BT", false);

  COM[2].xCOM[0] = preferences.getBool("COM2xCOM0", false);
  COM[2].xCOM[1] = preferences.getBool("COM2xCOM1", false);
  COM[2].xCOM[2] = preferences.getBool("COM2xCOM2", false);
  COM[2].xCOM[3] = preferences.getBool("COM2xCOM_BT", false);

  COM_BT.xCOM[0] = preferences.putBool("COM_BTxCOM0", false);
  COM_BT.xCOM[1] = preferences.putBool("COM_BTxCOM1", false);
  COM_BT.xCOM[2] = preferences.putBool("COM_BTxCOM2", false);
  COM_BT.xCOM[3] = preferences.putBool("COM_BTxCOM_BT", false);

  for (int i = 0; i < NUM_HW_COM-1; i++)
  {
    COM[i].NoNMEAPass = 0;
    COM[i].NoNMEABlock = 0;
    for (int j = 0; j < MAX_NO_FILTER; j++)
    {
      sprintf(szTmp, "NMEAPass%02u%02u", i, j);
      COM[i].NMEAPass[j] = preferences.getString(szTmp, "");
      if ( COM[i].NMEAPass[j].length() > 0)  COM[i].NoNMEAPass = j + 1;
      sprintf(szTmp, "NMEABlock%02u%02u", i, j);
      COM[i].NMEABlock[j] = preferences.getString(szTmp, "");
      if ( COM[i].NMEABlock[j].length() > 0)  COM[i].NoNMEABlock = j + 1;
    }
  }
  for (int j = 0; j < MAX_NO_FILTER; j++)
  {
    sprintf(szTmp, "NMEAPass_BT%02u", j);
    COM_BT.NMEAPass[j] = preferences.getString(szTmp, "");
    if ( COM_BT.NMEAPass[j].length() > 0)  COM_BT.NoNMEAPass = j + 1;
    sprintf(szTmp, "NMEABlock_BT%02u", j);
    COM_BT.NMEABlock[j] = preferences.getString(szTmp, "");
    if ( COM_BT.NMEABlock[j].length() > 0)  COM_BT.NoNMEABlock = j + 1;
  }
  receiverName = preferences.getString("receiverName", "LK8000");
  openAP      = preferences.getBool("openAP", true);
  AP_pw = preferences.getString("AP_pw", "Flightcomputer");
  WiFi_ssid = preferences.getString("WiFi_ssid", "");
  WiFi_pw = preferences.getString("WiFi_pw", "");
  iWiFiSearchTime = preferences.getInt("iWiFiSearchTime", 0);
  iWiFiTxPower = preferences.getInt("iWiFiTxPower",20);
  
  // Close the Preferences
  preferences.end();
}

String Extract(int No, String Text)
{ String Res = "";
  Text.trim();
  int Len = Text.length();
  int From = 0, To = 0;
  for (int i = 0; i <  (No + 1); i++)
  {
    while ((Text.charAt(To) == ' ') && (To < Len))
      To++;
    From = To;
    while ((Text.charAt(To) != ' ') && (To < Len))
      To++;
  }
  Res = Text.substring(From, To);
  Res.trim();
  return (Res);
}

void handleSetting(AsyncWebServerRequest *request) {
  int i;
  /*
    if( bLoginEnable && !request->authenticate(login_username.c_str(), login_password.c_str()))
      return request->requestAuthentication();
  */
  int params = request->params();
  for (i = 0; i < params; i++) {
    AsyncWebParameter* p = request->getParam(i);
    if (p->isFile()) {
      Serial.printf("_FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
    } else if (p->isPost()) {
      Serial.printf("_POST[%s]: %s\n", p->name().c_str(), p->value().c_str());

      if (p->name() == "WiFi_ssid") {
        //p->value().toCharArray(ssid, sizeof(ssid));
        WiFi_ssid = p->value();
      }
      if (p->name() == "WiFi_pw") {
        //p->value().toCharArray(password, sizeof(password));
        if (p->value() != "********")
          WiFi_pw = p->value();
      }
      if (p->name() == "AP_pw") {
        //p->value().toCharArray(password, sizeof(password));
        AP_pw = p->value();
      }
      if (p->name() == "receiverName") {
        //p->value().toCharArray(receiverName, sizeof(receiverName));
        receiverName = p->value();
        Serial.print("receiverName is now: ");
        Serial.println(receiverName);
      }
/*      
      if (p->name() == "login_username") {
        login_username = p->value();
        Serial.print("login_username is now: ");
        Serial.println(login_username);        
      }
      if (p->name() == "login_password") {
        login_password = p->value();
        Serial.print("login_password is now: ");
        Serial.println(login_password);            
      }

      if (p->name() == "bLoginEnable") {
        if (p->value() == "0")
          bLoginEnable = false;
        else
          bLoginEnable = true;
      }      
*/
                
      if (p->name() == "iWiFiSearchTime") {
        iWiFiSearchTime = atoi(p->value().c_str());
      }

      if (p->name() == "iWiFiTxPower") {
        iWiFiTxPower = atoi(p->value().c_str());
      }

      if (p->name() == "TCPPort0") {
        COM[0].Port = atol(p->value().c_str());
        Serial.println(COM[0].Port);
        InitSerial(0);
      }
      if (p->name() == "TCPPort1") {
        COM[1].Port = atol(p->value().c_str());
        Serial.println(COM[1].Port);
        InitSerial(1);
      }
      if (p->name() == "TCPPort2") {
        COM[2].Port = atol(p->value().c_str());
        Serial.println(COM[2].Port);
        InitSerial(2);
      }

      if (p->name() == "NMEAPass0") {
        Serial.println(p->value().c_str());
        COM[0].NoNMEAPass = 0;
        for (int i = 0; i < MAX_NO_FILTER ;  i++)
        {
          COM[0].NMEAPass[i] = Extract(  i, p->value());
          if (COM[0].NMEAPass[i].length() > 0)
          {
            COM[0].NoNMEAPass = i + 1;
            COM[0].NMEAPass[i].trim();
            Serial.print( "\n Pass 0: ");
            Serial.print( " Sentence:"); Serial.print( i );    Serial.print( " : ");
            Serial.println( COM[0].NMEAPass[i]);
          }
        }
        Serial.println( COM[0].NoNMEAPass);
      }

      if (p->name() == "NMEAPass1") {
        Serial.println(p->value().c_str());
        COM[1].NoNMEAPass = 0;
        for (int i = 0; i < MAX_NO_FILTER ;  i++)
        {
          COM[1].NMEAPass[i] = Extract(  i, p->value());
          if (COM[1].NMEAPass[i].length() > 0)
          {
            COM[1].NoNMEAPass = i + 1;
            COM[1].NMEAPass[i].trim();
            Serial.print( "\n Pass 1: ");
            Serial.print( " Sentence:"); Serial.print( i );    Serial.print( " : ");
            Serial.println( COM[1].NMEAPass[i]);
          }
        }
        Serial.println( COM[1].NoNMEAPass);
      }

      if (p->name() == "NMEAPass2") {
        Serial.println(p->value().c_str());
        COM[2].NoNMEAPass = 0;
        for (int i = 0; i < MAX_NO_FILTER ;  i++)
        {
          COM[2].NMEAPass[i] = Extract(  i, p->value());
          if (COM[2].NMEAPass[i].length() > 0)
          {
            COM[2].NoNMEAPass = i + 1;
            COM[2].NMEAPass[i].trim();
            Serial.print( "\n Pass 2: ");
            Serial.print( " Sentence:"); Serial.print( i );    Serial.print( " : ");
            Serial.println( COM[2].NMEAPass[i]);
          }
        }
        Serial.println( COM[2].NoNMEAPass);
      }

      if (p->name() == "NMEABlock0") {
        Serial.println(p->value().c_str());
        COM[0].NoNMEABlock = 0;
        for (int i = 0; i < MAX_NO_FILTER ;  i++)
        {
          COM[0].NMEABlock[i] = Extract(  i, p->value());
          if (COM[0].NMEABlock[i].length() > 0)
          {
            COM[0].NoNMEABlock = i + 1;
            COM[0].NMEABlock[i].trim();
            Serial.print( "\n Block 0: ");
            Serial.print( " Sentence:"); Serial.print( i );    Serial.print( " : ");
            Serial.println( COM[0].NMEABlock[i]);
          }
        }
        Serial.println( COM[0].NoNMEABlock);
      }

      if (p->name() == "NMEABlock1") {
        Serial.println(p->value().c_str());
        COM[1].NoNMEABlock = 0;
        for (int i = 0; i < MAX_NO_FILTER ;  i++)
        {
          COM[1].NMEABlock[i] = Extract(  i, p->value());
          if (COM[1].NMEABlock[i].length() > 0)
          {
            COM[1].NoNMEABlock = i + 1;
            COM[1].NMEABlock[i].trim();
            Serial.print( "\n Block 1:");
            Serial.print( " Sentence:"); Serial.print( i );    Serial.print( " : ");
            Serial.println( COM[1].NMEABlock[i]);
          }
        }
        Serial.println( COM[1].NoNMEABlock);
      }

      if (p->name() == "NMEABlock2") {
        Serial.println(p->value().c_str());
        COM[2].NoNMEABlock = 0;
        for (int i = 0; i < MAX_NO_FILTER ;  i++)
        {
          COM[2].NMEABlock[i] = Extract(  i, p->value());
          if (COM[2].NMEABlock[i].length() > 0)
          {
            COM[2].NoNMEABlock = i + 1;
            COM[2].NMEABlock[i].trim();
            Serial.print( "\n Block 2: ");
            Serial.print( " Sentence:"); Serial.print( i );    Serial.print( " : ");
            Serial.println( COM[2].NMEABlock[i]);
          }
        }
        Serial.println( COM[2].NoNMEABlock);
      }
    } else {
      Serial.printf("_GET[%s]: %s\n", p->name().c_str(), p->value().c_str());


      if (p->name() == "bCloseAP") {
        if (p->value() == "0")
          openAP = true;
        else
          openAP = false;
        Serial.print(p->name());    Serial.println(p->value());
      }
      if (p->name() == "WiFi_ssid") {
        //p->value().toCharArray(ssid, sizeof(ssid));
        WiFi_ssid = p->value();
      }
      if (p->name() == "WiFi_pw") {
        //p->value().toCharArray(password, sizeof(password));
        WiFi_pw = p->value();
      }

      if (p->name() == "iWiFiSearchTime") {
        iWiFiSearchTime = atoi(p->value().c_str());
        Serial.print("iWiFiSearchTime is now: ");
        Serial.println(iWiFiSearchTime);
      }

      if (p->name() == "iWiFiTxPower") {
        iWiFiTxPower = atoi(p->value().c_str());
        Serial.print("iWiFiTxPower is now: ");
        Serial.println(iWiFiTxPower);
      }
      
      if (p->name() == "receiverName") {
        receiverName = p->value();
        Serial.print("receiverName is now: ");
        Serial.println(receiverName);
      }


      if (p->name() == "AP_pw") {
        AP_pw = p->value();
      }
      if (p->name() == "bCloseAP") {
        if (p->value() == "0")
          openAP = true;
        else
          openAP = false;
      }

      if (p->name() == "COM0Baudrate") {
        COM[0].Baudrate = atoi(p->value().c_str());
        InitSerial(0);
      }
      if (p->name() == "COM1Baudrate") {
        COM[1].Baudrate = atoi(p->value().c_str());

        InitSerial(1);
      }
      if (p->name() == "COM2Baudrate") {
        COM[2].Baudrate = atoi(p->value().c_str());
        InitSerial(2);
      }


      if (p->name() == "COM0Data") {
        COM[0].Data = atoi(p->value().c_str());
        InitSerial(0);
      }
      if (p->name() == "COM1Data") {
        COM[1].Data = atoi(p->value().c_str());
        InitSerial(1);
      }
      if (p->name() == "COM2Data") {
        COM[2].Data = atoi(p->value().c_str());
        Serial.println(   COM[0].Data);
        InitSerial(2);
      }

      if (p->name() == "COM0Stop") {
        COM[0].Stop = atoi(p->value().c_str());
        InitSerial(0);
      }
      if (p->name() == "COM1Stop") {
        COM[1].Stop = atoi(p->value().c_str());
        InitSerial(1);
      }
      if (p->name() == "COM2Stop") {
        COM[2].Stop = atoi(p->value().c_str());
        InitSerial(2);
      }

      if (p->name() == "COM0Parity") {
        COM[0].Parity =  (char) (p->value().charAt(0));
        InitSerial(0);
      }
      if (p->name() == "COM1Parity") {
        COM[1].Parity =  (char) (p->value().charAt(0));
        InitSerial(1);
      }
      if (p->name() == "COM2Parity") {
        COM[2].Parity =  (char) (p->value().charAt(0));
        InitSerial(2);
      }

      if (p->name() == "COM0xCOM0") {
        if (p->value() == "0")
          COM[0].xCOM[0]  = false;
        else
          COM[0].xCOM[0]  = true;
      }
      if (p->name() == "COM0xCOM1") {
        if (p->value() == "0")
          COM[0].xCOM[1]  = false;
        else
          COM[0].xCOM[1]  = true;
      }
      if (p->name() == "COM0xCOM2") {
        if (p->value() == "0")
          COM[0].xCOM[2]  = false;
        else
          COM[0].xCOM[2]  = true;
      }
      if (p->name() == "COM0xCOM_BT") {
        if (p->value() == "0")
          COM[0].xCOM[3]  = false;
        else
          COM[0].xCOM[3]  = true;
      }

      if (p->name() == "COM1xCOM0") {
        if (p->value() == "0")
          COM[1].xCOM[0]  = false;
        else
          COM[1].xCOM[0]  = true;
      }
      if (p->name() == "COM1xCOM1") {
        if (p->value() == "0")
          COM[1].xCOM[1]  = false;
        else
          COM[1].xCOM[1]  = true;
      }
      if (p->name() == "COM1xCOM2") {
        if (p->value() == "0")
          COM[1].xCOM[2]  = false;
        else
          COM[1].xCOM[2]  = true;
      }
      if (p->name() == "COM1xCOM_BT") {
        if (p->value() == "0")
          COM[1].xCOM[3]  = false;
        else
          COM[1].xCOM[3]  = true;
      }


      if (p->name() == "COM2xCOM0") {
        if (p->value() == "0")
          COM[2].xCOM[0]  = false;
        else
          COM[2].xCOM[0]  = true;
      }
      if (p->name() == "COM2xCOM1") {
        if (p->value() == "0")
          COM[2].xCOM[1]  = false;
        else
          COM[2].xCOM[1]  = true;
      }
      if (p->name() == "COM2xCOM2") {
        if (p->value() == "0")
          COM[2].xCOM[2]  = false;
        else
          COM[2].xCOM[2]  = true;
      }
      if (p->name() == "COM2xCOM_BT") {
        if (p->value() == "0")
          COM[2].xCOM[3]  = false;
        else
          COM[2].xCOM[3]  = true;
      }


      if (p->name() == "COM_BTxCOM0") {
        if (p->value() == "0")
          COM_BT.xCOM[0]  = false;
        else
          COM_BT.xCOM[0]  = true;
      }
      if (p->name() == "COM_BTxCOM1") {
        if (p->value() == "0")
          COM_BT.xCOM[1]  = false;
        else
          COM_BT.xCOM[1]  = true;
      }
      if (p->name() == "COM_BTxCOM2") {
        if (p->value() == "0")
          COM_BT.xCOM[2]  = false;
        else
          COM_BT.xCOM[2]  = true;
      }
      if (p->name() == "COM_BTxCOM_BT") {
        if (p->value() == "0")
          COM_BT.xCOM[3]  = false;
        else
          COM_BT.xCOM[3]  = true;
      }

    }//if is GET parameter
  }//for in parameter
  Serial.println("save Config");

  saveConfig();

  String sRetStr = "Saved ";
  sRetStr += params;
  sRetStr += " parameter.";


  request->send(HTTP_STATUS_OK, "text/plain", sRetStr);

}

void handleRestart(AsyncWebServerRequest *request) {
  request->send(SPIFFS, "/restart.htm");
  //pinMode(led, INPUT);
  //pinMode(15, INPUT);//chip select of SPI
  delay(200); // give webserver enough time to send page
  WiFi.disconnect();
  delay(200); // give wifi enough time to disconnect
  ESP.restart();
}


//////////////////////////////////////////////////////////////////////////
// WiFi relatedd stuff ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void setup_wifi(void*) {
  if (connectFailed) 
  {
    WiFi.mode(WIFI_AP);
    //  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(receiverName.c_str(), AP_pw.c_str());

    if (bSioDebugInit) Serial.println("Deactivating Client WiFi.");
    if (bSioDebugInit) Serial.print("Created WiFi Access Point: '");
    if (bSioDebugInit) Serial.print(receiverName);
    if (bSioDebugInit) Serial.print("' Password: '");
    if (bSioDebugInit) Serial.print(AP_pw);
    if (bSioDebugInit) Serial.println("'");
    if (bSioDebugInit) Serial.print("IP address: ");
    if (bSioDebugInit) Serial.println( WiFi.softAPIP());
  } 
  else if (openAP) {
    WiFi.mode(WIFI_AP_STA);

    WiFi.softAP(receiverName.c_str(), AP_pw.c_str());

    if (bSioDebugInit) Serial.println("");
    if (bSioDebugInit) Serial.printf("Created WiFi: '");
    if (bSioDebugInit) Serial.print(receiverName);
    if (bSioDebugInit) Serial.print("' Password: '");
    if (bSioDebugInit) Serial.print(AP_pw);
    if (bSioDebugInit) Serial.println("'");
    if (bSioDebugInit) Serial.print("IP address: ");
    if (bSioDebugInit) Serial.println( WiFi.softAPIP()  );
  } else {
    WiFi.mode(WIFI_STA);
  }

  if (connectFailed == true) {
    //we open the AP only because connection failed...
    return;
  }

  WiFi.disconnect(true);
  if (iWiFiSearchTime == 0)
  {
    if (bSioDebugInit) Serial.print("WiFi Station mode disabled!");
  }
  else
  {
    WiFi.mode(WIFI_MODE_APSTA);


    if (bSioDebugInit) Serial.println("");
    if (bSioDebugInit) Serial.print("Connecting to WiFi: '");
    if (bSioDebugInit) Serial.print(WiFi_ssid);
    if (bSioDebugInit) Serial.print("' Password: '");
    if (bSioDebugInit) Serial.print(WiFi_pw);
    if (bSioDebugInit) Serial.println("'");

    WiFi.begin(WiFi_ssid.c_str(), WiFi_pw.c_str());
    if (openAP == false) {
      WiFi.mode(WIFI_STA);
    }

    // Wait for connection
    int iCnt = 0;

    while ( (WiFi.status() != WL_CONNECTED) && ((iCnt < (iWiFiSearchTime * 2)) || (iWiFiSearchTime < 0)) ) {
      delay(500);
      if (bSioDebugInit) Serial.print(".");
      iCnt++;
    }
    if (bSioDebugInit) Serial.println("");
    if ( (iWiFiSearchTime > 0) && (iCnt >= (iWiFiSearchTime * 2)) ) {
      if (bSioDebugError) Serial.print("Failed to connect to '");
      if (bSioDebugError) Serial.print(WiFi_ssid);
      if (bSioDebugError) Serial.println("'");
      
      WiFiClientState = "Failed to connect to '";
      WiFiClientState += WiFi_ssid;
      WiFiClientState += "'";
      connectFailed = true;
      setup_wifi(NULL);
    } else {

      if(bSioDebugInit) Serial.println("");  
      if(bSioDebugInit) Serial.print("Connected to: ");  
      if(bSioDebugInit) Serial.print(WiFi_ssid);                     
      if(bSioDebugInit) Serial.print(" IP address: ");
      if(bSioDebugInit) Serial.println( WiFi.localIP()  );        



      //update firmware OTA
      //update_firmware();
    }
  }
  esp_wifi_set_max_tx_power(iWiFiTxPower);  //lower WiFi Power
}

int setup_spiffs() {
  SPIFFS.begin(true);
  return 0;
}

void setup_webserver(void *) {



    
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.htm");
//  server.addHandler(new SPIFFSEditor(SPIFFS, login_username.c_str(), login_password.c_str()));

  server.on("/heap", HTTP_GET, [](AsyncWebServerRequest * request) {  

            
    request->send(HTTP_STATUS_OK, "text/plain", String(ESP.getFreeHeap()));
 
  });

  server.on("/set",  HTTP_ANY, handleSetting); //access protected
  server.onNotFound([](AsyncWebServerRequest * request) {    
    Serial.printf("NOT_FOUND: ");
    if (request->method() == HTTP_GET)
      Serial.printf("GET");
    else if (request->method() == HTTP_POST)
      Serial.printf("POST");
    else if (request->method() == HTTP_DELETE)
      Serial.printf("DELETE");
    else if (request->method() == HTTP_PUT)
      Serial.printf("PUT");
    else if (request->method() == HTTP_PATCH)
      Serial.printf("PATCH");
    else if (request->method() == HTTP_HEAD)
      Serial.printf("HEAD");
    else if (request->method() == HTTP_OPTIONS)
      Serial.printf("OPTIONS");
    else
      Serial.printf("UNKNOWN");
    Serial.printf(" http://%s%s\n", request->host().c_str(), request->url().c_str());

    if (request->contentLength()) {
      Serial.printf("_CONTENT_TYPE: %s\n", request->contentType().c_str());
      Serial.printf("_CONTENT_LENGTH: %u\n", request->contentLength());
    }

    int headers = request->headers();
    int i;
    for (i = 0; i < headers; i++) {
      AsyncWebHeader* h = request->getHeader(i);
      Serial.printf("_HEADER[%s]: %s\n", h->name().c_str(), h->value().c_str());
    }

    int params = request->params();
    for (i = 0; i < params; i++) {
      AsyncWebParameter* p = request->getParam(i);
      if (p->isFile()) {
        Serial.printf("_FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
      } else if (p->isPost()) {
        Serial.printf("_POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
      } else {
        Serial.printf("_GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
      }
    }

    request->send(404);
  });

  server.on("/loadRootState", HTTP_GET, [](AsyncWebServerRequest * request) {
    String values = ""; // Build a string, like this:  ID|VALUE|TYPE
    values += "net_state|WiFi: "; values += WiFiClientState; values += "|div\n";

 /*   //    values += "net_list|";values += Networks;values += "|div\n";
    values += "LastUpdate|60|input\n";
    if (1) { //if(timeStatus() == timeNotSet){
      values += "timeState|Time: Not synced yet|div\n";
    } else {

    }
*/
    request->send(HTTP_STATUS_OK, "text/plain", values);
  });
  server.on("/loadRootData", HTTP_GET, [](AsyncWebServerRequest * request) {
    String values = ""; // Build a string, like this:  ID|VALUE|TYPE
    values += "receiverName|"; values += receiverName; values += "|input\n";
    values += "WiFi_ssid|"; values += WiFi_ssid; values += "|input\n";
    values += "bCloseAP|" ; values += openAP        ? "" : "checked"; values += "|chk\n";

    values += "COM0xCOM0|"; values += COM[0].xCOM[0] ? "checked" : ""; values += "|chk\n";
    values += "COM0xCOM1|"; values += COM[0].xCOM[1] ? "checked" : ""; values += "|chk\n";
    values += "COM0xCOM2|"; values += COM[0].xCOM[2] ? "checked" : ""; values += "|chk\n";
    values += "COM0xCOM_BT|"; values += COM[0].xCOM[3] ? "checked" : ""; values += "|chk\n";

    values += "COM1xCOM0|"; values += COM[1].xCOM[0] ? "checked" : ""; values += "|chk\n";
    values += "COM1xCOM1|"; values += COM[1].xCOM[1] ? "checked" : ""; values += "|chk\n";
    values += "COM1xCOM2|"; values += COM[1].xCOM[2] ? "checked" : ""; values += "|chk\n";
    values += "COM1xCOM_BT|"; values += COM[1].xCOM[3] ? "checked" : ""; values += "|chk\n";

    values += "COM2xCOM0|"; values += COM[2].xCOM[0] ? "checked" : ""; values += "|chk\n";
    values += "COM2xCOM1|"; values += COM[2].xCOM[1] ? "checked" : ""; values += "|chk\n";
    values += "COM2xCOM2|"; values += COM[2].xCOM[2] ? "checked" : ""; values += "|chk\n";
    values += "COM2xCOM_BT|"; values += COM[2].xCOM[3] ? "checked" : ""; values += "|chk\n";

    values += "COM_BTxCOM0|"; values += COM_BT.xCOM[0] ? "checked" : ""; values += "|chk\n";
    values += "COM_BTxCOM1|"; values += COM_BT.xCOM[1] ? "checked" : ""; values += "|chk\n";
    values += "COM_BTxCOM2|"; values += COM_BT.xCOM[2] ? "checked" : ""; values += "|chk\n";
    values += "COM_BTxCOM_BT|"; values += COM_BT.xCOM[3] ? "checked" : ""; values += "|chk\n";

    values += "net_state|"; values += WiFiClientState; values += "|div\n";
    //values += "net_list|";values += Networks;values += "|div\n";
    values += "firmware_rev|Firmware: "; values += VERSION; values += "|div\n";
//    values += "login_username|"; values += login_username; values += "|input\n";
//    values += "login_password|"; values += login_password; values += "|input\n";
//    values += "bLoginEnable|"; values += bLoginEnable ? "checked" : ""; values += "|chk\n";
    values += "COM0Baudrate|"; values += COM[0].Baudrate; values += "|input\n";
    values += "COM1Baudrate|"; values += COM[1].Baudrate; values += "|input\n";
    values += "COM2Baudrate|"; values += COM[2].Baudrate; values += "|input\n";
    values += "TCPPort0|"; values += COM[0].Port; values += "|input\n";
    values += "TCPPort1|"; values += COM[1].Port; values += "|input\n";
    values += "TCPPort2|"; values += COM[2].Port; values += "|input\n";
    values += "TCPPort_BT|"; values += COM_BT.Port; values += "|input\n";

    values += "COM0Data|"; values +=  COM[0].Data; values += "|input\n";
    values += "COM1Data|"; values +=  COM[1].Data; values += "|input\n";
    values += "COM2Data|"; values +=  COM[2].Data; values += "|input\n";

    values += "COM0Stop|"; values +=  COM[0].Stop; values += "|input\n";
    values += "COM1Stop|"; values +=  COM[1].Stop; values += "|input\n";
    values += "COM2Stop|"; values +=  COM[2].Stop; values += "|input\n";

    values += "COM0Parity|"; values += COM[0].Parity; values += "|input\n";
    values += "COM1Parity|"; values += COM[1].Parity; values += "|input\n";
    values += "COM2Parity|"; values += COM[2].Parity; values += "|input\n";


    for (int i = 0; i < NUM_HW_COM-1; i++)
    {
      sprintf(szTmp, "NMEAPass%u|", i);
      values += szTmp;
      for (int j = 0; j < MAX_NO_FILTER  ; j++)
      {
        if (COM[i].NMEAPass[j] .length() > 0)
          values +=  COM[i].NMEAPass[j] + " ";
      }
      values += "|input\n";

      sprintf(szTmp, "NMEABlock%u|", i);
      values += szTmp;
      for (int j = 0; j < MAX_NO_FILTER ; j++)
      {
        if (COM[i].NMEABlock[j] .length() > 0)
          values +=  COM[i].NMEABlock[j] + " ";
      }
      values += "|input\n";
    }

    sprintf(szTmp, "NMEAPass_BT|");
    values += szTmp;
    for (int j = 0; j < MAX_NO_FILTER  ; j++)
    {
      if (COM_BT.NMEAPass[j] .length() > 0)
        values +=  COM_BT.NMEAPass[j] + " ";
    }
    values += "|input\n";

    sprintf(szTmp, "NMEABlock_BT|");
    values += szTmp;
    for (int j = 0; j < MAX_NO_FILTER ; j++)
    {
      if (COM_BT.NMEABlock[j] .length() > 0)
        values +=  COM_BT.NMEABlock[j] + " ";
    }
    values += "|input\n";

    values += "headline|Serial Bridge - "; values += receiverName; values += "|div\n";
    values += "AP_pw|"; values += AP_pw; values += "|input\n";
    values += "iWiFiSearchTime|"; values += iWiFiSearchTime; values += "|input\n";
     values += "iWiFiTxPower|"; values += iWiFiTxPower; values += "|input\n";
    //values += "bPolyPosCorr|";values += bPolyPosCorr?"checked":"";values += "|chk\n";
    values += "updateView|click|button";


    request->send(HTTP_STATUS_OK, "text/plain", values);
   
  
  });
  server.on("/loadConsoleText", HTTP_ANY, [](AsyncWebServerRequest * request) {
    //request->send(HTTP_STATUS_OK, "text/plain",HttpConsoleString);
    request->send(HTTP_STATUS_OK, "text/plain", "OK");
  });
 
  server.on("/restart",  HTTP_ANY, handleRestart); //access protected
  server.begin();

  return ;
}



bool COM_available(uint8_t num)
{
  if( num < BT_IDX)
    return (HW_SERIAL[num]->available());
#ifdef BT_HANDLER_NEU      
  else   
  if(SerialBT.hasClient())
    return SerialBT.available();
#endif  
return false;  
}

int COM_read(uint8_t num)
{
xSemaphoreTake(COM[num].ReadSemaphore, portMAX_DELAY);    
  int n=0;
  if( num < BT_IDX)
    n = (HW_SERIAL[num]->read());
#ifdef BT_HANDLER_NEU      
  else   
    if(SerialBT.hasClient())  
      n = SerialBT.read();
#endif
xSemaphoreGive(COM[num].ReadSemaphore);   
return  n;   
}

size_t COM_write(uint8_t num, const uint8_t* buf, const uint8_t len )
{  
xSemaphoreTake(COM[num].WriteSemaphore, portMAX_DELAY);  
size_t n=0;
  if(num < BT_IDX)
    n = HW_SERIAL[num]->write(buf, len);
#ifdef BT_HANDLER_NEU   
  else
  if(SerialBT.hasClient())  
    n = SerialBT.write(buf, len);   
#endif
xSemaphoreGive(COM[num].WriteSemaphore);  
return  n;
}




void setup() {
 
  for(int i= 0; i < NUM_HW_COM; i++)
  {
    hwserver[i] = NULL;
    i1[i] =0;
    i2[i] =0;
    COM[i].WriteSemaphore = xSemaphoreCreateMutex ( );       
    COM[i].ReadSemaphore  = xSemaphoreCreateMutex( );           
  }
  Serial.begin(115200);
  delay(100);
  if (Debug_COM != NULL) Debug_COM->println("");     
  if (Debug_COM != NULL) Debug_COM->println("************************************");     
  if (Debug_COM != NULL) Debug_COM->print("ESP32SerialBridge V");     
  if (Debug_COM != NULL) Debug_COM->println(VERSION);     
  if (Debug_COM != NULL) Debug_COM->println("************************************");     
  loadConfig();
  COM[0].RxPin = SERIAL0_RXPIN;
  COM[0].TxPin = SERIAL0_TXPIN;
  COM[1].RxPin = SERIAL1_RXPIN;
  COM[1].TxPin = SERIAL1_TXPIN;
  COM[2].RxPin = SERIAL2_RXPIN;
  COM[2].TxPin = SERIAL2_TXPIN;


  delay(100);

  InitSerial(0);
  InitSerial(1);
  InitSerial(2);
#ifdef BT_HANDLER_NEU
// ESP_BT_MODE_BLE to use BLE only then it is safe to call esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT
// ESP_BT_MODE_BTDM

 //  esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
//  esp_err_t res= esp_bt_controller_mem_release( ESP_BT_MODE_BLE);
//  esp_err_t res2 = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N2  );
//  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
//  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  SerialBT.begin(receiverName); //Bluetooth device name
  
#endif

  HW_SERIAL[0]->println("\r\n==================\r\nCOM0:\r\n==================\r\n");
  HW_SERIAL[1]->println("\r\n==================\r\nCOM1:\r\n==================\r\n");
  HW_SERIAL[2]->println("\r\n==================\r\nCOM2:\r\n==================\r\n");

  static WiFiServer server_0(COM[0].Port);
  static WiFiServer server_1(COM[1].Port);
  static WiFiServer server_2(COM[2].Port);
  static WiFiServer server_3(COM_BT.Port);

  hwserver[0] = &server_0;
  hwserver[1] = &server_1;
  hwserver[2] = &server_2;
  hwserver[3] = &server_3;
  setup_spiffs();
  setup_wifi(NULL);
  setup_webserver(NULL);
      
  if (Debug_COM != NULL) Debug_COM->println("\n\n\n=====================");
  // configTime(0, 0, "pool.ntp.org");
  if (Debug_COM != NULL) Debug_COM->print("LOGIN USER: ");
  if (Debug_COM != NULL) Debug_COM->println(login_username);
  if (Debug_COM != NULL) Debug_COM->print("LOGIN PASS: ");
  if (Debug_COM != NULL) Debug_COM->println(login_password);
  delay(100);

  if (Debug_COM != NULL) Debug_COM->println("\n=====================");
  for (int num = 0; num < (NUM_HW_COM) ; num++)
  {
    if (Debug_COM != NULL) Debug_COM->print("Starting TCP Server ");
    if (Debug_COM != NULL) Debug_COM->println(num);
    if(hwserver[num] != NULL)
      hwserver[num]->begin(); // start TCP server
    //    hwserver[num]->setNoDelay(true);
    delay(100);
  }
  if (Debug_COM != NULL) Debug_COM->println("=====================\n");


  
#ifdef OTA_HANDLER
  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
     SPIFFS.end();
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  // if DNSServer is started with "*" for domain name, it will reply with
  // provided IP to all DNS request


  ArduinoOTA.begin();
                     /* Core where the task should run */
#endif // OTA_HANDLER   
    



#ifdef TASK_HANDLER
for(int i= 0 ; i < NUM_HW_COM ; i++){
char TaskName[50];
sprintf(TaskName,"PortHandler#%u", i);
Debug_COM = HW_SERIAL[0];
 if (Debug_COM != NULL) Debug_COM->print(TaskName);
 if (Debug_COM != NULL) Debug_COM->print("start!");
delay(250);

  xTaskCreatePinnedToCore
  (
    TaskHandler,           /* Function to implement the task */
    TaskName,              /* Name of the task */
    1000,                  /* Stack size in words */
    (void*)i,              /* Task input parameter */
    (i+1) *5,              /* Priority of the task */
    &COM[i].TaskHandle,    /* Task handle. */
    i%2);                  /* Core where the task should run */    
}
/*
for(int i= 0 ; i < NUM_HW_COM ; i++){
  xSemaphoreTake(barrierSemaphore, portMAX_DELAY);
}*/

#endif
}

void TaskHandler(void* PortNumber) 
{
  int num = (int) (PortNumber);
  for(;;)
  {
    PortHandler(num);
 //   vPortYield(); 
  }
}

/*************************************************************/
int PortHandler(int num) {
char c;
bool bSend=false;

unsigned long  oldTime=0;

/*  
  if( (micros() - oldTime) > 1000000)
  {
    oldTime = micros();
    HW_SERIAL[0]->print(num);   
  } 
*/

  if(hwserver[num] != NULL)
  {
    bool found = false;
    found = false;
    if (hwserver[num]->hasClient())
    {
      int i = 0;
      while(!found && i < MAX_NMEA_CLIENTS) {
        //find free/disconnected spot
        if (!hwclient[num][i] || !hwclient[num][i].connected()) {
          if (hwclient[num][i]) hwclient[num][i].stop();
          hwclient[num][i] = hwserver[num]->available();
          Debug_COM = HW_SERIAL[0];
          if (Debug_COM != NULL) Debug_COM->print("New client for COM");
          if (Debug_COM != NULL) Debug_COM->print(num);
          if (Debug_COM != NULL) Debug_COM->print(" Client #");
          if (Debug_COM != NULL) Debug_COM->println(i);
     //     continue;
          found = true;
        }
        i++;
      }
      //no free/disconnected spot so reject
      if(!found)
      {
        WiFiClient TmpserverClient = hwserver[num]->available();
        TmpserverClient.stop();
      }
    }
  

    for (byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
    {
      if (hwclient[num][cln])
      {
        while (hwclient[num][cln].available())
        {
          buf1[num][i1[num]] = (uint8_t)hwclient[num][cln].read(); // read char from client (LK8000 app)
          if (i1[num] < bufferSize - 1) i1[num]++;
        }
        COM_write(num, buf1[num], i1[num]); // now send to UART(num):

        if (Debug_COM != NULL) Debug_COM->write(buf1[num], i1[num]); // now send to debug UART
        i1[num] = 0;
      }
    }
  }    
}    


/*************************************************************/
int COMHandler(int num) {
char c;
bool bSend=false;

  if (COM_available(num))
  {
    bool bEnd = false;
    bool bNMEA = false;
    if ((COM[num].NoNMEAPass + COM[num].NoNMEABlock) > 0)
      bNMEA = true;

    while (COM_available(num) && !bEnd)
    {       
      c = COM_read(num) ;
      if (bNMEA)
        if (c == '$')
        {
          i2[num] = 0;
        }
      buf2[num][i2[num]] = c; // read char from UART(num)

      i2[num]++;

      if (i2[num] >= bufferSize - 1) bEnd = true;
      if (bNMEA)
      {
        if (i2[num] > 3)
          if (buf2[num][i2[num] - 3] == '*')
          {
            bEnd = true;                         
          }
      }
      else 
      { 
        if (!COM_available(num))
        {
          bEnd = true;                   
        }
      }
    }

    if (bEnd)
    {
      if (bNMEA)
      {
        buf2[num][i2[num]++] = '\n' ;
        buf2[num][i2[num]] = 0;
        bSend = true;
        String Comp = (char*)buf2[num];
        if (COM[num].NoNMEAPass > 0)
          bSend = false;
        for (int i = 0; i < COM[num].NoNMEAPass; i++)
        {
          if (Comp.startsWith(COM[num].NMEAPass[i]))
            bSend = true;
        }

        for (int i = 0; i < COM[num].NoNMEABlock; i++)
        {
          if (Comp.startsWith(COM[num].NMEABlock[i]))
            bSend = false;
        }
      } else  bSend = true;

      // now send to WiFi:
      if (bSend)
      {
        for (byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
        {              
          if (hwclient[num][cln])
          {
             hwclient[num][cln].write((char*)buf2[num], i2[num]);     
          }
        }
        for (int x = 0; x < NUM_HW_COM; x++)
        {
          if (COM[num].xCOM[x])
          {
            COM_write(x, buf2[num], i2[num]);    
          }
        }               
        i2[num] = 0;
      }
    }
  } else  { 
#ifdef  TASK_HANDLER 
 
Debug_COM = HW_SERIAL[0];
 /*  if (Debug_COM != NULL) Debug_COM->print(" suspent Task #");             
   if (Debug_COM != NULL) Debug_COM->println(num);*/
   vTaskSuspend(COM[num].TaskHandle);      
#endif   
   delay(1);
  }      
}    





/*************************************************************/
unsigned long  oldTime2;
void loop() {
#ifdef OTA_HANDLER
  ArduinoOTA.handle();
#endif // OTA_HANDLER

  if( (micros() - oldTime2) > 10000)
  {
    oldTime2 = micros();

    if (Debug_COM != NULL) Debug_COM->println(heap_caps_get_free_size(MALLOC_CAP_8BIT)); 
    if (Debug_COM != NULL) Debug_COM->println("M"); 
  } 
 
  for(int i= 0 ; i < NUM_HW_COM ; i++){  
#ifdef  TASK_HANDLER    
    if(COM_available(i))
    if(  eTaskGetState( COM[i].TaskHandle ) == eSuspended)
    {     
       /* 
      if (Debug_COM != NULL) Debug_COM->print("\n Task");     
      if (Debug_COM != NULL) Debug_COM->print(i);
      if (Debug_COM != NULL) Debug_COM->println(" resume!");    */        
      vTaskResume(COM[i].TaskHandle);
    }
#else    
    PortHandler(i);
    COMHandler(i);
#endif
  } 
}


