#ifndef GLOBAL_H
#define GLOBAL_H

#define VERSION "1.07"
/* Version history
 *  V0.01   22.11.2017  First ESP32 port
 */

#define mls_ (1)
#define sek_ (1000*mls_)
#define min_ (60*sek_)
#define hour_ (60*min_)
#define APRS_BEACON_INTERVAL  (1*min_)
#define CALIB_INTERVAL        (15*min_)
#define LOADMEASURE_INTERVALL (20*sek_)
#define WIFI_CHECK_INTERVALL  (30*min_)




//website
bool bLoginEnable = false;
String login_username = "admin";
String login_password = "admin";


// Receiver
String receiverName = "LK8000";


bool openAP = true;
bool connectFailed=false;


// Debugging
bool bSioDebugRun = true;
bool bSioDebugInit = true;
bool bSioDebugError = true;
String WiFiClientState = "";

// Webserver
AsyncWebServer server(80);
//Settings storage
Preferences preferences;

// WiFi AP
String AP_pw = "Flightcomputer";
// WiFi Client
String WiFi_ssid;
String WiFi_pw;
int iWiFiSearchTime = 20;//sec
int iWiFiTxPower = 20;






#endif


