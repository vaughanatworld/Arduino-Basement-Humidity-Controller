// 2013013 BAV
// Sunday at Dunkin Donuts.
// FS_Temp_Sensor_Ver2.pde   ·   2011-09-22 08:33   ·   Daniel McClain
// http://makeprojects.com/Project/Arduino-Temp-Humidity-Monitor-with-Web-and-SNMP/1367
// 
// LIBRARY INCLUDES

#include <Streaming.h> // Used to format print strings.
#include <MemoryFree.h> // Used to report the amount of free memory. (A nice to have)
#include <Agentuino.h> // Light weight SNMP agent for arduino
#include <Bounce.h>    // mechanical switch debounce. (for menu navigation)
#include <SPI.h>       // Used by ethernet shield
#include <Ethernet.h>   
//#include <SD.h>        // SD card on ethernet shield.
#include <Flash.h>     // flash memory on microcontroller
#include <MAX31855.h>  // K thermocouple with Maxim chip.
#include <SoftwareSerial.h> // To talk to the Sparkfun Serial LCD kit
#include <serLCD.h>    // Sparkfun Serial LCD kit. Higher level
#include <TimedAction.h> // To implement Protothreading and general millis() timing. (sample rate for history generation)
#include "DHT.h"       // Temp/humidity library

//
#define DEBUG
//
//pin constants
const int LCDpin      = 2; // Sparkfun Serial LCD Kit
const int BUTTONpin   = 3; // Menu: (NO) push button; to ground; needs debouncing
const int MICROSDpin  = 4; // Hardwired on Ethernet Shield Micro SD card
const int REDLEDpin   = 9; // Not used
const int THERMOdo    = 6; // Thermocouple K type via MAX31855
const int THERMOcs    = 7; // Thermocouple 
const int THERMOclk   = 8; // Thermocouple
const int GREENLEDpin = 5; // Not used
//        ETHERNET    = 11; // Ethernet Shield WIZnet 5100
//        ETHERNET    = 12; // ''
//        ETHERNET    = 13; // ''
const int POTpin      = A1; // Backlight setting for LCD (10k, 5V to ground)
//const int HUMIDpin    = A0; // Not used. Sparkfun breakout for Honeywell's analog HIH-4030 humidity sensor.
const int DHTpin1     = A2; // DHT22 temp/humidity sensor (digital) outside
const int DHTpin2     = A3; // DHT22 temp/humidity sensor (digital) inside basement

//Constants
byte mac[] = { 0x90, 0xA2, 0xDA, 0x00, 0x78, 0x80 };
byte ip[] = { 192,168,11,5 };
static byte gateway[] = { 192, 168, 11, 1 };
static byte subnet[] = { 255, 255, 255, 0 };
int webport = 80;

// SMNP (I used iReasoning MIB Browser http://www.ireasoning.com )
static char sysDescr[] PROGMEM      = "1.3.6.1.2.1.1.1.0";  // read-only  (DisplayString)
static char sysObjectID[] PROGMEM   = "1.3.6.1.2.1.1.2.0";  // read-only  (ObjectIdentifier)
static char sysUpTime[] PROGMEM     = "1.3.6.1.2.1.1.3.0";  // read-only  (TimeTicks)
static char sysContact[] PROGMEM    = "1.3.6.1.2.1.1.4.0";  // read-write (DisplayString)
static char sysName[] PROGMEM       = "1.3.6.1.2.1.1.5.0";  // read-write (DisplayString)
static char sysLocation[] PROGMEM   = "1.3.6.1.2.1.1.6.0";  // read-write (DisplayString)
static char sysServices[] PROGMEM   = "1.3.6.1.2.1.1.7.0";  // read-only  (Integer)
static char sysTemp1name[] PROGMEM  = "1.3.6.1.4.1.38635.5.1.1.0"; // read-write (temp name)
static char sysTemp1Loc[] PROGMEM   = "1.3.6.1.4.1.38635.5.1.2.0"; // read-write (temp Location)
static char sysTemp1Fval[] PROGMEM  = "1.3.6.1.4.1.38635.5.1.3.0"; // read-only (Fahrenheit Temp)
static char sysHumi1name[] PROGMEM  = "1.3.6.1.4.1.38635.6.1.1.0"; // read-write (humid 1 name)
static char sysHumi1val[] PROGMEM   = "1.3.6.1.4.1.38635.6.1.3.0"; // read-only (humidity)
static char locDescr[]              = "Dano's NetShield";  // read-only (static)
static char locObjectID[]           = "1.3.6.1.3.38635.0";                      // read-only (static)
static uint32_t locUpTime           = 0;                                        // read-only (static)
static char locContact[20]          = "Dan McClain";                            // should be stored/read from EEPROM - read/write (not done for simplicity)
static char locName[20]             = "Agentuino";                              // should be stored/read from EEPROM - read/write (not done for simplicity)
static char locLocation[20]         = "Fort Worth, TX";                         // should be stored/read from EEPROM - read/write (not done for simplicity)
static int32_t locServices          = 7;                                        // read-only (static)
static char locTemp1name[20]        = "PRIMARY TEMP";
static char locTemp2loc[20]         = "SENSOR TOP";
static char locHumi1name[20]        = "PRIMARY HUMID";

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11 
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor
DHT dht1(DHTpin1, DHTTYPE);
DHT dht2(DHTpin2, DHTTYPE);

//variables
int BUTTONstatus      = 0;
int BUTTONoldstatus   = 0;
int MENUvalue         = 1;
int POTvalue          = 0;
int POToldvalue       = 0;
int HUMIDvalue        = 0;
int HUMIDhistory[4]   = {0,0,0,0};
int TEMPvalue         = 0;
int TEMPhistory[4]    = {0,0,0,0};
float dv1 = 0.0; // absolute humidity g/m3
float dv2 = 0.0; // vapor density g/m3
uint32_t prevMillis = millis();
char oid[SNMP_MAX_OID_LEN];
SNMP_API_STAT_CODES api_status;
SNMP_ERR_CODES status;

//OBJECTS
EthernetServer WEBSERVER    = EthernetServer(webport);
MAX31855 TEMPSENSOR         = MAX31855(THERMOclk, THERMOcs, THERMOdo);
Bounce MENUBUTTON           = Bounce( BUTTONpin, 10);
TimedAction DISPLAYMENU1    = TimedAction(1000,displayMENU1);
TimedAction DISPLAYMENU2    = TimedAction(1000,displayMENU2);
TimedAction DISPLAYMENU3    = TimedAction(1000,displayMENU3);
TimedAction DISPLAYMENU4    = TimedAction(1000,displayMENU4);
TimedAction UPDATEBACKLIGHT = TimedAction(100,updateBACKLIGHT);
TimedAction UPDATEHUMID     = TimedAction(1000,updateHUMID);
TimedAction UPDATETEMP      = TimedAction(1000,updateTEMP);
TimedAction CHECKFORWEB     = TimedAction(100,checkforWEB);
serLCD LCD                  = serLCD(LCDpin);

void displayMENU1(){
  LCD.clear();
  LCD.print("Humidity    :");
  LCD.print(HUMIDvalue);
  LCD.print("%");
  LCD.setCursor(2,1);
  LCD.print("Temperature :");
  LCD.print(TEMPvalue);
  LCD.print("c");
}
void displayMENU2(){
  LCD.clear();
  LCD.print("Humidity History");
  LCD.setCursor(2,1);
  LCD.print(HUMIDhistory[0]);
  LCD.print("% ");
  LCD.print(HUMIDhistory[1]);
  LCD.print("% ");
  LCD.print(HUMIDhistory[2]);
  LCD.print("% ");
  LCD.print(HUMIDhistory[3]);
  LCD.print("% ");
}
void displayMENU3(){
  LCD.clear();
  LCD.print("Temp History");
  LCD.setCursor(2,1);
  LCD.print(TEMPhistory[0]);
  LCD.print("c ");
  LCD.print(TEMPhistory[1]);
  LCD.print("c ");
  LCD.print(TEMPhistory[2]);
  LCD.print("c ");
  LCD.print(TEMPhistory[3]);
  LCD.print("c ");
}
void displayMENU4(){
  LCD.clear();
  //LCD.print("MAC: ");
  LCD.print(mac[0],HEX);LCD.print(":");LCD.print(mac[1],HEX);LCD.print(":");LCD.print(mac[2],HEX);LCD.print(":");LCD.print(mac[3],HEX);LCD.print(":");LCD.print(mac[4],HEX);LCD.print(":");LCD.print(mac[5],HEX);
  LCD.setCursor(2,1);
  //LCD.print("IP: ");
  LCD.print(" ");LCD.print(ip[0]);LCD.print(".");LCD.print(ip[1]);LCD.print(".");LCD.print(ip[2]);LCD.print(".");LCD.print(ip[3]);
}
void updateBACKLIGHT(){
  POTvalue = analogRead(POTpin);
    if (POTvalue != POToldvalue){
      LCD.setBrightness(map(POTvalue , 0 , 1023 , 0 , 29));
      POToldvalue = POTvalue;
    }
}
void updateHUMID(){
  // HIH-4030 humidity sensor. conversion is a little simplistic.
  //float HUMIDcalc = map(analogRead(HUMIDpin) , 0 , 1023 , 0 , 50); //converts analog input to voltage
  //HUMIDvalue = map(HUMIDcalc , 0 , 50 , 0 , 1000)/10; //converts voltage to percent
  HUMIDvalue = dht1.readHumidity();
  HUMIDvalue = dht1.readTemperature();
  HUMIDhistory[3] = HUMIDhistory[2];
  HUMIDhistory[2] = HUMIDhistory[1];
  HUMIDhistory[1] = HUMIDhistory[0];
  HUMIDhistory[0] = HUMIDvalue;
}
void updateTEMP(){
  TEMPvalue = TEMPSENSOR.readCelsius(); // Thermocouple
  TEMPvalue = dht2.readTemperature();   // DHT22
  TEMPhistory[3] = TEMPhistory[2];
  TEMPhistory[2] = TEMPhistory[1];
  TEMPhistory[1] = TEMPhistory[0];
  TEMPhistory[0] = TEMPvalue;
  dv1 = calcVaporDensity_Exponentials( TEMPhistory[0], HUMIDhistory[0]);
  dv2 = calcVaporDensity( TEMPhistory[0], HUMIDhistory[0]);
}

// grams / meter cubed.
float calcVaporDensity_Exponentials(float t, float rh) {
  return 216.7*(rh/100*6.112*exp(17.62*t/(243.12+t))/(273.15+t));
}

// vapor density grams/cubic meter.
#define vaporDensityat100rh(t)  (float)5.018 + (float)0.32321*t + (float)8.1847E-3*t*t + (float)3.1243E-4*t*t*t
float calcVaporDensity( float t, float rh)
{
  return rh * vaporDensityat100rh(t);
}

void checkforWEB(){  
   EthernetClient client = WEBSERVER.available();
   if(client) {
     boolean currentLineIsBlank = true;
     while (client.connected()){
       if (client.available()) {
         char c = client.read();
         if (c == '\n' && currentLineIsBlank){
           client.println("HTTP/1.1 200 OK");
           client.println("Content-Type: text/html");
           client.println();     
           client.print("Humidity is ");
           client.print(HUMIDvalue);
           client.print(" %");
           client.println("<br />");
           client.print("Temperature is ");
           client.print(TEMPvalue);
           client.print(" deg C");
           client.println("<br />");
           break;
         }
         if (c == '\n'){
           currentLineIsBlank = true;
         }
         else if (c != '\r'){
           currentLineIsBlank = false;
         }
       }
     }
     delay(1);
     client.stop();
   }
}
void pduReceived()
{
  SNMP_PDU pdu;
  //
  #ifdef DEBUG
    Serial << F("UDP Packet Received Start..") << F(" RAM:") << freeMemory() << endl;
  #endif
  //
  api_status = Agentuino.requestPdu(&pdu);
  Serial << api_status << endl;
  //
  if ( pdu.type == SNMP_PDU_GET || pdu.type == SNMP_PDU_GET_NEXT || pdu.type == SNMP_PDU_SET
    && pdu.error == SNMP_ERR_NO_ERROR && api_status == SNMP_API_STAT_SUCCESS ) {
    //
    pdu.OID.toString(oid);
    //
    Serial << "OID: " << oid << endl;
    //Serial << "pdu.type: " << pdu.type << endl;
    //Serial << "SNMP_PDU_GET_NEXT: " << SNMP_PDU_GET_NEXT << endl;
    //Serial << "PDU_SET: " << SNMP_PDU_SET << endl;
    //Serial << "API_status: " << api_status << endl;
    //Serial << "sysDescr: " << sysDescr << endl;
    //
    if ( strcmp_P(oid, sysDescr ) == 0 ) {
      // handle sysDescr (set/get) requests
      if ( pdu.type == SNMP_PDU_SET ) {
        // response packet from set-request - object is read-only
        pdu.type = SNMP_PDU_RESPONSE;
        pdu.error = SNMP_ERR_READ_ONLY;
      } else {
        // response packet from get-request - locDescr
        status = pdu.VALUE.encode(SNMP_SYNTAX_OCTETS, locDescr);
	pdu.type = SNMP_PDU_RESPONSE;
        pdu.error = status;
      }
      //
      #ifdef DEBUG
        Serial << F("sysDescr...") << locDescr << F(" ") << pdu.VALUE.size << endl;
      #endif
    } else if ( strcmp_P(oid, sysUpTime ) == 0 ) {
      // handle sysName (set/get) requests
      if ( pdu.type == SNMP_PDU_SET ) {
        // response packet from set-request - object is read-only
        pdu.type = SNMP_PDU_RESPONSE;
        pdu.error = SNMP_ERR_READ_ONLY;
      } else {
        // response packet from get-request - locUpTime
        status = pdu.VALUE.encode(SNMP_SYNTAX_TIME_TICKS, locUpTime);
        pdu.type = SNMP_PDU_RESPONSE;
        pdu.error = status;
      }
      //
      #ifdef DEBUG
        Serial << F("sysUpTime...") << locUpTime << F(" ") << pdu.VALUE.size << endl;
      #endif
    } else if ( strcmp_P(oid, sysName ) == 0 ) {
      // handle sysName (set/get) requests
      if ( pdu.type == SNMP_PDU_SET ) {
        // response packet from set-request - object is read/write
        status = pdu.VALUE.decode(locName, strlen(locName)); 
        pdu.type = SNMP_PDU_RESPONSE;
        pdu.error = status;
      } else {
        // response packet from get-request - locName
        status = pdu.VALUE.encode(SNMP_SYNTAX_OCTETS, locName);
        pdu.type = SNMP_PDU_RESPONSE;
        pdu.error = status;
      }
      //
      #ifdef DEBUG
        Serial << F("sysName...") << locName << F(" ") << pdu.VALUE.size << endl;
      #endif
    } else if ( strcmp_P(oid, sysContact ) == 0 ) {
      // handle sysContact (set/get) requests
      if ( pdu.type == SNMP_PDU_SET ) {
        // response packet from set-request - object is read/write
        status = pdu.VALUE.decode(locContact, strlen(locContact)); 
        pdu.type = SNMP_PDU_RESPONSE;
        pdu.error = status;
      } else {
        // response packet from get-request - locContact
        status = pdu.VALUE.encode(SNMP_SYNTAX_OCTETS, locContact);
        pdu.type = SNMP_PDU_RESPONSE;
        pdu.error = status;
      }
      //
      #ifdef DEBUG
        Serial << F("sysContact...") << locContact << F(" ") << pdu.VALUE.size << endl;
      #endif
    } else if ( strcmp_P(oid, sysLocation ) == 0 ) {
      // handle sysLocation (set/get) requests
      if ( pdu.type == SNMP_PDU_SET ) {
	// response packet from set-request - object is read/write
        status = pdu.VALUE.decode(locLocation, strlen(locLocation)); 
        pdu.type = SNMP_PDU_RESPONSE;
        pdu.error = status;
      } else {
        // response packet from get-request - locLocation
        status = pdu.VALUE.encode(SNMP_SYNTAX_OCTETS, locLocation);
        pdu.type = SNMP_PDU_RESPONSE;
        pdu.error = status;
      }
      //
      #ifdef DEBUG
        Serial << F("sysLocation...") << locLocation << F(" ") << pdu.VALUE.size << endl;
      #endif
    } else if ( strcmp_P(oid, sysHumi1val ) == 0 ) { //HUMIDITY
      // Read HUMIDITY VALUE
      updateHUMID();
      // handle sysLocation (set/get) requests
      if ( pdu.type == SNMP_PDU_SET ) {
        // response packet from set-request - object is read/write
        pdu.type = SNMP_PDU_RESPONSE;
        pdu.error = SNMP_ERR_READ_ONLY;
      } else {
        // response packet from get-request - locLocation
        status = pdu.VALUE.encode(SNMP_SYNTAX_INT, HUMIDvalue);
        pdu.type = SNMP_PDU_RESPONSE;
        pdu.error = status;
      }
      //
      #ifdef DEBUG
        Serial << F("HUMIDITY...") << HUMIDvalue << F(" ") << pdu.VALUE.size << endl;
      #endif
    } else if ( strcmp_P(oid, sysTemp1Fval ) == 0 ) { //TEMP
      // Read HUMIDITY VALUE
      updateTEMP();
      // handle sysLocation (set/get) requests
      if ( pdu.type == SNMP_PDU_SET ) {
        // response packet from set-request - object is read/write
        pdu.type = SNMP_PDU_RESPONSE;
        pdu.error = SNMP_ERR_READ_ONLY;
      } else {
        // response packet from get-request - locLocation
        status = pdu.VALUE.encode(SNMP_SYNTAX_INT, TEMPvalue);
        pdu.type = SNMP_PDU_RESPONSE;
        pdu.error = status;
      }
      //
      #ifdef DEBUG
        Serial << F("TEMP...") << TEMPvalue << F(" ") << pdu.VALUE.size << endl;
      #endif
    } else if ( strcmp_P(oid, sysObjectID ) == 0 ) {
      // handle sysContact (set/get) requests
      if ( pdu.type == SNMP_PDU_SET ) {
        // response packet from set-request - object is read/write
        status = pdu.VALUE.decode(locObjectID, strlen(locObjectID)); 
        pdu.type = SNMP_PDU_RESPONSE;
        pdu.error = status;
      } else {
        // response packet from get-request - locContact
        status = pdu.VALUE.encode(SNMP_SYNTAX_OCTETS, locObjectID);
        pdu.type = SNMP_PDU_RESPONSE;
        pdu.error = status;
      }
      //
      #ifdef DEBUG
        Serial << F("sysObjectID...") << locObjectID << F(" ") << pdu.VALUE.size << endl;
      #endif
    } else if ( strcmp_P(oid, sysServices) == 0 ) {
      // handle sysServices (set/get) requests
      if ( pdu.type == SNMP_PDU_SET ) {
        // response packet from set-request - object is read-only
        pdu.type = SNMP_PDU_RESPONSE;
        pdu.error = SNMP_ERR_READ_ONLY;
      } else {
        // response packet from get-request - locServices
        status = pdu.VALUE.encode(SNMP_SYNTAX_INT, locServices);
        pdu.type = SNMP_PDU_RESPONSE;
        pdu.error = status;
      }
      //
      #ifdef DEBUG
        Serial << F("locServices...") << locServices << F(" ") << pdu.VALUE.size << endl;
      #endif
    } else {
      // oid does not exist
      //
      // response packet - object not found
      pdu.type = SNMP_PDU_RESPONSE;
      pdu.error = SNMP_ERR_NO_SUCH_NAME;
    }
    //
    Agentuino.responsePdu(&pdu);
  }
  //
  Agentuino.freePdu(&pdu);
  //
  Serial << "UDP Packet Received End.." << " RAM:" << freeMemory() << endl;
}
///
///
void setup()
{
  pinMode(LCDpin, OUTPUT);
  //pinMode(HUMIDpin, INPUT);
  pinMode(POTpin, INPUT);
  pinMode(BUTTONpin, INPUT);
  pinMode(GREENLEDpin, OUTPUT);
  pinMode(REDLEDpin, OUTPUT);
  digitalWrite(BUTTONpin, HIGH);
  //digitalWrite(HUMIDpin, LOW);
  dht1.begin();
  dht2.begin();
  Serial.begin(9600);
  Ethernet.begin(mac, ip);
  WEBSERVER.begin();
  //
  api_status = Agentuino.begin();
  //
  if ( api_status == SNMP_API_STAT_SUCCESS ) {
    //
    Agentuino.onPduReceive(pduReceived);
    //
    delay(10);
    //
    Serial << F("SNMP Agent Initalized...") << endl;
    //
    return;
  }
  //
  delay(10);
  //
  Serial << F("SNMP Agent Initalization Problem...") << status << endl;
}
///
///
void loop()
{
  // listen/handle for incoming SNMP requests
  Agentuino.listen();
  //
  // sysUpTime - The time (in hundredths of a second) since
  // the network management portion of the system was last
  // re-initialized.
  if ( millis() - prevMillis > 1000 ) {
    // increment previous milliseconds
    prevMillis += 1000;
    //
    // increment up-time counter
    locUpTime += 100;
  }
    
   if ( MENUBUTTON.update() ) {
     if ( MENUBUTTON.read() == LOW) {
        if ( MENUvalue < 4){
          MENUvalue = MENUvalue + 1;
        } else {
          MENUvalue = 1;
        }
     }
   }
   switch(MENUvalue) {
     case 1:
     DISPLAYMENU1.check();
     break;
     case 2:
     DISPLAYMENU2.check();
     break;
     case 3:
     DISPLAYMENU3.check();
     break;
     case 4:
     DISPLAYMENU4.check();
     break;
   }
   UPDATEBACKLIGHT.check();
   UPDATEHUMID.check();
   UPDATETEMP.check();
   CHECKFORWEB.check();
  
}

