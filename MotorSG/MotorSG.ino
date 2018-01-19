#include <mcp_can.h>              //CAN-Interface
#include <SPI.h>                  //CAN-Interface
#include <OneWire.h>              //Temp-Sensor
#include <DallasTemperature.h>    //Temp-Sensor
#include <Wire.h>                 //RTC
#include <RTClib.h>               //RTC
#include <SoftwareSerial.h>       //Fuer BT Debugging
SoftwareSerial BTSerial(7, 8);    //Fuer BT Debugging RX | TX
#include <avr/wdt.h>              //AVR Watchdog

//Makros
#define CAN0_INT      2           //Interrupt für CAN Pin
#define ONE_WIRE_BUS  9           //OneWire Pin
#define ONE_WIRE_RES  9           //Aufloesung Temperatur in Bit
#define MC_EIN        3           //Motor Controller "an" Pin
#define MC_LAST       5           //Motor Controller Lastzustand Pin
#define LUEFTER       6           //Motor und Gehaeuseluefter Pin
#define M_HALL2       A2          //Motor Strom Hallsensor 2 Pin (Sensor 1 ist ohne Nachbereitung durch Sensormodul)

#define MC_SECURE_TIMER 200       //Timer zur Gaspedal Sicherheit 

//CAN-Identifier
#define id_ZEIT     0x120     //CAN id aus Datenbasis
#define id_TEMP     0x121     //CAN id aus Datenbasis
#define id_LAST     0x111     //CAN id aus Datenbasis
#define id_GESCHW   0x112     //CAN id aus Datenbasis
#define id_MAMPERE  0x113     //CAN id aus Datenbasis
#define id_STAT     0x110     //CAN id aus Datenbasis

//Globale Variablen
double  mampere = 0;
double  Vcc;
volatile INT8U mclast = 0;
INT8U geschw = 0;
INT8U mc_secure = 0;
INT8U do_mcontrol = 0;
INT8U kl15 = 0;     
INT8U soc = 0;      //Batterie SOC errechnet
INT8U relais1 = 0;  //Vorladewiderstand
INT8U relais2 = 0;  //Hauptrelais
INT8U relais3 = 0;  //Entladerelais


//Struktur für Debuglevel
 union {
    struct {         //einzelne Bits
      unsigned int can  :1;
      unsigned int msg  :1;
      unsigned int rtc  :1;
      unsigned int temp :1;
      unsigned int amp  :1;
      unsigned int main :1;    
      unsigned int ext  :1;
      unsigned int debug:1;
    }function;
    byte byte1:8;
}debug;

//RTC globals
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sonntag", "Montag", "Dienstag", "Mittwoch", "Donnerstag", "Freitag", "Samstag"};

//Globale Variable für CAN Botschaft
MCP_CAN CAN0(10);             //CAN0 Set CS to pin 10
byte data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
INT8U dlc;
long unsigned int id;
long unsigned int rxId;
INT8U len = 0;
unsigned char rxBuf[8];
char msgString[128];      //Array für Datenempfang

//OneWire mit Dallas Sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress TempSens1, TempSens2, TempSens3, TempSens4, TempSens5, TempSens6, TempSens7, TempSens8;

////////////////////////////////////////////////////////////////////////////////////////////////
//Setup
////////////////////////////////////////////////////////////////////////////////////////////////
void setup(void)
{
  wdt_disable();
  BTSerial.begin(115200);    
  delay(3000); //Zeit zum initialisieren von BTSerial
  BTSerial.println(F("Bluetooth Verbindung initialisiert"));
  BTSerial.println(F("Setup Anfang \n"));
  
  //Initialisiere MCP2515 mit 8MHz und 125kb/s.
  //MCP_ANY(canIDMode):CAN configuration für Tranceiver;  CAN_125KBPS(canSpeed): CAN Geschwindigkeit; MCP_8MHZ(canClock):verbauter Quarz auf Board;
  BTSerial.println(F("CAN Setup Start"));
  if(CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK){ BTSerial.println("CAN Tranceiver initialisiert");}
    else BTSerial.println(F("CAN Tranceiver Initialisierungsfehler"));
  CAN0.setMode(MCP_NORMAL);   
  BTSerial.println(F("CAN Setup Ende \n"));

  //Pin configuration
  BTSerial.println(F("Pinout Setup Start"));
  pinMode(LUEFTER, OUTPUT);
  digitalWrite(LUEFTER, LOW);
  BTSerial.print(F("Luefter Pin:"));
  BTSerial.println(LUEFTER);

  pinMode(CAN0_INT, INPUT_PULLUP);
  BTSerial.print(F("CAN Interrupt Pin:"));
  BTSerial.println(CAN0_INT);

  pinMode(MC_EIN, OUTPUT);
  digitalWrite(MC_EIN, LOW);
  BTSerial.print(F("Motor Controller ein/aus Pin:"));
  BTSerial.println(MC_EIN);

  pinMode(MC_LAST, OUTPUT);
  digitalWrite(MC_LAST, LOW);
  BTSerial.print(F("Motor Controller Lastzustand Pin:"));
  BTSerial.println(MC_LAST);
  BTSerial.println(F("Pinout Setup Ende \n"));

  //Interrupts / ISR
  attachInterrupt(digitalPinToInterrupt(CAN0_INT), ISR_recMSG, FALLING);

  ////////////////////////////////////////////////////////////////////////////////////////////////
  //OneWire
  ////////////////////////////////////////////////////////////////////////////////////////////////
  BTSerial.println("OneWire Setup Start\n");
  //Initialisiere
  sensors.begin();
  BTSerial.println(F("Temperatursensoren initialisiert"));

  //Anzahl Sensoren
  BTSerial.print(F("Es wurden "));
  BTSerial.print(sensors.getDeviceCount(), DEC);
  BTSerial.println(F(" gefunden."));

  //Parasitaere Verschaltung oder Normalbetrieb
  BTSerial.print(F("Sensormodus: "));
  if (sensors.isParasitePowerMode()) BTSerial.println(F("Parasitaer\n"));
  else BTSerial.println(F("Normal\n"));

  //OneWire Sensor Zuweisung bzw. Pruefung ob vorhanden
  oneWire.reset_search();
  BTSerial.println(F("OneWire Suche zurueckgesetzt."));
  BTSerial.println(F("OneWire Sensoren zuweisen."));
  if (!oneWire.search(TempSens1)){ BTSerial.println(F("Keine Adresse gefunden fuer TempSens1"));}
  if (!oneWire.search(TempSens2)){ BTSerial.println(F("Keine Adresse gefunden fuer TempSens2"));}
  if (!oneWire.search(TempSens3)){ BTSerial.println(F("Keine Adresse gefunden fuer TempSens3"));}
  if (!oneWire.search(TempSens4)){ BTSerial.println(F("Keine Adresse gefunden fuer TempSens4"));}
  if (!oneWire.search(TempSens5)){ BTSerial.println(F("Keine Adresse gefunden fuer TempSens5"));}
  if (!oneWire.search(TempSens6)){ BTSerial.println(F("Keine Adresse gefunden fuer TempSens6"));}
  if (!oneWire.search(TempSens7)){ BTSerial.println(F("Keine Adresse gefunden fuer TempSens7"));}
  if (!oneWire.search(TempSens8)){ BTSerial.println(F("Keine Adresse gefunden fuer TempSens8"));} 
  BTSerial.println(F("OneWire Sensoren zugewiesen.\n"));

  //Sensor Adresse per Serial senden
  BTSerial.println(F("Temperatur Sensor 1 Addresse: "));
  printAddress(TempSens1);
  BTSerial.println(F("Temperatur Sensor 2 Addresse: "));
  printAddress(TempSens2);
  BTSerial.println(F("Temperatur Sensor 3 Addresse: "));
  printAddress(TempSens3);
  BTSerial.println(F("Temperatur Sensor 4 Addresse: "));
  printAddress(TempSens4);
  BTSerial.println(F("Temperatur Sensor 5 Addresse: "));
  printAddress(TempSens5);
  BTSerial.println(F("Temperatur Sensor 6 Addresse: "));
  printAddress(TempSens6);
  BTSerial.println(F("Temperatur Sensor 7 Addresse: "));
  printAddress(TempSens7);
  BTSerial.println(F("Temperatur Sensor 8 Addresse: "));
  printAddress(TempSens8);

  //Sensor Aufloesung setzen
  BTSerial.println(F("\nOneWire Sensoren Aufloesung setzen..."));
  sensors.setResolution(TempSens1, ONE_WIRE_RES);
  sensors.setResolution(TempSens2, ONE_WIRE_RES);
  sensors.setResolution(TempSens3, ONE_WIRE_RES);
  sensors.setResolution(TempSens4, ONE_WIRE_RES);
  sensors.setResolution(TempSens5, ONE_WIRE_RES);
  sensors.setResolution(TempSens6, ONE_WIRE_RES);
  sensors.setResolution(TempSens7, ONE_WIRE_RES);
  sensors.setResolution(TempSens8, ONE_WIRE_RES);
  BTSerial.println(F("OneWire Sensoren Aufloesung gesetzt.\n"));

  //Aufloesung abfragen und per Serial senden
  BTSerial.print(F("Temperatur Sensor 1 Aufloesung: "));
  BTSerial.println(sensors.getResolution(TempSens1), DEC);
  BTSerial.print(F("Temperatur Sensor 2 Aufloesung: "));
  BTSerial.println(sensors.getResolution(TempSens2), DEC);
  BTSerial.print(F("Temperatur Sensor 3 Aufloesung: "));
  BTSerial.println(sensors.getResolution(TempSens3), DEC);
  BTSerial.print(F("Temperatur Sensor 4 Aufloesung: "));
  BTSerial.println(sensors.getResolution(TempSens4), DEC);
  BTSerial.print(F("Temperatur Sensor 5 Aufloesung: "));
  BTSerial.println(sensors.getResolution(TempSens5), DEC);
  BTSerial.print(F("Temperatur Sensor 6 Aufloesung: "));
  BTSerial.println(sensors.getResolution(TempSens6), DEC);
  BTSerial.print(F("Temperatur Sensor 7 Aufloesung: "));
  BTSerial.println(sensors.getResolution(TempSens7), DEC);
  BTSerial.print(F("Temperatur Sensor 8 Aufloesung: "));
  BTSerial.println(sensors.getResolution(TempSens8), DEC);
 
  BTSerial.println(F("OneWire Setup Ende \n"));

  ////////////////////////////////////////////////////////////////////////////////////////////////
  //RTC
  ////////////////////////////////////////////////////////////////////////////////////////////////
  BTSerial.println(F("RTC Setup Start"));
  if (! rtc.begin()) {
    BTSerial.println(F("RTC nicht gefunden"));
    while (1);
  }
  if (rtc.lostPower()) {
    BTSerial.println(F("RTC hatte Spannungsverlust"));
    BTSerial.println(F("Neue Zeit wurde gesetzt:"));       //Bei Spannungsverlust auf Initialwert
    rtc.adjust(DateTime(__DATE__, __TIME__));             //Datum des Sketches als Initialwert
  }
  else{
    BTSerial.println(F("RTC hat bereits eine Zeit gespeichert"));
  }
  BTSerial.println(F("RTC Setup Ende\n"));

  BTSerial.println(F("Setup Ende\n"));
  
  wdt_enable(WDTO_2S); //Watchdogtimer 2s
}

////////////////////////////////////////////////////////////////////////////////////////////////
//OneWire Funktionen
////////////////////////////////////////////////////////////////////////////////////////////////
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) BTSerial.print(F("0"));
    BTSerial.print(deviceAddress[i], HEX);
  }
  BTSerial.println();
}

void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  BTSerial.print(F("Temperatur: "));
  BTSerial.print(tempC);
  BTSerial.println(F("°C"));
}

void printResolution(DeviceAddress deviceAddress)
{
  BTSerial.print(F("Aufloesung: "));
  BTSerial.print(sensors.getResolution(deviceAddress));
  BTSerial.println();
}

void printData(DeviceAddress deviceAddress)
{
  BTSerial.print(F("Sensor Addresse: "));
  printAddress(deviceAddress);
  BTSerial.print(F(" "));
  printTemperature(deviceAddress);
  BTSerial.println();
}

void sendTemps(void)
{
  if(debug.function.temp){
    BTSerial.print(F("\nsendTemps() Start"));
    BTSerial.print(F("Temperatursensoren auslesen..."));
  }
  sensors.requestTemperatures();
  if(debug.function.temp){
    BTSerial.println(F("Temperatursensoren ausgelesen."));
  }
  //Temperaturen senden
  noInterrupts();
  put_data(TempSens1, TempSens2, TempSens3, TempSens4, TempSens5,TempSens6,TempSens7,TempSens8);
  sendMSG(id_TEMP, 8);
  interrupts();
  if(debug.function.temp){
    BTSerial.println(F("Temperaturen gesendet."));
    BTSerial.println(F("sendTemps() Ende\n\n"));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////
//CAN Funktionen
////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt fuer CAN
void ISR_recMSG(void)
{
  if(debug.function.can){
    BTSerial.println(F("\nISR_recMSG() Start"));
  }

  CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

  switch(rxId)
  {
    case(id_LAST):
      if(kl15  && !do_mcontrol){
        mclast = (rxBuf[0]*2.55);
        do_mcontrol = 1;
        if(debug.function.can){
          BTSerial.print(F("Gaspedalstellung:"));
          //nicht benoetigt?
          //BTSerial.print(mclast);
        }
      }
    break;

    case(id_STAT):
      if(debug.function.can){
        BTSerial.print(F("Status:"));
      }
      kl15    = rxBuf[0];
      soc     = rxBuf[1];
      relais1 = rxBuf[2];
      relais2 = rxBuf[3];
      relais3 = rxBuf[4];
    break;

    default:
        BTSerial.println(F("\n\n ============================"));
        BTSerial.println(F("Unbekannte ID"));
        BTSerial.println(F("Kein ISR case"));
        BTSerial.println(F("============================\n\n"));
    break;
  }

  if(debug.function.can){
    for(byte i = 0; i<len; i++)
    {
      sprintf(msgString, " 0x%.2X", rxBuf[i]);
      BTSerial.println(msgString);
    }
    BTSerial.println(F("ISR_recMSG() Ende\n\n"));
  }
}

/*
  //Pruefen ob standard(11bit) oder extended(29bit) Frame
  if((rxId & 0x80000000) == 0x80000000)
    sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);  //speichert extended Frame
  else
    sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);            //speichert standard frame

  Serial.print(msgString);  //String ausgeben
  //Pruefen ob remote request frame
  if((rxId & 0x40000000) == 0x40000000)
    {
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    }

  /Payload
  else
    {
      for(byte i = 0; i<len; i++){
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
    }
   Serial.println("");
   Serial.println("ISR_recMSG() Ende");
 }
 */

//MSG werden nur durch inhalt von data und id bestimmt
void sendMSG(INT32U id, INT8U dlc) //ID für MSG und Payload laenge
{
  if(debug.function.can){
    BTSerial.println(F("\nsendMSG() Start"));
  }
  //Send Data:  ID, CAN Frame / Extended Frame, Data length = 8 bytes, datab (global 8 byte)
  byte sndStat = CAN0.sendMsgBuf(id, 0, dlc, data);
  if(debug.function.can){
    BTSerial.println(F("Message Sent Successfully!"));
    BTSerial.println(F("sendMSG() Ende\n\n"));
  }
}

void put_data(byte byte0, byte byte1, byte byte2, byte byte3, byte byte4, byte byte5, byte byte6, byte byte7){
  if(debug.function.can){
    BTSerial.println(F("\nput_data() Start"));
  }
  data[0] =  byte0;
  data[1] =  byte1;
  data[2] =  byte2;
  data[3] =  byte3;
  data[4] =  byte4;
  data[5] =  byte5;
  data[6] =  byte6;
  data[7] =  byte7;
  if(debug.function.can){
    BTSerial.println(F("put_data() Ende\n\n"));
  }
 }

////////////////////////////////////////////////////////////////////////////////////////////////
//Motor Steuerung
////////////////////////////////////////////////////////////////////////////////////////////////
 void f_mcontrol(void){
   if(debug.function.msg){
     BTSerial.println(F("\nf_mcontrol() Start"));
   }
  //if (mc_secure < MC_SECURE_TIMER){
      analogWrite(MC_LAST, mclast);
      if(debug.function.msg){
        BTSerial.println(F("Last:"));
        BTSerial.println(mclast);
      }
  /*}
  else{
    analogWrite(MC_LAST, 0);
  }
 */ mc_secure = 0;
  do_mcontrol = 0;
  if(debug.function.msg){
    BTSerial.println(F("f_mcontrol Ende\n\n"));
  }
 }


////////////////////////////////////////////////////////////////////////////////////////////////
//Strommessung
////////////////////////////////////////////////////////////////////////////////////////////////

void readVcc(void) {    //Referenzspannung ermitteln um analogRead() genauer zu machen
    if(debug.function.amp){
    BTSerial.println(F("\nread_Vcc() Start"));
  }
  long puffer;
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); //Pause wegen Multiplexing
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  puffer =   ADCL;
  puffer |=  ADCH<<8;
  puffer =   1125300L / puffer; 
  Vcc = puffer;
  
  if(debug.function.amp){
    BTSerial.print(F("\Referenzspannung Vcc:"));
    BTSerial.println(Vcc);
    BTSerial.println(F("read_Vcc() Ende\n\n"));
  }
}


void f_mampere(void){
  if(debug.function.amp){
    BTSerial.println(F("\nf_mampere() Start"));
  }

  int offset          = 2585;  // Offset Spannung fuer 0 Ampere 
  INT8U mVpAmp        = 20;    // Laut Datenblatt 20mV/A
  int mampere_roh     = 0;  
  double mampere_volt = 0;
     
  analogRead(M_HALL2);    //um den gemultiplexten Analogeingaengen entgegen zu wirken
  delay(10);        //um den gemultiplexten Analogeingaengen entgegen zu wirken

  mampere_roh   = analogRead(M_HALL2);
  mampere_volt  = mampere_roh * Vcc / 1024L;   //Eingangsmesswert * Referenzspannung / Aufloesung
  mampere = ((mampere_volt - offset) / mVpAmp);

  
  if(mampere<0){  //Damit durch offset keine negativstroeme angezeigt werden   
    if(debug.function.amp){
      BTSerial.print(F("Negativstrom!"));
      BTSerial.print(F("Strom ungefiltert:"));
      BTSerial.println(mampere,1); 
    }
    mampere=0;
  }
  
  if(debug.function.amp){
    BTSerial.print(F("Hallsensor (Rohwert):"));
    BTSerial.println(mampere_roh); 
    BTSerial.print(F("\t mV = ")); 
    BTSerial.print(mampere_volt,2); 
    BTSerial.print(F("\t Ampere = ")); 
    BTSerial.println(mampere,1); 
  }

  //Daten in CAN PAyload
  if(mampere <=25){
    data[0] = mampere*10;         //Motorstrom [A]
    data[1] = 10;                 //Faktor 10
  }
  else{
    data[0] = mampere;            //Motorstrom [A]
    data[1] = 1;                  //Faktor 1
  }
  data[2] = mampere_roh/5;        //analogRead() [0-1023]
  data[3] = mampere_volt/20;      //Spannung an analogRead()[0-5V]
  data[4] = mVpAmp;               
  data[5] = offset/12;

  if(debug.function.amp){     
    BTSerial.println(F("Aktuelle Werte per Can senden..."));
  }
  
  sendMSG(id_MAMPERE, 6);
  
  if(debug.function.amp){   
    BTSerial.println(F("Aktuelle Werte per Can gsendet"));
    BTSerial.println(F("f_mampere() Ende\n\n"));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////
//Geschwindigkeit
////////////////////////////////////////////////////////////////////////////////////////////////

//noch nicht umgesetzt

void f_geschw(void){
  if(debug.function.msg){
    BTSerial.println(F("f_geschw() Start\n"));
  }
  data[0] = 99;                                     //Fixer Wert wird gesendet
  sendMSG(id_GESCHW, 1);
  if(debug.function.msg){
    BTSerial.println(F("f_geschw() Ende\n\n"));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////
//RTC
////////////////////////////////////////////////////////////////////////////////////////////////
void printZeit(void)
{
  if(debug.function.rtc){
    BTSerial.println(F("\nprintZeit() Start")); 
  }

  DateTime now = rtc.now();
  if(debug.function.rtc){
    BTSerial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    BTSerial.print(", ");
    BTSerial.print(now.day(), DEC);
    BTSerial.print(".");
    BTSerial.print(now.month(), DEC);
    BTSerial.print(".");
    BTSerial.print(now.year(), DEC);
    BTSerial.print(" ");
    BTSerial.print(now.hour(), DEC);
    BTSerial.print(':');
    BTSerial.print(now.minute(), DEC);
    BTSerial.print(':');
    BTSerial.print(now.second(), DEC);
    BTSerial.println("Uhr");
    
    BTSerial.println(F("Zeit per CAN senden..."));
  }
    //Zeit per CAN senden
    put_data(now.second(),now.minute(),now.hour(),now.day(),now.dayOfTheWeek(),now.month(),0x12,0);
    sendMSG(id_ZEIT, 7);


/* Fuer Funktionen die alle x ablaufen sollen
DateTime future (now + TimeSpan(3,2,11,33)); // tage, stunden, minuten, Sekunden
Serial.print(future.minute(), DEC);
future statt now gibt den Zeitpunkt in der Zukunft auf den man wartet aus
*/
  if(debug.function.rtc){
    BTSerial.println(F("Zeit per CAN gesendet."));
    BTSerial.println(F("\nprintZeit() Ende\n\n")); 
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////
//Debugging
////////////////////////////////////////////////////////////////////////////////////////////////
void do_debug(void){
 noInterrupts();
  //Puffervariablen
  volatile int debugpuffer;
  volatile int pretoggle;

  if (BTSerial.available()){  //Pruefen ob neue Daten per Bluetooth da sind, ansonsten funktion Ende
    
    debugpuffer = BTSerial.read();         //Daten aus Serial lesen und in debuglevel speichern

    if(debug.function.debug){ 
      BTSerial.print("Debuglevel:");
      BTSerial.println(debug.byte1);
      BTSerial.println("Jetzt switch-case:\n\n");
    }  

    switch(debugpuffer){
      case(0):
        BTSerial.println(F("\n\n========================================"));
        BTSerial.println(F("\tBefehlsuebersicht:"));
        BTSerial.println(F("========================================"));
        BTSerial.println(F("0x00\tHilfe/Befehlsuebersicht"));
        
        BTSerial.println(F("\n\tDebuglevel:"));
        BTSerial.println(F("----------------------------------------"));
        BTSerial.println(F("0x01\tCAN-BUS"));
        BTSerial.println(F("0x02\tMotor-Steuerung"));
        BTSerial.println(F("0x03\tReal-Time-Clock"));
        BTSerial.println(F("0x04\tTemperatursensoren"));
        BTSerial.println(F("0x05\tStrommessung"));
        BTSerial.println(F("0x06\tmain()-Funktion"));
        BTSerial.println(F("0x07\tStatus Extern (per CAN)"));
        BTSerial.println(F("0x08\tDebugging fuer Debugger :)"));

        BTSerial.println(F("\n\tWeitere Funktionen:"));
        BTSerial.println(F("----------------------------------------"));  
        //BTSerial.println(F("0x91\tMotorstrom Offset setzen"));
        //BTSerial.println(F("0x92\tMotorstrom mV/A setzen"));
        BTSerial.println(F("0x99\tHardreset des Steuergeraetes durchfuehren"));
        BTSerial.println(F("\n\n"));
      break;

      
      case(1):
        BTSerial.println(F("Debugging: CAN"));
        if (debug.function.can){                //Wenn Bit 1 war wird es deaktiviert
          BTSerial.print(F("deaktivieren... "));
        }
        else{                                   //Wenn Bit 0 war wird es aktiviert
          BTSerial.print(F("aktivieren...    "));
        }
        
        pretoggle = debug.function.can;         //War speichern
        debug.function.can ^= 1;                //Bit togglen 
        
        if (pretoggle == debug.function.can){   //Wurde das Bit geaendert
          BTSerial.println(F("FEHLER"));
        }
        else{
          BTSerial.println(F("OK"));
        }
        if(debug.function.debug){
        BTSerial.print(F("CAN-Bit: "));
        BTSerial.println(debug.function.can);
        BTSerial.print(F("Debuglevel: "));
        BTSerial.println(debug.byte1, BIN);
        }
      break;

    
      case(2):
        BTSerial.println(F("Debugging: Motor SG"));
        if (debug.function.msg){                //Wenn Bit 1 war wird es deaktiviert
          BTSerial.print(F("deaktivieren... "));
        }
        else{                                   //Wenn Bit 0 war wird es aktiviert
          BTSerial.print(F("aktivieren...    "));
        }
        
        pretoggle = debug.function.msg;         //War speichern
        debug.function.msg ^= 1;                //Bit togglen 
        
        if (pretoggle == debug.function.msg){   //Wurde das Bit geaendert
          BTSerial.println(F("FEHLER"));
        }
        else{
          BTSerial.println(F("OK"));
        }
        if(debug.function.debug){
        BTSerial.print(F("MSG-Bit: "));
        BTSerial.println(debug.function.msg);
        BTSerial.print(F("Debuglevel: "));
        BTSerial.println(debug.byte1, BIN);
        }
      break;  

     
      case(3):
        BTSerial.println(F("Debugging: RTC"));
        if (debug.function.rtc){                //Wenn Bit 1 war wird es deaktiviert
          BTSerial.print(F("deaktivieren... "));
        }
        else{                                   //Wenn Bit 0 war wird es aktiviert
          BTSerial.print(F("aktivieren...    "));
        }
        
        pretoggle = debug.function.rtc;         //War speichern
        debug.function.rtc ^= 1;                //Bit togglen 
        
        if (pretoggle == debug.function.rtc){   //Wurde das Bit geaendert
          BTSerial.println(F("FEHLER"));
        }
        else{
          BTSerial.println(F("OK"));
        }
        if(debug.function.debug){
        BTSerial.print(F("RTC-Bit: "));
        BTSerial.println(debug.function.rtc);
        BTSerial.print(F("Debuglevel: "));
        BTSerial.println(debug.byte1, BIN);
        }
      break;


      case(4):
        BTSerial.println(F("Debugging: Temperatur"));
        if (debug.function.temp){                 //Wenn Bit 1 war wird es deaktiviert
          BTSerial.print(F("deaktivieren... "));
        }
        else{                                     //Wenn Bit 0 war wird es aktiviert
          BTSerial.print(F("aktivieren...    "));
        }
        
        pretoggle = debug.function.temp;         //War speichern
        debug.function.temp ^= 1;                //Bit togglen 
        
        if (pretoggle == debug.function.temp){   //Wurde das Bit geaendert
          BTSerial.println(F("FEHLER"));
        }
        else{
          BTSerial.println(F("OK"));
        }
        if(debug.function.debug){
        BTSerial.print(F("Temperatur-Bit: "));
        BTSerial.println(debug.function.temp);
        BTSerial.print(F("Debuglevel: "));
        BTSerial.println(debug.byte1, BIN);
        }
      break; 

     
      case(5):
        BTSerial.println(F("Debugging: Strommessung"));
        if (debug.function.amp){                //Wenn Bit 1 war wird es deaktiviert
          BTSerial.print(F("deaktivieren... "));
        }
        else{                                   //Wenn Bit 0 war wird es aktiviert
          BTSerial.print(F("aktivieren...    "));
        }
        
        pretoggle = debug.function.amp;         //War speichern
        debug.function.amp ^= 1;                //Bit togglen 
        
        if (pretoggle == debug.function.amp){   //Wurde das Bit geaendert
          BTSerial.println(F("FEHLER"));
        }
        else{
          BTSerial.println(F("OK"));
        }
        if(debug.function.debug){
        BTSerial.print(F("Strommessung-Bit: "));
        BTSerial.println(debug.function.amp);
        BTSerial.print(F("Debuglevel: "));
        BTSerial.println(debug.byte1, BIN);
        }
      break;

    
      case(6):
        BTSerial.println(F("Debugging: main()-Funktion"));
        if (debug.function.main){               //Wenn Bit 1 war wird es deaktiviert
          BTSerial.print(F("deaktivieren... "));
        }
        else{                                   //Wenn Bit 0 war wird es aktiviert
          BTSerial.print(F("aktivieren...    "));
        }
        
        pretoggle = debug.function.main;         //War speichern
        debug.function.main ^= 1;                //Bit togglen 
        
        if (pretoggle == debug.function.main){   //Wurde das Bit geaendert
          BTSerial.println(F("FEHLER"));
        }
        else{
          BTSerial.println(F("OK"));
        }
        if(debug.function.debug){
        BTSerial.print(F("main()-Bit: "));
        BTSerial.println(debug.function.main);
        BTSerial.print(F("Debuglevel: "));
        BTSerial.println(debug.byte1, BIN);
        }
      break;

    
      case(7):        
        BTSerial.println(F("Debugging: Status Extern"));
        if (debug.function.ext){                //Wenn Bit 1 war wird es deaktiviert
          BTSerial.print(F("deaktivieren... "));
        }
        else{                                   //Wenn Bit 0 war wird es aktiviert
          BTSerial.print(F("aktivieren...    "));
        }
        
        pretoggle = debug.function.ext;         //War speichern
        debug.function.ext ^= 1;                //Bit togglen 
        
        if (pretoggle == debug.function.ext){   //Wurde das Bit geaendert
          BTSerial.println(F("FEHLER"));
        }
        else{
          BTSerial.println(F("OK"));
        }
        if(debug.function.debug){
        BTSerial.print(F("Exetern-Bit: "));
        BTSerial.println(debug.function.ext);
        BTSerial.print(F("Debuglevel: "));
        BTSerial.println(debug.byte1, BIN);
        }
      break;


      case(8):
        BTSerial.println(F("Debugging: Debugger"));
        if (debug.function.debug){                //Wenn Bit 1 war wird es deaktiviert
          BTSerial.print(F("deaktivieren... "));
        }
        else{                                   //Wenn Bit 0 war wird es aktiviert
          BTSerial.print(F("aktivieren...    "));
        }
        
        pretoggle = debug.function.debug;         //War speichern
        debug.function.debug ^= 1;                //Bit togglen 
        
        if (pretoggle == debug.function.debug){   //Wurde das Bit geaendert
          BTSerial.println(F("FEHLER"));
        }
        else{
          BTSerial.println(F("OK"));
        }
        if(debug.function.debug){
        BTSerial.print(F("Debugger-Bit: "));
        BTSerial.println(debug.function.debug);
        BTSerial.print(F("Debuglevel: "));
        BTSerial.println(debug.byte1, BIN);
        }
      break;


      /*case(91):
              BTSerial.println(F("Motorstrom Offset setzen:"));
              BTSerial.println(F("...tbd...."));
      break;

      
      case(92):
              BTSerial.println(F("Motorstrom mV/A setzen:"));
              BTSerial.println(F("...tbd...."));
      break;
      */
      
      case(153):
              BTSerial.println(F("Arduino Reset wird druchgefuehrt..."));
              while(1);
      break;

   
      default:
        BTSerial.println(F("Unbekannter Befehl."));
        BTSerial.println(F("Bitte einen definierten Befehl senden!"));
        BTSerial.println(F("Fuer Hilfe 0x00 senden."));
        BTSerial.print(F("Debuglevel: "));
        if(debug.byte1){
          BTSerial.println(debug.byte1, BIN);
        }
        else{
          BTSerial.println(F("inaktiv"));
        }
      break;
    }
  } 
  interrupts(); 
}


////////////////////////////////////////////////////////////////////////////////////////////////
//main()
////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  int i_kl15;

  //Temperaturen abfragen und senden
  //sendTemps();

  //Zeit ausgeben Serial und CAN
  //printZeit();
  
  if(debug.function.main){
    BTSerial.println(F("Start"));
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////
  //KL15 Start
  ////////////////////////////////////////////////////////////////////////////////////////////////
  while(kl15)
  {
    if(debug.function.main){
      BTSerial.println(F("Kl.15 eingeschaltet."));
    }
    noInterrupts();
    readVcc();  //Referenzspannung fuer Strommessung aktualisieren (nur einmal pro Zuendungswechsel noetig)
    //digitalWrite(LUEFTER, HIGH);
    digitalWrite(MC_EIN, 1);
    wdt_reset();
    interrupts();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //KL15 ein
    ////////////////////////////////////////////////////////////////////////////////////////////////
    while(kl15)
    {
      do_debug();
      if(do_mcontrol)   //Wenn neue CAN Botschaft(Last) - Controller einstellen
      {
        noInterrupts();
        f_mcontrol();
        interrupts();
      }

      //////////////////
      //CAN Botschaften
      //////////////////
 /*     if(!(i_kl15%2)){    //Ampere senden alle 2 Zyklen
        f_mampere();
      }*/
      if(!(i_kl15%4)){//Geschwindigkeit senden alle 4 Zyklen
        noInterrupts();
        wdt_reset();
        f_geschw();
        f_mampere();
        interrupts();
      }

      if (i_kl15 > 1000)   //Zeit und Temperatur senden alle x Zyklen
      {
        interrupts();
        wdt_reset();
        printZeit();
        sendTemps();
        i_kl15 = 0;
        if(debug.function.ext){  
          BTSerial.print(F("\n\n Kl15:\t\t"));
          BTSerial.println(kl15);          
          BTSerial.print(F("Batterie SOC::\t"));
          BTSerial.print(soc);
          BTSerial.println(F("%"));
          BTSerial.print(F("Vorladerelais:\t"));
          BTSerial.println(relais1);
          BTSerial.print(F("Hauptrelais:\t"));
          BTSerial.println(relais2);
          BTSerial.print(F("Entladerelais:\t"));
          BTSerial.println(relais3);
        }
        interrupts();
      }

     interrupts();
      wdt_reset();
      mc_secure++;
      i_kl15++;
      interrupts();
      delay(3);         //Zeitspanne für ISR

      if(!kl15){break;} //inner kl15 verlassen
    }
    if(!kl15){break;}   //aeussere kl15 verlassen
  }


  ////////////////////////////////////////////////////////////////////////////////////////////////
  //KL15 aus
  ////////////////////////////////////////////////////////////////////////////////////////////////
  //Aufgaben bei Kl.15 aus
  while(!kl15)
  {
    do_debug();
    wdt_reset();
    if(debug.function.main){
      BTSerial.println(F("Kl.15 ausgeschaltet."));
    }

    noInterrupts();
    digitalWrite(LUEFTER, LOW);
    digitalWrite(MC_LAST, 0);
    digitalWrite(MC_EIN, 0);
    interrupts();

    if(debug.function.main){
      BTSerial.println(F("\nGehe in Kl.15 Warteschleife:"));
      BTSerial.println(F("Warten auf Zuendung ein..."));
    }

    while(!kl15){       //Warten auf kl15 ein
      do_debug();
      wdt_reset();
      if(kl15){break;}  //Warten verlassen
      delay(100);
    }
    if(kl15){break;}    //Zurueck in root main
  }

 /* //CAN Botschaft senden
  put_data(0x03, 0x03, 0x02, 0x01, 0x02, 0x03, 0x04, 0x05);
  sendMSG(0x12, 8);

  //CAN Botschaft senden
  put_data(0x02, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07);
  sendMSG(0x10, 8);
  */
}
