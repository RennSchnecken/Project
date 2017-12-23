#include <mcp_can.h>              //CAN-Interface
#include <SPI.h>                  //CAN-Interface
#include <OneWire.h>              //Temp-Sensor
#include <DallasTemperature.h>    //Temp-Sensor
#include <Wire.h>                 //RTC
#include <RTClib.h>               //RTC

//Makro
#define CAN0_INT 2        //Interrupt für CAN Pin
#define ONE_WIRE_BUS 9    //OneWire Pin
#define ONE_WIRE_RES 9    //Aufloesung Temperatur in Bit
#define MC_EIN  3         //Motor Controller "an" Pin
#define MC_LAST 5         //Motor Controller Lastzustand Pin
#define LUEFTER 6         //Motor und Gehaeuseluefter Pin
#define M_HALL1  A1       //Motor Strom Hallsensor Pin
#define M_HALL2  A2       //Motor Strom Hallsensor Pin
#define MC_SECURE_TIMER 200  //Timer zur Gaspedal Sicherheit
 
#define id_ZEIT     0x120     //CAN id
#define id_TEMP     0x121     //CAN id
#define id_LAST     0x111     //CAN id
#define id_GESCHW   0x112     //CAN id
#define id_MAMPERE  0x113     //CAN id
#define id_KL15     0x110     //CAN id

//CAN0 CS auf Pin 10
MCP_CAN CAN0(10);         // Set CS to pin 10

//RTC globals
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sonntag", "Montag", "Dienstag", "Mittwoch", "Donnerstag", "Freitag", "Samstag"};

//Globale Variablen
volatile int klemme15 = 0;
volatile int mclast = 0;
volatile int geschw = 0;
volatile float mampere = 20;
int mc_secure = 0;
int do_mcontrol = 0;
int kl15 = 0;


//Globale Variable für CAN Botschaft
byte data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
INT8U dlc;
long unsigned int id;
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];      //Array für Datenempfang

//OneWire mit Dallas Sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress TempSens1, TempSens2, TempSens3, TempSens4, TempSens5;

////////////////////////////////////////////////////////////////////////////////////////////////
//Setup
////////////////////////////////////////////////////////////////////////////////////////////////
void setup(void)
{
  Serial.begin(115200);
  delay(3000); //Serielle Verbindung aufbauen lassen
  Serial.println("Setup Anfang");

  //Initialisiere MCP2515 mit 8MHz und 125kb/s.
  //MCP_ANY(canIDMode):CAN configuration für Tranceiver;  CAN_125KBPS(canSpeed): CAN Geschwindigkeit; MCP_8MHZ(canClock):verbauter Quarz auf Board;
  if(CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
    else Serial.println("Initialisierungsfehler CAN Tranceiver!");
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

  //Pin configuration
  pinMode(LUEFTER, OUTPUT);
  digitalWrite(LUEFTER, LOW);
  //Serial.println("Luefter Pin: %d",LUEFTER );
  pinMode(CAN0_INT, INPUT_PULLUP);
  //Serial.println("CAN Interrupt Pin: %d", CAN0_INT);
  pinMode(MC_EIN, OUTPUT);
  digitalWrite(MC_EIN, LOW);
  //Serial.println("Motor Controller ein/aus Pin:" MC_EIN);
  pinMode(MC_LAST, OUTPUT);
  digitalWrite(MC_LAST, LOW);
  //Serial.println("Motor Controller Lastzustand Pin:" MC_LAST);

  //Interrupts / ISR
  attachInterrupt(digitalPinToInterrupt(CAN0_INT), ISR_recMSG, FALLING);

  ////////////////////////////////////////////////////////////////////////////////////////////////
  //OneWire
  ////////////////////////////////////////////////////////////////////////////////////////////////
  //Initialisiere
  sensors.begin();

  //Anzahl Sensoren
  Serial.println("Temperatursensoren initialisiert");
  Serial.print("Es wurden ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" gefunden.");

  //Parasitaere Verschaltung oder Normalbetrieb
  Serial.print("Sensormodus: ");
  if (sensors.isParasitePowerMode()) Serial.println("Parasitaer");
  else Serial.println("Normal");

  //OneWire Sensor Zuweisung bzw. Pruefung ob vorhanden
  oneWire.reset_search();
  /*Serial.println("OneWire Suche zurueckgesetzt.");
  Serial.println("OneWire Sensoren zuweisen.");
  if (!oneWire.search(TempSens1)) Serial.println("Unable to find address for "TempSens1);
  if (!oneWire.search(TempSens2)) Serial.println("Unable to find address for "TempSens2);
  if (!oneWire.search(TempSens3)) Serial.println("Unable to find address for "TempSens3);
  if (!oneWire.search(TempSens4)) Serial.println("Unable to find address for "TempSens4);
  if (!oneWire.search(TempSens5)) Serial.println("Unable to find address for "TempSens5);
  Serial.println("OneWire Sensoren zugewiesen.");

  //Sensor Adresse per Serial senden
  Serial.print("Temperatur Sensor 1 Addresse: ");
  printAddress(TempSens1);
  Serial.println();
  Serial.print("Temperatur Sensor 2 Addresse: ");
  printAddress(TempSens2);
  Serial.println();
  Serial.print("Temperatur Sensor 3 Addresse: ");
  printAddress(TempSens3);
  Serial.println();
  Serial.print("Temperatur Sensor 4 Addresse: ");
  printAddress(TempSens4);
  Serial.println();
  Serial.print("Temperatur Sensor 5 Addresse: ");
  printAddress(TempSens5);
  Serial.println();

  //Sensor Aufloesung setzen
  Serial.println("OneWire Sensoren Aufloesung setzen.");
  sensors.setResolution(TempSens1, ONE_WIRE_RES);
  sensors.setResolution(TempSens2, ONE_WIRE_RES);
  sensors.setResolution(TempSens3, ONE_WIRE_RES);
  sensors.setResolution(TempSens4, ONE_WIRE_RES);
  sensors.setResolution(TempSens5, ONE_WIRE_RES);
  Serial.println("OneWire Sensoren Aufloesung gesetzt.");

  //Aufloesung abfragen und per Serial senden
  Serial.print("Temperatur Sensor 1 Aufloesung: ");
  Serial.print(sensors.getResolution(TempSens1), DEC);
  Serial.println();
  Serial.print("Temperatur Sensor 2 Aufloesung: ");
  Serial.print(sensors.getResolution(TempSens2), DEC);
  Serial.println();
  Serial.print("Temperatur Sensor 3 Aufloesung: ");
  Serial.print(sensors.getResolution(TempSens3), DEC);
  Serial.println();
  Serial.print("Temperatur Sensor 4 Aufloesung: ");
  Serial.print(sensors.getResolution(TempSens4), DEC);
  Serial.println();
  Serial.print("Temperatur Sensor 5 Aufloesung: ");
  Serial.print(sensors.getResolution(TempSens5), DEC);
  Serial.println();
*/
  ////////////////////////////////////////////////////////////////////////////////////////////////
  //RTC
  ////////////////////////////////////////////////////////////////////////////////////////////////
  if (! rtc.begin()) {
    Serial.println("RTC nicht gefunden"); 
    while (1);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC auf Initialwert gesetzt");  //Bei Spannungsverlust auf Initialwert
    rtc.adjust(DateTime(__DATE__, __TIME__));   //Datum des Sketches als Initialwert
  }

  
  Serial.println("Setup Ende");
}

////////////////////////////////////////////////////////////////////////////////////////////////
//OneWire Funktionen
////////////////////////////////////////////////////////////////////////////////////////////////
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  Serial.print("Temperatur C: ");
  Serial.print(tempC);
  Serial.print(" Temperatur F: ");
  Serial.print(DallasTemperature::toFahrenheit(tempC));
}

void printResolution(DeviceAddress deviceAddress)
{
  Serial.print("Aufloesung: ");
  Serial.print(sensors.getResolution(deviceAddress));
  Serial.println();
}

void printData(DeviceAddress deviceAddress)
{
  Serial.print("Sensor Addresse: ");
  printAddress(deviceAddress);
  Serial.print(" ");
  printTemperature(deviceAddress);
  Serial.println();
}

void sendTemps(void)
{
  //Serial.print("Temperatursensoren auslesen...");
  sensors.requestTemperatures();
  //Serial.println("Temperatursensoren ausgelesen.");
  //Temperaturen senden
  noInterrupts();
  put_data(TempSens1, TempSens2, TempSens3, TempSens4, TempSens5,0,0,0);
  sendMSG(id_TEMP, 5);
  interrupts();

}
////////////////////////////////////////////////////////////////////////////////////////////////
//CAN Funktionen
////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt fuer CAN
void ISR_recMSG(void)
{
  //Serial.println("ISR_recMSG() Anfang");
  CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

  switch(rxId)
  {
    case(id_LAST):
    if(kl15  && !do_mcontrol){
    Serial.print("Gaspedal Stellung:");
    mclast = (rxBuf[0]*2.55);
    do_mcontrol = 1;
    Serial.print(mclast);
    }
    break;
    
    case(id_KL15):
    Serial.print("Zuendung:");
    noInterrupts();
    kl15 = rxBuf[0]; 
    interrupts();
    break;

    default:
    break;
  }

  for(byte i = 0; i<len; i++)
  {
    sprintf(msgString, " 0x%.2X", rxBuf[i]);
    Serial.println(msgString);
  }
  //Serial.println("ISR_recMSG() Ende");
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
void sendMSG(INT32U id, INT8U dlc) //ID für MSG
{
  //Serial.println("sendMSG() Anfang");
  //Send Data:  ID, CAN Frame / Extended Frame, Data length = 8 bytes, datab (global 8 byte)
  byte sndStat = CAN0.sendMsgBuf(id, 0, dlc, data);
  //Serial.println("Message Sent Successfully!");
  //Serial.println("sendMSG() Ende");
}


void put_data(byte byte0, byte byte1, byte byte2, byte byte3, byte byte4, byte byte5, byte byte6, byte byte7){
  //Serial.println("put_data Anfang");
  data[0] =  byte0;
  data[1] =  byte1;
  data[2] =  byte2;
  data[3] =  byte3;
  data[4] =  byte4;
  data[5] =  byte5;
  data[6] =  byte6;
  data[7] =  byte7;
  //Serial.println("put_data Ende");
 }

 
////////////////////////////////////////////////////////////////////////////////////////////////
//Motor Steuerung
////////////////////////////////////////////////////////////////////////////////////////////////

 void f_mcontrol(void){
  Serial.println("f_mcontrol");
  //if (mc_secure < MC_SECURE_TIMER){
    analogWrite(MC_LAST, mclast);
      Serial.println("Last:");
      Serial.println(mclast);

  /*}
  else{
    analogWrite(MC_LAST, 0);
  }
 */ mc_secure = 0;
  do_mcontrol = 0;
    Serial.println("f_mcontrol Ende");

 }
 
////////////////////////////////////////////////////////////////////////////////////////////////
//Strommessung
////////////////////////////////////////////////////////////////////////////////////////////////
 void f_mampere(void){
  mampere = (analogRead(M_HALL1)/0.2);
  data[0] = mampere;
  data[1] = analogRead(M_HALL1);
  
  mampere = (analogRead(M_HALL2)/0.2);
  data[2] = mampere;
  data[3] = analogRead(M_HALL2);
  
  sendMSG(id_MAMPERE, 4);
 }

////////////////////////////////////////////////////////////////////////////////////////////////
//Geschwindigkeit
////////////////////////////////////////////////////////////////////////////////////////////////
void f_geschw(void){
  data[0] = 99;
  sendMSG(id_GESCHW, 1);
} 

////////////////////////////////////////////////////////////////////////////////////////////////
//RTC
////////////////////////////////////////////////////////////////////////////////////////////////
void printZeit(void)
{
  DateTime now = rtc.now();
  /*  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(", ");
    Serial.print(now.day(), DEC);
    Serial.print(".");
    Serial.print(now.month(), DEC);
    Serial.print(".");
    Serial.print(now.year(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println("Uhr");
    */
    //Zeit per CAN senden
    put_data(now.second(),now.minute(),now.hour(),now.day(),now.dayOfTheWeek(),now.month(),0x12,0);
    sendMSG(id_ZEIT, 7);
  
/* Fuer Funktionen die alle x ablaufen sollen
DateTime future (now + TimeSpan(3,2,11,33)); // tage, stunden, minuten, Sekunden
Serial.print(future.minute(), DEC);
future statt now gibt den Zeitpunkt in der Zukunft auf den man wartet aus
*/
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
    Serial.println("Start");

////////////////////////////////////////////////////////////////////////////////////////////////
//KL15 Start 
////////////////////////////////////////////////////////////////////////////////////////////////
  while(kl15) 
  {
    Serial.println("Kl.15 eingeschaltet.");
    noInterrupts();
    //digitalWrite(LUEFTER, HIGH);
    digitalWrite(MC_EIN, 1);
    interrupts();
    
////////////////////////////////////////////////////////////////////////////////////////////////
//KL15 ein
////////////////////////////////////////////////////////////////////////////////////////////////
    while(kl15)
    {
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
        f_geschw();
        f_mampere();
        interrupts();
      }
      
      if (i_kl15 > 1000)   //Zeit und Temperatur senden alle x Zyklen
      {
        interrupts();
        printZeit();
        sendTemps();
        i_kl15 = 0;
        interrupts();
      }

     interrupts();
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
    Serial.println("Kl.15 ausgeschaltet.");
    noInterrupts();
    digitalWrite(LUEFTER, LOW);
    digitalWrite(MC_LAST, 0);
    digitalWrite(MC_EIN, 0);
    interrupts();
    Serial.println("Warten auf Zuendung ein.");
    
    while(!kl15){       //Warten auf kl15 ein
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
