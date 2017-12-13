#include <mcp_can.h>              //CAN-Interface
#include <SPI.h>                  //CAN-Interface
#include <OneWire.h>              //Temp-Sensor
#include <DallasTemperature.h>    //Temp-Sensor
#include <Wire.h>
#include <RTClib.h>

//Makro
#define CAN0_INT 2        //Interrupt für CAN Pin
#define ONE_WIRE_BUS xx   //OneWire Pin
#define ONE_WIRE_RES 9    //Aufloesung Temperatur in Bit
#define MC_EIN  xx        //Motor Controller "an" Pin
#define MC_LAST xx        //Motor Controller Lastzustand Pin
#define LUEFTER xx        //Motor und Gehaeuseluefter Pin

#define id_Zeit xx        //CAN id
#define id_Temp xx        //CAN id

//CAN0 CS auf Pin 10
MCP_CAN CAN0(10);         // Set CS to pin 10

//RTC globals
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sonntag", "Montag", "Dienstag", "Mittwoch", "Donnerstag", "Freitag", "Samstag"};

//Globale Variablen
volatile int klemme15 = 0;
volatile int mclast = 0;

//Globale Variable für CAN Botschaft
volatile byte data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
INT32U id;
volatile long unsigned int rxId;
unsigned char len = 0;
volatile unsigned char rxBuf[8];
volatile char msgString[128];      //Array für Datenempfang

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
  Serial.println("Setup Anfang");

  //Initialisiere MCP2515 mit 8MHz und 50kb/s.
  //MCP_ANY(canIDMode):CAN configuration für Tranceiver;  CAN_50KBPS(canSpeed): CAN Geschwindigkeit; MCP_8MHZ(canClock):verbauter Quarz auf Board;
  if(CAN0.begin(MCP_ANY, CAN_50KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
    else Serial.println("Initialisierungsfehler CAN Tranceiver!");
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

  //Pin configuration
  pinMode(LUEFTER, OUTPUT);
  digitalWrite(LUEFTER, LOW);
  Serial.println("Luefter Pin:" LUEFTER);
  pinMode(CAN0_INT, INPUT);
  Serial.println("CAN Interrupt Pin:" CAN0_INT);
  pinMode(MC_EIN, OUTPUT);
  digitalWrite(MC_EIN, LOW);
  Serial.println("Motor Controller ein/aus Pin:" MC_EIN);
  pinMode(MC_LAST, OUTPUT);
  digitalWrite(MC_LAST, LOW);
  Serial.println("Motor Controller Lastzustand Pin:" MC_LAST);

  //Interrupts / ISR
  attachInterrupt(digitalPinToInterrupt(CAN0_INT), ISR_recMSG, FALLING);
  noInterrupts(); //Bis zur main()

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
  Serial.println("OneWire Suche zurueckgesetzt.");
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

  ////////////////////////////////////////////////////////////////////////////////////////////////
  //RTC
  ////////////////////////////////////////////////////////////////////////////////////////////////
  if (! rtc.begin()) {
      Serial.println("RTC nicht gefunden");
    }
  if (rtc.lostPower()) {
    Serial.println("RTC auf Initialwert gesetzt");  //Bei Spannungsverlust auf Initialwert
    rtc.adjust(DateTime(2017, 12, 31, 23, 59, 0));   //Beliebiges Datum als Initialwert 31.Dez.2017 23h 59min 59s 0ms
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

void getTemps(void)
{
  Serial.print("Temperatursensoren auslesen...");
  sensors.requestTemperatures();
  Serial.println("Temperatursensoren ausgelesen.");
}
////////////////////////////////////////////////////////////////////////////////////////////////
//CAN Funktionen
////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt fuer CAN
void ISR_recMSG(void)
{
  Serial.println("ISR_recMSG() Anfang");
  CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

  //Pruefen ob standard(11bit) oder extended(29bit) Frame
  if((rxId & 0x80000000) == 0x80000000)
    sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);  //speichert standard Frame
  else
    sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);            //speichert extended frame

  Serial.print(msgString);  //String ausgeben
  //Pruefen ob remote request frame
  if((rxId & 0x40000000) == 0x40000000)
    {
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    }

  //Standard MSG
  else
    {
      for(byte i = 0; i<len; i++){
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
    }
   Serial.println("");
   Serial.println("ISR_recMSG() Ende");
 }
}

//MSG werden nur durch inhalt von data und id bestimmt
void sendMSG(INT32U id) //ID für MSG
{
  Serial.println("sendMSG() Anfang");
  //Send Data:  ID, CAN Frame / Extended Frame, Data length = 8 bytes, datab (global 8 byte)
  byte sndStat = CAN0.sendMsgBuf(id, 0, 8, data);
  if(sndStat == CAN_OK){
    Serial.println("Message Sent Successfully!");
  } else {
    Serial.println("Error Sending Message...");
  }
  delay(300);   // send data per 300ms
  Serial.println("sendMSG() Ende");
}
////////////////////////////////////////////////////////////////////////////////////////////////
//RTC
////////////////////////////////////////////////////////////////////////////////////////////////
void printZeit(void)
{
  DateTime now = rtc.now();
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
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

}

interrupts(); //Interrupts einschalten vor main()
////////////////////////////////////////////////////////////////////////////////////////////////
//main()
////////////////////////////////////////////////////////////////////////////////////////////////
void main(void)
{



  //Temperaturen abfragen
  getTemps();

  //Zeit ausgeben Serial
  printZeit();

  //Aufgaben bei Kl.15 ein
  if (klemme15 == 1)
  {
    Serial.println("Kl.15 eingeschaltet.");
    noInterrupts();
    digitalWrite(LUEFTER, HIGH);
    digitalWrite(MC_EIN, 1);
    analogWrite(MC_LAST, mclast);
    klemme15 = 2;
    interrupts();
  }

  //Aufgaben bei Kl.15 ein und einschalten durchgelaufen
  if (klemme15 == 2)
  {
    Serial.println("Kl.15 Status 2.");
    //Motor Last einstellen -->Auslagern in Funktion bei ISR CAN
    noInterrupts();
    analogWrite(MC_LAST, mclast);
    interrupts();

    //Zeit per CAN senden
    noInterrupts();
    data = rtc.now();
    sendMSG(id_Zeit);
    interrupts();

    //Temperaturen senden
    noInterrupts();
    data = {TempSens1, TempSens2, TempSens3, TempSens4, TempSens5}
    sendMSG(id_Temp);
    interrupts();


    noInterrupts();

    interrupts();
    noInterrupts();

    interrupts();

  }


  //Aufgaben bei Kl.15 aus
  if (klemme15 == 0)
  {
    Serial.println("Kl.15 ausgeschaltet.");
    noInterrupts();
    digitalWrite(LUEFTER, LOW);
    digitalWrite(MC_LAST, 0);
    digitalWrite(MC_EIN, 0);
    interrupts();
    Serial.println("Warten auf Zuendung ein.");
    while(klemme15 == 0);

  }



  //CAN Botschaft senden
  data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03};
  sendMSG(0x12);

  //CAN Botschaft senden
  data = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
  sendMSG(0x01);



return 0;
}
