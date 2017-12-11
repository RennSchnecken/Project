#include <mcp_can.h>
#include <SPI.h>

//Makro
#define CAN0_INT 2        //Interrupt Pin für CAN                         

//CAN0 CS auf Pin 10
MCP_CAN CAN0(10);         // Set CS to pin 10

//Globale Variable für CAN Botschaft
volatile byte data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
INT32U id;
volatile long unsigned int rxId;
unsigned char len = 0;
volatile unsigned char rxBuf[8];
volatile char msgString[128];      //Array für Datenempfang



//Setup
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
  pinMode(CAN0_INT, INPUT);                            //Pin für CAN receive msg ISR
  attachInterrupt(digitalPinToInterrupt(CAN0_INT), ISR_recMSG, FALLING);

  
  Serial.println("Setup Ende");
}

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


//Funktion zur Übersichtlichkeit in main()
//MSG werden nur durch inhalt von data und id bestimmt
void sendMSG(INT32U id)) //ID für MSG
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


void loop(void)
{
  data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03};
  sendMSG(0x12);
  
  data = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
  sendMSG(0x01);

}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
