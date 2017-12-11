#include <mcp_can.h>
#include <SPI.h>


//Makros
MCP_CAN CAN0(10);     // Set CS to pin 10


//Setup
void setup()
{
  Serial.begin(115200);
  Serial.println("Setup Anfang");

  // Initialize MCP2515 running at 8MHz with a baudrate of 50kb/s and the masks and filters disabled.
  // MCP_ANY(canIDMode):CAN configuration für Tranceiver;  CAN_50KBPS(canSpeed): CAN Geschwindigkeit; MCP_8MHZ(canClock):verbauter Quarz auf Board;
  if(CAN0.begin(MCP_ANY, CAN_50KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
    else Serial.println("Fehler bei der Initialisierung des CAN Tranceiver...");
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  
  Serial.println("Setup Ende");
}

//Globale Variable für CAN Botschaft
byte data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
INT32U id;

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


void loop()
{
  data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03};
  sendMSG(0x12);
  
  data = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
  sendMSG(0x01);

}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
