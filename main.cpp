// GR24 pedals node firmware
// Target platform: Any

#include <mcp_can.h>
#include <SPI.h>

#define CS_CAN A2
#define CAN0_INT 1 // goes LOW when data is received
MCP_CAN CAN0(CS_CAN);

#define S1 A5
#define S2 A4
#define S3 A3
#define S4 13
#define S5 5
#define S6 10
#define S7 9
#define S8 6
#define S9 3
#define S10 11

#define AMS_pin   S1//S1
#define LED1_pin  S3//S2
#define LED2_pin  S2//S3
#define IMD_pin   S7//S7
#define LED3_pin  S9//S8
#define LED4_pin  S8//S9

uint32_t now = millis();
uint32_t lastSendTime = 0;
uint32_t TS_counter = 0;
uint32_t RTD_counter = 0;
bool TS_LastState = 0;
bool RTD_LastState = 0;

void sendData(byte TS_Active, byte TS_Off, byte RTD_On, byte RTD_Off){
  uint8_t data[8];
  lastSendTime = now; 
  data[0] = TS_Active;
  data[1] = TS_Off;
  data[2] = RTD_On;
  data[3] = RTD_Off;
  data[4] = digitalRead(AMS_pin);
  data[5] = digitalRead(IMD_pin);;
  data[6] = 0;
  data[7] = 0;
  byte sndStat = CAN0.sendMsgBuf(0x13000, 1, 8, data); // extended CAN frame, 8-byte
  //Serial.println(sndStat);
  if (sndStat != CAN_OK)
    digitalWrite(S6, LOW);
  else
    digitalWrite(S6, HIGH);
}



void setup() {
  Serial.begin(9600);
  pinMode(CS_CAN, OUTPUT);
  digitalWrite(CS_CAN, HIGH);

  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(S4, INPUT_PULLUP); // TS
  pinMode(S6, OUTPUT);
  pinMode(S7, OUTPUT);
  pinMode(S8, OUTPUT);
  pinMode(S9, OUTPUT);
  pinMode(S10, INPUT_PULLUP); //RTD


  // MCP2515 Init
  while (CAN0.begin(MCP_STDEXT, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) { // max SPI clock is 1/4 master clock
    Serial.println(F("MCP2515 init failed"));
    delay(1000);
  }
  CAN0.init_Mask(0, 1, 0x1FFFFFFF);              // Init first mask...
  CAN0.init_Filt(0, 1, 0x00013001);              // Init first filter...
  CAN0.init_Filt(1, 1, 0x00013001);              // Init second filter...
  CAN0.init_Mask(1, 1, 0x1FFFFFFF);              // Init second mask...
  CAN0.init_Filt(2, 1, 0x00012FFE);              // Init third filter...
  CAN0.init_Filt(3, 1, 0x00012FFE);              // Init fourth filter...
  CAN0.init_Filt(4, 1, 0x00012FFE);              // Init fifth filter...
  CAN0.init_Filt(5, 1, 0x00012FFE);              // Init sixth filter...
  CAN0.setMode(MCP_NORMAL);
}



void loop() {
  now = millis();
  bool TS_State = digitalRead(S4);
  bool RTD_State = digitalRead(S10);

  if(!TS_LastState && TS_State){
    //button was last pressed
    if((now - TS_counter) > 20 && (now - TS_counter) < 250){
      //TS_On
      sendData(1,0,0,0);
      Serial.println("TS on");
    }
    else if((now - TS_counter) > 500 && (now - TS_counter) < 5000){
      //TS_Off
      sendData(0,1,0,0);
      Serial.println("TS off");
    }
  }
  else if(TS_LastState && !TS_State){
    //button was not pressed
    TS_counter = now;
  }
  if(!TS_LastState && ((now - TS_counter) > 5000)){//Turn off if button is held 10s
    sendData(0,1,0,0);
    TS_counter = now - 4800;
    Serial.println("TS held for 5s");
  }
  TS_LastState = TS_State;


  if(!RTD_LastState && RTD_State){
    //button was last pressed
    if((now - RTD_counter) > 20 && (now - RTD_counter) < 250){
      //RTD_On
      sendData(0,0,1,0);
      Serial.println("RTD on");
    }
    else if((now - RTD_counter) > 500 && (now - RTD_counter) < 5000){
      //RTD_Off
      sendData(0,0,0,1);
      Serial.println("RTD off");
    }
  }
  else if(RTD_LastState && !RTD_State){
    //button was not pressed
    RTD_counter = now;
  }
  if(!RTD_LastState && ((now - RTD_counter) > 5000)){//Turn off if button is held 10s
    sendData(0,0,0,1);
    RTD_counter = now - 4800;
    Serial.println("RTD held for 5s");
  }
  RTD_LastState = RTD_State;

  if((now - lastSendTime) > 50)
    sendData(0,0,0,0);

  if (!digitalRead(CAN0_INT)) {
    uint32_t rxId;
    uint8_t rxLen = 0;
    uint8_t rxBuf[8];
    CAN0.readMsgBuf(&rxId, &rxLen, rxBuf);
    rxId &= 0b01111111111111111111111111111111;
    if (rxId == 0x12FFE) { // Ping request from VDM
      // Respond to ping request by sending back received data
      byte sndStat = CAN0.sendMsgBuf(0x12FFF, 1, 8, rxBuf); // extended CAN frame, 4-byte
      if (sndStat != CAN_OK)
        digitalWrite(S6, LOW);
      else
        digitalWrite(S6, HIGH);
    }
    else if(rxId == 0x13001){
      analogWrite(AMS_pin, rxBuf[0]);
      analogWrite(IMD_pin, rxBuf[1]);
      analogWrite(LED1_pin, rxBuf[2]);
      analogWrite(LED2_pin, rxBuf[3]);
      analogWrite(LED3_pin, rxBuf[4]);
      analogWrite(LED4_pin, rxBuf[5]);
    }
    else {
      Serial.println(F("CAN filter isn't working"));
    }
  }
}