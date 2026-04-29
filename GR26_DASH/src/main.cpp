// GR24 pedals node firmware
// Target platform: Any

#include <mcp_can.h>
#include <SPI.h>
#include "defs.h"
// #include <SoftPWM.h>

#define CS_CAN A2
#define CAN0_INT 1 // goes LOW when data is received
MCP_CAN CAN0(CS_CAN);

#define press_time_min 20
#define press_time_max 1000
#define hold_time_min 1500
#define hold_time_max 5000
#define spam_time 200

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
#define LED1_pin  S3//S2 TS ACTIVE R
#define LED2_pin  S2//S3 TS ACTIVE G
#define IMD_pin   S7//S7
#define LED3_pin  S9//S8 RTD ACTIVE R
#define LED4_pin  S8//S9 RTD ACTIVE G

uint32_t now = millis();
uint32_t lastSendTime = 0;
uint32_t TS_counter = 0;
uint32_t RTD_counter = 0;
bool TS_LastState = 0;
bool RTD_LastState = 0;
bool IMD_State = 0; // 0 off
bool BMS_State = 0; // 0 off


void sendData(bool TS_Active, bool RTD, bool TS_Off, bool RTD_Off, bool IMD = IMD_State, bool BMS = BMS_State){
  GRCAN_DASH_STATUS_MSG msg;
  msg.BF.BTN.TS_Active = TS_Active;
  msg.BF.BTN.RTD = RTD;
  msg.BF.BTN.TS_Off = TS_Off;
  msg.BF.BTN.RTD_Off = RTD_Off;
  msg.LB.LED.IMD = IMD;
  msg.LB.LED.BMS = BMS;
  byte sndStat = CAN0.sendMsgBuf(0x501A02, 1, 2, (uint8_t*)&msg); // extended CAN frame, 8-byte
  //Serial.println(sndStat);
  if (sndStat != CAN_OK)
    digitalWrite(S6, LOW);
  else
    digitalWrite(S6, HIGH);
}



void setup() {
  Serial.begin(9600);
  // SoftPWMBegin();
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
  CAN0.init_Mask(0, 1, 0x1FFFFFFF);              // first mask
  CAN0.init_Filt(0, 1, 0x00200300);              // Accept 0x200300.
  CAN0.init_Filt(1, 1, 0x00200300);              // Accept 0x200300.
  CAN0.init_Mask(1, 1, 0x1FFFFFFF);              // second mask
  CAN0.init_Filt(2, 1, 0x00200200);              // Accept 0x200200.
  CAN0.init_Filt(3, 1, 0x00200200);              // Accept 0x200200.
  CAN0.init_Filt(4, 1, 0x00200200);              // Accept 0x200200.
  CAN0.init_Filt(5, 1, 0x00201B05);              // Accept 0x201B05. this is dash connfig from ecu
  CAN0.setMode(MCP_NORMAL);
}



void loop() {
  now = millis();
  bool TS_State = digitalRead(S4);
  bool RTD_State = digitalRead(S10);
  // Serial.println("working");

  if(!TS_LastState && TS_State){
    //button was last pressed
    if((now - TS_counter) > press_time_min && (now - TS_counter) < press_time_max){
      //TS_On
      sendData(1, 0, 0, 0);
      Serial.println("TS on");
    }
    else if((now - TS_counter) > hold_time_min && (now - TS_counter) < hold_time_max){
      //TS_Off
      sendData(0, 0, 1, 0);
      Serial.println("TS off");
    }
  }
  else if(TS_LastState && !TS_State){
    //button was not pressed
    TS_counter = now;
  }
  if(!TS_LastState && ((now - TS_counter) > hold_time_max)){//Turn off if button is held 10s
    sendData(0, 0, 1, 0);
    TS_counter = now - hold_time_max - spam_time;
    Serial.println("TS held for 5s");
  }
  TS_LastState = TS_State;


  if(!RTD_LastState && RTD_State){
    //button was last pressed
    if((now - RTD_counter) > press_time_min && (now - RTD_counter) < press_time_max){
      //RTD_On
      sendData(0, 1, 0, 0);
      Serial.println("RTD on");
    }
    else if((now - RTD_counter) > hold_time_min && (now - RTD_counter) < hold_time_max){
      //RTD_Off
      sendData(0, 0, 0, 1);
      Serial.println("RTD off");
    }
  }
  else if(RTD_LastState && !RTD_State){
    //button was not pressed
    RTD_counter = now;
  }
  if(!RTD_LastState && ((now - RTD_counter) > hold_time_max)){//Turn off if button is held 10s
    sendData(0, 0, 0, 1);
    RTD_counter = now - hold_time_max - spam_time;
    Serial.println("RTD held for 5s");
  }
  RTD_LastState = RTD_State;

  if((now - lastSendTime) > 50)
    // sendData(0,0,0,0);

  if (!digitalRead(CAN0_INT)) {
    uint32_t rxId;
    uint8_t rxLen = 0;
    uint8_t rxBuf[8];
    CAN0.readMsgBuf(&rxId, &rxLen, rxBuf);
    rxId &= 0x1FFFFFFF;
    // for(int i = 0; i < rxLen; i++){
    //   Serial.print(rxBuf[i], HEX);
    //   Serial.print(" ");
    // }
    // Serial.println();
    // Serial.println((rxBuf[0], HEX));
    // Serial.print(F("Received CAN message with ID: 0x"));
    // Serial.println(rxId, HEX);;
    if (rxId == 0x00200200) { // Ping request from VDM
      // Respond to ping request by sending back received data
      byte sndStat = CAN0.sendMsgBuf(0x500202, 1, 4, rxBuf); // extended CAN frame, 4-byte
      if (sndStat != CAN_OK)
        digitalWrite(S6, LOW);
      else
        digitalWrite(S6, HIGH);
    }
    else if(rxId == 0x00200300){
      typedef int GR_ECU_State;
      GR_ECU_State ecu_state = rxBuf[0] & 0b00111111; //mask out reserved bits
      // GRCAN_ECU_STATUS_1_MSG* ecu_status = (GRCAN_ECU_STATUS_1_MSG*)rxBuf;
      // enum GR_ECU_State ecu_state = (GR_ECU_State)(ecu_status->ecu_state);

      Serial.println(ecu_state, BIN);
      switch (ecu_state)
      {
      case GR_GLV_OFF:
        digitalWrite(LED1_pin, LOW);
        digitalWrite(LED2_pin, LOW);
        digitalWrite(LED3_pin, LOW);
        digitalWrite(LED4_pin, LOW);
        break;
      case GR_GLV_ON:
        digitalWrite(LED1_pin, LOW);
        digitalWrite(LED2_pin, LOW);
        digitalWrite(LED3_pin, LOW);
        digitalWrite(LED4_pin, LOW);
        /* code */
        break;
      case GR_PRECHARGE_ENGAGED:
        digitalWrite(LED1_pin, HIGH);
        /* code */
        break;
      case GR_PRECHARGE_COMPLETE:
        digitalWrite(LED1_pin, LOW);
        digitalWrite(LED2_pin, HIGH);
        digitalWrite(LED3_pin, HIGH);
        /* code */
        break;
      case GR_DRIVE_ACTIVE:
        digitalWrite(LED3_pin, LOW);
        digitalWrite(LED4_pin, HIGH);
         /* code */
        
        /* code */
        break;
      case GR_TS_DISCHARGE:
        digitalWrite(LED1_pin, HIGH);
        digitalWrite(LED3_pin, HIGH);
        /* code */  
      default:
        break;
      }
      // SoftPWMSet(AMS_pin, rxBuf[0]); //analogWrite(AMS_pin, rxBuf[0]);
      // SoftPWMSet(IMD_pin, rxBuf[1]); //analogWrite(IMD_pin, rxBuf[1]);
      // SoftPWMSet(LED1_pin, rxBuf[2]); //analogWrite(LED1_pin, rxBuf[2]);
      // SoftPWMSet(LED2_pin, rxBuf[3]); //analogWrite(LED2_pin, rxBuf[3]);
      // SoftPWMSet(LED3_pin, rxBuf[4]); //analogWrite(LED3_pin, rxBuf[4]);
      // SoftPWMSet(LED4_pin, rxBuf[5]); //analogWrite(LED4_pin, rxBuf[5]);
      // analogWrite(AMS_pin, rxBuf[0]);
      // analogWrite(IMD_pin, rxBuf[1]);
      // analogWrite(LED1_pin, rxBuf[2]);
      // analogWrite(LED2_pin, rxBuf[3]);
      // analogWrite(LED3_pin, rxBuf[4]);
      // analogWrite(LED4_pin, rxBuf[5]);
    }
    else if(rxId == 0x00201B05){
      GRCAN_DASH_CONFIG_MSG* dash_config = (GRCAN_DASH_CONFIG_MSG*)rxBuf;
      // Serial.println(dash_config->led_bits, BIN);

      digitalWrite(AMS_pin, dash_config->LF.LED.BMS); //analogWrite(AMS_pin, dash_config->led_bits & 0b00000001);
      digitalWrite(IMD_pin, dash_config->LF.LED.IMD); //analogWrite
      IMD_State = dash_config->LF.LED.IMD;
      BMS_State = dash_config->LF.LED.BMS;
    }
    else {
      Serial.println(F("CAN filter isn't working"));
    }
  }
  // digitalWrite(LED1_pin, HIGH);
  // digitalWrite(LED2_pin, HIGH);
}