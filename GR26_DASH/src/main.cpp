// GR24 pedals node firmware
// Target platform: Any

#include <mcp_can.h>
#include <SPI.h>
#include "defs.h"
#include <FastLED.h>

#define CS_CAN A2
#define CAN0_INT 1 // goes LOW when data is received
MCP_CAN CAN0(CS_CAN);

#define press_time_min 20
#define press_time_max 200
#define hold_time_min 200
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

#define TS_BTN      S2
#define RTD_BTN     S3
#define BTN_LED     S1
#define ERROR_LED   S9

#define NUM_BTN_LED     2
#define NUM_ERROR_LED   3

uint32_t now = millis();
uint32_t lastSendTime = 0;
uint32_t lastRecTime = 0;
uint32_t TS_counter = 0;
uint32_t RTD_counter = 0;
bool TS_LastState = 0;
bool RTD_LastState = 0;
bool IMD_State = 0; // 0 off
bool BMS_State = 0; // 0 off
bool BSPD_State = 0; // 0 off
bool BMS_Latch_State = 0;
bool IMD_Latch_State = 0;
bool BSPD_Latch_State = 0;

CRGB btn_leds[NUM_BTN_LED];
CRGB error_leds[NUM_ERROR_LED];
int randNumber1;
int randNumber2;

CRGB nvidia_green = CRGB(5, 100, 2);

bool rainbow = false;
bool flash = false;
bool flash2 = false;
bool reset_car_pls = false;
bool precharge = false;
bool discharge = false;



void sendData(bool TS_Active, bool RTD, bool TS_Off, bool RTD_Off, bool IMD = IMD_State, bool BMS = BMS_State, bool BSPD = BSPD_State){
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

void setButtonLED(CRGB color1, CRGB color2) {
  btn_leds[0] = color1;
  btn_leds[1] = color2;
}

void updateErrorLEDs() {
  error_leds[0] = BSPD_Latch_State ? CRGB::Black : CRGB::Red;
  error_leds[1] = IMD_Latch_State ? CRGB::Black : CRGB::Red;
  error_leds[2] = BMS_Latch_State ? CRGB::Black : CRGB::Red;
}

void startupTest() {
  const unsigned int duration = 1500;
  unsigned long startTime = millis();

  CRGB baseColor = CRGB(0, 50, 120);

  const int num_btn = 2;
  const int num_err = 3;

  float sigma = 0.8;
  float offset = 2.0;  // how far "offscreen" the Gaussian starts/ends

  while ((millis() - startTime) < duration) {
    float t = (millis() - startTime) / (float)duration;  // 0 → 1

    // Optional smoothing (looks much nicer)
    t = t * t * (3 - 2 * t);

    // Sweep from before first LED → past last LED
    float center_btn = -offset + t * ((num_btn - 1) + 2 * offset);
    float center_err = -offset + t * ((num_err - 1) + 2 * offset);

    // --- Button LEDs ---
    for (int i = 0; i < num_btn; i++) {
      float d = (num_btn-i-1) - center_btn;
      float brightness = exp(-(d * d) / (2 * sigma * sigma));

      btn_leds[i] = baseColor;
      btn_leds[i].nscale8(brightness * 255);
    }

    // --- Error LEDs ---
    for (int i = 0; i < num_err; i++) {
      float d = (num_err-i-1) - center_err;
      float brightness = exp(-(d * d) / (2 * sigma * sigma));

      error_leds[i] = baseColor;
      error_leds[i].nscale8(brightness * 255);
    }

    FastLED.show();
    delay(10);
  }

  fill_solid(btn_leds, num_btn, CRGB::Black);
  fill_solid(error_leds, num_err, CRGB::Black);
  FastLED.show();
  delay(200);
}

void setup() {
  Serial.begin(9600);

  randomSeed(analogRead(0));
  randNumber1 = random(255);
  randNumber2 = random(255);


  // SoftPWMBegin();
  pinMode(CS_CAN, OUTPUT);
  digitalWrite(CS_CAN, HIGH);

  pinMode(TS_BTN, INPUT_PULLUP); //BTN input
  pinMode(RTD_BTN, INPUT_PULLUP);

  FastLED.addLeds<WS2812, BTN_LED, GRB>(btn_leds, NUM_BTN_LED);
  FastLED.addLeds<WS2812, ERROR_LED, GRB>(error_leds, NUM_ERROR_LED);


  // MCP2515 Init
  while (CAN0.begin(MCP_STDEXT, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) { // max SPI clock is 1/4 master clock
    Serial.println(F("MCP2515 init failed"));
    delay(1000);
  }
  CAN0.init_Mask(0, 1, 0x1FFFFFFF);              // first mask
  CAN0.init_Filt(0, 1, 0x00200300);              // Accept 0x200300.
  CAN0.init_Filt(1, 1, 0x00300300);              // Accept 0x200300.
  CAN0.init_Mask(1, 1, 0x1FFFFFFF);              // second mask
  CAN0.init_Filt(2, 1, 0x00200200);              // Accept 0x200200.
  CAN0.init_Filt(3, 1, 0x00200200);              // Accept 0x200200.
  CAN0.init_Filt(4, 1, 0x00300702);              // Accept 0x200200.
  CAN0.init_Filt(5, 1, 0x00201B05);              // Accept 0x201B05. this is dash connfig from ecu
  CAN0.setMode(MCP_NORMAL);

  startupTest();
}



void loop() {
  now = millis();
  float breathe_brightness = sin(millis()/400.0)*sin(millis()/400.0);
  bool TS_State = digitalRead(TS_BTN);
  bool RTD_State = digitalRead(RTD_BTN);
  flash = false;
  flash2 = false;
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
  if(!TS_LastState && ((now - TS_counter) > hold_time_max)){//Turn off if button is held 5s
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
  if(!RTD_LastState && ((now - RTD_counter) > hold_time_max)){//Turn off if button is held 5s
    sendData(0, 0, 0, 1);
    RTD_counter = now - hold_time_max - spam_time;
    Serial.println("RTD held for 5s");
  }
  RTD_LastState = RTD_State;

  // if((now - lastSendTime) > 50)    //Heartbeat
    // sendData(0,0,0,0);

  if (!digitalRead(CAN0_INT)) {
    uint32_t rxId;
    uint8_t rxLen = 0;
    uint8_t rxBuf[8];
    CAN0.readMsgBuf(&rxId, &rxLen, rxBuf);
    rxId &= 0x1FFFFFFF;

    if (rxId == 0x00200200) { // Ping request from VDM
      // Respond to ping request by sending back received data
      byte sndStat = CAN0.sendMsgBuf(0x500202, 1, 4, rxBuf); // extended CAN frame, 4-byte
      if (sndStat != CAN_OK)
        digitalWrite(S6, LOW);
      else
        digitalWrite(S6, HIGH);
    }
    else if(rxId == 0x00200300){  //ECU State
      lastRecTime = millis();
      typedef int GR_ECU_State;
      GR_ECU_State ecu_state = rxBuf[0] & 0b00111111; //mask out reserved bits
      // GRCAN_ECU_STATUS_1_MSG* ecu_status = (GRCAN_ECU_STATUS_1_MSG*)rxBuf;
      // enum GR_ECU_State ecu_state = (GR_ECU_State)(ecu_status->ecu_state);
      CRGB breathe_red = CRGB::Red;
      CRGB breathe_green = nvidia_green;
      CRGB breathe_blue = CRGB(0, 50, 120);
      breathe_red = breathe_red.nscale8(breathe_brightness * 255);
      breathe_green = breathe_green.nscale8(breathe_brightness * 255);
      breathe_blue = breathe_blue.nscale8(breathe_brightness * 255);

    
      rainbow = false;
      reset_car_pls = false;
      bool all_latched = BMS_Latch_State && IMD_Latch_State && BSPD_Latch_State;
      bool all_good = !BMS_State && !IMD_State && !BSPD_State;
      precharge = false;
      discharge = false;
      // Serial.println(ecu_state);
      switch (ecu_state)
      {
        case GR_GLV_ON:
          // rainbow = true;
          setButtonLED(CRGB::Black, breathe_blue);
          if(all_good && !all_latched)
            reset_car_pls = true;
          break;

        case GR_PRECHARGE_ENGAGED:
          //setButtonLED(CRGB::Black, CRGB::Red);
          precharge = true;
          break;

        case GR_PRECHARGE_COMPLETE:
          setButtonLED(breathe_blue, nvidia_green);
          break;

        case GR_DRIVE_ACTIVE:
          setButtonLED(nvidia_green, nvidia_green);
          break;

        case GR_TS_DISCHARGE:
          discharge = true;
          break;

        default:
          break;
      }
    }
    else if(rxId == 0x00201B05){
      GRCAN_DASH_CONFIG_MSG* dash_config = (GRCAN_DASH_CONFIG_MSG*)rxBuf;

 
      IMD_State = dash_config->LF.LED.IMD;
      BMS_State = dash_config->LF.LED.BMS;
      BSPD_State = dash_config->LF.LED.BSPD;
      
      IMD_Latch_State = dash_config->LF.LED.IMD_Latch;
      BMS_Latch_State = dash_config->LF.LED.BMS_Latch;
      BSPD_Latch_State = dash_config->LF.LED.BSPD_Latch;

      updateErrorLEDs();
    }
    else if(rxId == 0x00300702){
      unsigned int Bat_Voltage = rxBuf[0] + (((int)rxBuf[1])<<8);
      unsigned int TS_Voltage = rxBuf[2] + (((int)rxBuf[3])<<8);
      float brightness = min((float)TS_Voltage/(float)Bat_Voltage, 1.0);

      if(precharge){        
        CRGB greenish = nvidia_green;
        greenish = greenish.nscale8(brightness *brightness *brightness*brightness * 255);
        setButtonLED(CRGB::Black, greenish);
      }
      else if(discharge){
        float brightness = constrain(((float)TS_Voltage - 6000.0f)/(float)Bat_Voltage, 0.0f, 1.0f);
        CRGB orangeish = CRGB::Orange;
        orangeish = orangeish.nscale8(brightness *brightness *brightness * 255);
        setButtonLED(orangeish, orangeish);
      }
    }
    else {
      Serial.println(F("CAN filter isn't working"));
    }
  }

  if(millis() - lastRecTime > 100) // stale date
    flash = true;

  if((BMS_State || IMD_State ||BSPD_State))
    flash2 = true;

  
  if(flash){
    CRGB flasher = CRGB::Red;
    float t = millis() / 50.0;
    float blink = pow(sin(t), 4);
    float gate = sin(t / 3.0) > 0 ? 1.0 : 0.0;
    float flash_brightness = blink * gate;
    flasher = flasher.nscale8(flash_brightness * 255);
    
    btn_leds[0] = flasher;
    btn_leds[1] = flasher;

    error_leds[0] = CRGB::Black;
    error_leds[1] = CRGB::Black;
    error_leds[2] = CRGB::Black;
  }
  else if(flash2){
    CRGB flasher = CRGB::Red;
    float flash_brightness = pow(sin(millis()/50.0), 4);
    flasher = flasher.nscale8(flash_brightness * 255);
    
    btn_leds[0] = flasher;
    btn_leds[1] = flasher;
  }
  else if(reset_car_pls){
    CRGB flasher = CRGB::Purple;
    float flash_brightness = sin(millis()/500.0)*sin(millis()/500.0)*sin(millis()/500.0)*sin(millis()/500.0);
    flasher = flasher.nscale8(flash_brightness * 255);
    
    btn_leds[0] = flasher;
    btn_leds[1] = flasher;
  }
  else if(rainbow){
    btn_leds[0] = CHSV((millis() / 16) + randNumber1, 255, 255);
    btn_leds[1] = CHSV((millis() / 17) + randNumber2, 255, 255);

    float brightness = sin(millis()/400.0)*sin(millis()/400.0);
    btn_leds[1].nscale8(brightness * 255);
  }
  FastLED.show();
}