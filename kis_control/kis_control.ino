
#include <EEPROM.h>

//#define _NAMIKI_MOTOR   //for Namiki 22CL-103501PG80:1
/********************************************************************/
/*
                Power Switch
                Sonar0x11
         -------------------------
        /                         \
             /               \
            /                 \
      M3   /                   \ M2
           INT0 /                   \INT1
         /                     \
        /                     \
       /                       \
       \                       /
        \                     /
           \                     /
          \                   /
      Sonar0x12  \                     / Sonar0x13
            \                       /
             \                     /
        --------------------------
              M1
*/


#include <fuzzy_table.h>
#include <PID_Beta6.h>

#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>

#include <MotorWheel.h>
#include <Omni3WD.h>

#include <SONAR.h>
#define TIMESTEP 3000
/******************************************/
// SONAR

SONAR sonar11(0x11), sonar12(0x12), sonar13(0x13);

unsigned short distBuf[3];
void sonarsUpdate() {
  static unsigned char sonarCurr = 1;
  if (sonarCurr == 3) sonarCurr = 1;
  else ++sonarCurr;
  if (sonarCurr == 1) {
    distBuf[1] = sonar12.getDist();
    sonar12.trigger();
    sonar12.showDat();
  } else if (sonarCurr == 2) {
    distBuf[2] = sonar13.getDist();
    sonar13.trigger();
    sonar13.showDat();
  } else {
    distBuf[0] = sonar11.getDist();
    sonar11.trigger();
    sonar11.showDat();
  }
}

/*********************************************/
//Delay
String inputString = ""; // a string to hold incoming data
boolean stringComplete = false; // whether the string is complete
unsigned long starttime = 0;
bool timing = false;
float t_k;
float *tk = &t_k;
float c0_1, c0_2, c1_1, c1_2, c2_1, c2_2, c3_1, c3_2;
float *c01 = &c0_1, *c02 = &c0_2, *c11 = &c1_1, *c12 = &c1_2;
float *c21 = &c2_1, *c22 = &c2_2, *c31 = &c3_1, *c32 = &c3_2;
float q1 = 0.0f;
float q2 = 0.0f;
float target_1 = 0.0f, target_2 = 0.0f;
float v_d1 = 0.0f;
float v_d2 = 0.0f;
float u1 = 0.0f, u2 = 0.0f, u3 = 0.0f;

/*******************************************/
//cubic trajectory generation

//general cubic trajectory generation
void cubic(float q_i, float q_f, float v_i, float v_f, float tkk, float *t_kk, float *c_0, float *c_1, float *c_2, float *c_3)
{
  *c_0 = q_i;
  *c_1 = v_i;
  *c_2 = 3.0f * q_f - 3.0f * q_i - v_f - 2.0f * v_i;
  *c_3 = 2.0f * q_i - 2.0f * q_f + v_f + v_i;
  *t_kk = tkk;
}

//x cubic trajectory
void cubic1(float tar_q, float in_v, float tar_v, float tkkk)
{
  cubic(q1, tar_q, in_v, tar_v, tkkk, tk, c01, c11, c21, c31);
  target_1 = tar_q;
}
//y cubic trajectory
void cubic2(float tar_q, float in_v, float tar_v, float tkkk)
{
  cubic(q2, tar_q, in_v, tar_v, tkkk, tk, c02, c12, c22, c32);
  target_2 = tar_q;
}
/*******************************************/
// Motors

irqISR(irq1, isr1);
MotorWheel wheel1(9, 8, 6, 7, &irq1);    // Pin9:PWM, Pin8:DIR, Pin6:PhaseA, Pin7:PhaseB

irqISR(irq2, isr2);
MotorWheel wheel2(10, 11, 14, 15, &irq2); // Pin10:PWM, Pin11:DIR, Pin14:PhaseA, Pin15:PhaseB

irqISR(irq3, isr3);
MotorWheel wheel3(3, 2, 4, 5, &irq3);    // Pin3:PWM, Pin2:DIR, Pin4:PhaseA, Pin5:PhaseB

Omni3WD Omni(&wheel1, &wheel2, &wheel3);
/******************************************/

/******************************************/
// demo
unsigned long currMillis = 0;
void demoWithSensors(unsigned int speedMMPS, unsigned int distance) {
  if (millis() - currMillis > SONAR::duration) {
    currMillis = millis();
    sonarsUpdate();
  }

  if (distBuf[1] < distance) Omni.setCarSlow2Stop(500);
  else if (distBuf[2] < distance) Omni.setCarSlow2Stop(500);
  else if (distBuf[0] < distance) Omni.setCarSlow2Stop(500);

}

/******************************************/
//serial communication protocol
char data;
//checksum function
bool checkSum() {
  char sum = 0;
  char check = inputString[9];
  for (int i = 0; i < 9; i++) {
    sum = sum + inputString[i];
  }
  return sum == check;
}

//merge ints from msb and lsb from incoming data
int mergeInts(int MSB, int LSB)
{
  long a = (256 * (int)(unsigned char)MSB) + (unsigned char)LSB;
  Serial.println(a);
  return a;
}

/*****************************************/
// setup()
void setup() {
  TCCR1B = TCCR1B & 0xf8 | 0x01; // Pin9,Pin10 PWM 31250Hz
  TCCR2B = TCCR2B & 0xf8 | 0x01; // Pin3,Pin11 PWM 31250Hz

  //  SONAR::init(13);
  //  wheel1.PIDEnable(0.26, 0.02, 0, 10); // used whewl1 to call the PIDEnable
  //  wheel2.PIDEnable(0.26, 0.02, 0, 10); // used whewl1 to call the PIDEnable
  //  wheel3.PIDEnable(0.26, 0.02, 0, 10); // used whewl1 to call the PIDEnable
  Omni.PIDEnable(0.26, 0.02, 0, 10);

  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
}

/****************************************/
// loop()
void loop() {
  if (stringComplete) {
    //perform checksum
    bool checksum = checkSum();
    if (checksum) {
      Serial.println("received");
      switch (inputString[2]) {
        case 0:
          Omni.setMotorAllStop();
          break;
        case 1:
          float x = mergeInts((int)inputString[4], (int)inputString[5]);
          float y = mergeInts((int)inputString[7], (int)inputString[8]);
          if (inputString[3]) {
            x = 0 - x;
          }
          if (inputString[6]) {
            y = 0 - y;
          }
          cubic1(x, v_d1, 0, TIMESTEP);
          cubic2(y, v_d2, 0, TIMESTEP);
          starttime = millis();
          timing = true;
      }
    }
    else {
      Serial.println("resend");
    }
    //    Serial.println(inputString);
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
  if (millis() - starttime >= TIMESTEP && timing) {
    Omni.setMotorAllStop();
    v_d1 = 0.0f;
    v_d2 = 0.0f;
    timing = false;
  }
  else if (timing) {
    float tau = (millis() - starttime) / (float)TIMESTEP;
    v_d1 = c1_1 + 2.0f * c2_1 * tau + 3.0f * c3_1 * tau * tau;
    v_d2 = c1_2 + 2.0f * c2_2 * tau + 3.0f * c3_2 * tau * tau;
    u1 = v_d1;
    u2 = -0.5f * v_d1 - v_d2 * sin(PI / 3);
    u3 = -0.5f * v_d1 + v_d2 * sin(PI / 3);
    if (u1 > 0) {
      Omni.wheelBackSetSpeedMMPS((int)u1, DIR_ADVANCE);
    }
    else if (u1 < 0) {
      Omni.wheelBackSetSpeedMMPS((int)(0 - u1), DIR_BACKOFF);
    }
    if (u2 > 0) {
      Omni.wheelLeftSetSpeedMMPS((int)u2, DIR_ADVANCE);
    }
    else if (u2 < 0) {
      Omni.wheelLeftSetSpeedMMPS((int)(0 - u2), DIR_BACKOFF);
    }
    if (u3 > 0) {
      Omni.wheelRightSetSpeedMMPS((int)u3, DIR_ADVANCE);
    }
    else if (u3 < 0) {
      Omni.wheelRightSetSpeedMMPS((int)(0 - u3), DIR_BACKOFF);
    }
    //    if (v_d1 > 0) {
    //      Omni.setCarAdvance((int)v_d1);
    //      //      Serial.println(v_d1);
    //    }
    //    else if (v_d1 < 0) {
    //      Omni.setCarBackoff((int)(0 - v_d1));
    //      //      Serial.println(v_d1);
    //    }
    //    Serial.print("u1 = ");
    //    Serial.print(u1, DEC);
    //    Serial.print(", u2 = ");
    //    Serial.print(u2, DEC);
    //    Serial.print(", u3 = ");
    //    Serial.println(u3, DEC);
    Omni.PIDRegulate();
    //    delay(10);
  }
}
//char prevChar = '0';
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inputString.length() == 11 && inChar == '\n') {
      stringComplete = true;
    }
//    prevChar = inChar;
  }
}
