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

char array[20];
char SM_id = 0;
int getPackage = 0;
char command_ID;
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
float q[10];
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
//Communication

// store data from pc
void SM_RxD(int c)
{
  if (getPackage == 0) {
    if (SM_id < 2) {
      if (c == 255) {
        array[SM_id] = c;
        SM_id++;
      } else {
        SM_id = 0;
      }
    } else if (SM_id == 2) {
      array[SM_id] = c;
      command_ID = c;
      SM_id++;
    } else if (SM_id > 2) {
      array[SM_id] = c;
      if (SM_id >= 9) {
        getPackage = 1;
        SM_id = 0;
      } else {
        SM_id++;
      }
    }
  }
}
// performs checksum
int sumCheck()
{
  char sum = 0;
  char checksum = array[10];
  for (int i = 0; i < 9; i++) {
    sum = sum + (char)array[i];
  }
  sum = (char)sum;
  if (sum == checksum) {
    return 1;
  } else {
    return 0;
  }
}

int mergeInts(int MSB, int LSB)
{
  long a = (256 * (int)(unsigned char)MSB) + (unsigned char)LSB;
//  printf("merged %d and %d into: %d \n", MSB, LSB, a);
  return a;
}

float intsToFloat(unsigned char LSB, unsigned char hexadec)
{
  float flo = (float)LSB + ((float)hexadec) / 256;
//  printf("merged %d and %d into: %0.2f\n", LSB, hexadec, flo);
  return flo;
}




char data;
/*****************************************/
// setup()
void setup() {
  TCCR1B = TCCR1B & 0xf8 | 0x01; // Pin9,Pin10 PWM 31250Hz
  TCCR2B = TCCR2B & 0xf8 | 0x01; // Pin3,Pin11 PWM 31250Hz

  //  SONAR::init(13);
  Omni.PIDEnable(0.26, 0.02, 0, 10);

  Serial.begin(115200);
}

/****************************************/
// loop()
void loop() {

  if (Serial.available() > 0) {

    data = Serial.read();
    SM_RxD(data);
  }
  if (getPackage) {
    int received = sumCheck();
    if (!received) {
      Serial.println("res");
      getPackage = 0;
    }
    else {
      Serial.println("received");
      switch (array[2]) {
        case 0:
          Omni.setMotorAllStop();
          v_d1 = 0.0f;
          v_d2 = 0.0f;
          timing = false;
          getPackage = 0;
          break;
        case 1:
          
          for (int i = 1; i < 3; i++) {
            q[i - 1] = mergeInts(array[3 * i + 1], array[ 3 * i + 2]);
            if (array[3 * i]) {
              q[i - 1] = 0 - q[i - 1];
            }
          }
          cubic1(q[0], v_d1, 0, TIMESTEP);
          cubic2(q[1], v_d2, 0, TIMESTEP);
          starttime = millis();
          timing = true;
      }
    }
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
