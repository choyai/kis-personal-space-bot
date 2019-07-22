
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

#include <PID_Beta6.h>
#include <fuzzy_table.h>

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
  if (sonarCurr == 3)
    sonarCurr = 1;
  else
    ++sonarCurr;
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
// Delay
String inputString = "";        // a string to hold incoming data
boolean stringComplete = false; // whether the string is complete
unsigned long starttime = 0;
bool timing = false;
float t_k;
float *tk = &t_k;
float c0_1, c0_2, c0_3, c1_1, c1_2, c1_3, c2_1, c2_2, c2_3, c3_1, c3_2, c3_3;
float *c01 = &c0_1, *c02 = &c0_2, *c03 = &c0_3, *c11 = &c1_1, *c12 = &c1_2, *c13 = &c1_3;
float *c21 = &c2_1, *c22 = &c2_2, *c23 = &c2_3, *c31 = &c3_1, *c32 = &c3_2, *c33 = &c3_3;
float q1 = 0.0f;//x
float q2 = 0.0f;//y
float q3 = 0.0f;//theta
float r_1 = 0.0f;
float r_2 = 0.0f;
float r_3 = 0.0f;
float r1 = 0.0f, r2 = 0.0f, r3 = 0.0f;
float target_1 = 0.0f, target_2 = 0.0f, target_3 = 0.0f;
float v_d1 = 0.0f;//v_x
float v_d2 = 0.0f;//v_y
float v_d3 = 0.0f;//omega
float u_d1 = 0.0f, u_d2 = 0.0f, u_d3 = 0.0f;
float *vd1 = &u_d1, *vd2 = &u_d2, *vd3 = &u_d3;
float u1 = 0.0f, u2 = 0.0f, u3 = 0.0f;
float *u_1 = &u1, *u_2 = &u2, *u_3 = &u3;
float q[10];
float d = 25 + 150 * cos(PI / 6);
float x, y, omega;
float Kp = 0.50f, Ki = 0.0f, Kd = 0.0f;
float s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
float *s_1 = &s1, *s_2 = &s2, *s_3 = &s3;
float p1 = 0.0f, p2 = 0.0f, p3 = 0.0f;
float *p_1 = &p1, *p_2 = &p2, *p_3 = &p3;
/*******************************************/
// cubic trajectory generation

// general cubic trajectory generation
void cubic(float q_i, float q_f, float v_i, float v_f, float tkk, float *t_kk,
           float *c_0, float *c_1, float *c_2, float *c_3) {
  *c_0 = q_i;
  *c_1 = v_i;
  *c_2 = 3.0f * q_f - 3.0f * q_i - v_f - 2.0f * v_i;
  *c_3 = 2.0f * q_i - 2.0f * q_f + v_f + v_i;
  *t_kk = tkk;
}

// x cubic trajectory
void cubic1(float tar_q, float in_v, float tar_v, float tkkk) {
  cubic(q1, tar_q, in_v, tar_v, tkkk, tk, c01, c11, c21, c31);
  target_1 = tar_q;
}
// y cubic trajectory
void cubic2(float tar_q, float in_v, float tar_v, float tkkk) {
  cubic(q2, tar_q, in_v, tar_v, tkkk, tk, c02, c12, c22, c32);
  target_2 = tar_q;
}
// theta cubic trajectory
void cubic3(float tar_q, float in_v, float tar_v, float tkkk) {
  cubic(q3, tar_q, in_v, tar_v, tkkk, tk, c03, c13, c23, c33);
  target_3 = tar_q;
}

// General PID
void GenPID(float pos, float q, float *s, float *p, float *u, float Kp,
            float Ki, float Kd, float v_d)
{
  float e = pos - q;
  *s = *s + e;
  //  *u = (Kp * e) + (Ki * *s) + (Kd * (v_d - (e - *p)));
  *u = (Kp * e) + (Ki * *s) - (Kd * (e - *p));
  *p = e;
}

/*******************************************/
// Motors

irqISR(irq1, isr1);
MotorWheel wheel1(9, 8, 6, 7,
                  &irq1); // Pin9:PWM, Pin8:DIR, Pin6:PhaseA, Pin7:PhaseB

irqISR(irq2, isr2);
MotorWheel wheel2(10, 11, 14, 15,
                  &irq2); // Pin10:PWM, Pin11:DIR, Pin14:PhaseA, Pin15:PhaseB

irqISR(irq3, isr3);
MotorWheel wheel3(3, 2, 4, 5,
                  &irq3); // Pin3:PWM, Pin2:DIR, Pin4:PhaseA, Pin5:PhaseB

Omni3WD Omni(&wheel1, &wheel2, &wheel3);
/******************************************/


/******************************************/
// serial communication protocol
char data;
// checksum function
bool checkSum() {
  char sum = 0;
  char check = inputString[12];
  for (int i = 0; i < 12; i++) {
    sum = sum + inputString[i];
  }
  return sum == check;
}

// merge ints from msb and lsb from incoming data
int mergeInts(int MSB, int LSB) {
  long a = (256 * (int)(unsigned char)MSB) + (unsigned char)LSB;
  //  Serial.println(a);
  return a;
}

/*****************************************/
// setup()
void setup() {
  TCCR1B = TCCR1B & 0xf8 | 0x01; // Pin9,Pin10 PWM 31250Hz
  TCCR2B = TCCR2B & 0xf8 | 0x01; // Pin3,Pin11 PWM 31250Hz

  //  SONAR::init(13);
  Omni.PIDEnable(0.26, 0.02, 0, 10);

  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
}

/****************************************/
// loop()
void loop() {
  if (stringComplete) {
    // perform checksum
    if (checkSum()) {
      //            Serial.println("received");
      switch (inputString[2]) {
        case 0:
          Omni.setMotorAllStop();
          v_d1 = 0.0f;
          v_d2 = 0.0f;
          v_d3 = 0.0f;
          timing = false;
          break;
        case 1:
          x = mergeInts((int)inputString[4], (int)inputString[5]);
          y = mergeInts((int)inputString[7], (int)inputString[8]);
          omega = mergeInts((int)inputString[10], (int)inputString[11]);
          if (inputString[3]) {
            x = 0 - x;
          }
          if (inputString[6]) {
            y = 0 - y;
          }
          if (inputString[9]) {
            omega = 0 - omega;
          }
          Serial.println("going");
          cubic1(x, v_d1, 0, TIMESTEP);
          cubic2(y, v_d2, 0, TIMESTEP);
          cubic3(omega, v_d3, 0, TIMESTEP);
          starttime = millis();
          timing = true;
          //                    Serial.print("target angle = ");
          //                    Serial.println(omega, DEC);
          break;
        case 2:
          q1 = mergeInts((int)inputString[4], (int)inputString[5]);
          //                    Serial.print("set x to ");
          //                    Serial.print(q1, DEC);
          q2 = mergeInts((int)inputString[7], (int)inputString[8]);
          //                    Serial.print(", set y to ");
          //                    Serial.print(q2, DEC);
          q3 = mergeInts((int)inputString[10], (int)inputString[11]);
          //                    Serial.print(", set angle to ");
          //                    Serial.println(q3, DEC);
          break;
        default:
          break;
      }
    } else {
      Serial.println("resend");
    }
    //    Serial.println(inputString);
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
  if ((abs(target_1 - q1) < 30 && abs(target_2 - q2) < 30 && abs(target_3 - q3) < 30) || millis() - starttime >= TIMESTEP && timing) {
    Omni.setMotorAllStop();
    v_d1 = 0.0f;
    v_d2 = 0.0f;
    v_d3 = 0.0f;
    //    target_1 = q1, target_2 = q2, target_3 = q3;
    timing = false;
  } else if (timing) {
    //    float tau = (millis() - starttime) / (float)TIMESTEP;
    //            r_1 = c0_1 + c1_1 * tau + c2_1 * tau * tau + c3_1 * tau * tau * tau;
    //            r_2 = c0_2 + c1_2 * tau + c2_2 * tau * tau + c3_2 * tau * tau * tau;
    //            r_3 = c0_3 + c1_3 * tau + c2_3 * tau * tau + c3_3 * tau * tau * tau;
    //    r_3 = 0.0f;
    //
    //    v_d1 = c1_1 + 2.0f * c2_1 * tau + 3.0f * c3_1 * tau * tau;
    //    v_d2 = c1_2 + 2.0f * c2_2 * tau + 3.0f * c3_2 * tau * tau;
    //    v_d3 = c1_3 + 2.0f * c2_3 * tau + 3.0f * c3_3 * tau * tau;
    //        v_d3 = 0.0f;
    //    GenPID(target_1 , q1, s_1, p_1, vd1, Kp, Ki, Kd, v_d1);
    //    GenPID(target_2, q2, s_2, p_2, vd2, Kp, Ki, Kd, v_d2);
    //    GenPID(target_3, q3, s_3, p_3, vd3, Kp, Ki, Kd, v_d3);
    //    u1 = v_d1;
    //    u2 = -0.5f * v_d1 - v_d2 * sin(PI / 3);
    //    u3 = -0.5f * v_d1 + v_d2 * sin(PI / 3);
    u1 = target_1;
    u2 = target_2;
    u3 = target_3;
    //    u1 = u_d1 - d * u_d3 * PI / 18000.0f;
    //    u2 = -0.5f * u_d1 - u_d2 * sin(PI / 3) - d * u_d3 * PI / 18000.0f;
    //    u3 = -0.5f * u_d1 + u_d2 * sin(PI / 3) - d * u_d3 * PI / 18000.0f;
    if (u1 > 0) {
      Omni.wheelBackSetSpeedMMPS((int)u1, DIR_ADVANCE);
    } else if (u1 < 0) {
      Omni.wheelBackSetSpeedMMPS((int)(0 - u1), DIR_BACKOFF);
    }
    if (u2 > 0) {
      Omni.wheelLeftSetSpeedMMPS((int)u2, DIR_ADVANCE);
    } else if (u2 < 0) {
      Omni.wheelLeftSetSpeedMMPS((int)(0 - u2), DIR_BACKOFF);
    }
    if (u3 > 0) {
      Omni.wheelRightSetSpeedMMPS((int)u3, DIR_ADVANCE);
    } else if (u3 < 0) {
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
    //    Serial.print(", u3 = ");
    //    Serial.print(u2, DEC);
    //    Serial.println(u3, DEC);

    //    delay(10);
  }
    Omni.PIDRegulate();
}
// char prevChar = '0';
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inputString.length() == 14 && inChar == '\n') {
      stringComplete = true;
    }
    //    prevChar = inChar;
  }
}
