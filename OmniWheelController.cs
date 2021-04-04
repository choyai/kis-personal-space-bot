
using System.Collections;

using System.Collections.Generic;

using UnityEngine;

using System;





//Robot control through serialController, calculates output of each motor based on position x y and theta

public class OmniWheelController : MonoBehaviour

{

GameObject[] sbj = new GameObject[4];       //three robots and camera，0:R, 1:G, 2:B, 3:Camera

GameObject centre;       //the centre of group

GameObject NextRobotLoc;

float[] location = new float[3];        //(x,y,theta)

private float timeElapsed;

//time from last sent serial command.
private float timeElapsed_position;

//set frequency of serial command
private float timeOut_position = 0.1f;         //control TIME\

private float timeOut = 2.5f;               //control TIME

private bool sentPos = false;

private float current_angle=0f;         //not used anymore

//Target position variables
private float tar_x, tar_y, tar_theta;

private bool moving  = false;

// Period
private float t_k;

//Cubic spline coefficients
private float c0_1 = 0.0f, c0_2 = 0.0f, c0_3 = 0.0f, c1_1 = 0.0f, c1_2 = 0.0f, c1_3 = 0.0f, c2_1 = 0.0f, c2_2 = 0.0f, c2_3 = 0.0f, c3_1 = 0.0f, c3_2 = 0.0f, c3_3 = 0.0f;

//x,y, theta, etc... u_x, u_y, u_theta are output velocities for PID
private float x, y, theta, u_x = 0.0f, u_y = 0.0f, u_theta = 0.0f, s_x, s_y, s_theta, p_x, p_y, p_theta;

/*********************************************/
//Tune PID Gains here
private float Kp = 2.4f, Ki = 0.000015f, Kd = 0.001f;

/*********************************************/

//output for each motor
private float[] u = new float[3];

// d = distance from center of robot to wheel.
private float d = (25.0f + 150.0f * (float)Math.Cos(((float)Math.PI) / 6))/1000f;

// r = target position at time tau
private float r_1 =0.0f, r_2 = 0.0f, r_3 =0.0f;

//desired velocities
private float v_d1 = 0.0f, v_d2= 0.0f, v_d3=0.0f;

//tau
private float t = 0.0f;

// u of robot from the robot's frame of reference
private float u_x_b, u_y_b;


GameObject ViveCtrler;

public ArduinoSerial serialController;

//for test





// Start

void Start()

{

        //initializaton

        ViveCtrler = GameObject.Find("Controller (right)"); //location feedback from vive controler!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        Debug.Log("start!");

        // Find serialController
        serialController = GameObject.Find("ArduinoSerial").GetComponent<ArduinoSerial>();



}





// Update is called once per frame

void Update()

{

        timeElapsed += Time.deltaTime;

        timeElapsed_position +=Time.deltaTime;

        x = ViveCtrler.transform.position.x;

        y = ViveCtrler.transform.position.z;

        theta = ViveCtrler.transform.rotation.eulerAngles.y;
        if(theta >=180f) {
                theta = theta - 360f;
        }
        //Debug.Log("y = " + y.ToString());
        //Debug.Log("x = " + x.ToString());
        //Debug.Log("theta = " + theta.ToString());


        if(Input.GetKeyDown(KeyCode.Space)) {

                Debug.Log("sending commands");

                MoveTo(0.0f, 0.0f, 0.0f);

        }


        //
        if(moving) {
                //calculate the tau value --> how far in the curve you are
                t = timeElapsed / t_k;
                //t should not be more than 1
                if(timeElapsed >= t_k) {
                        t = 1.0f;
                }

                //use the cubic coefficients to calculate position at time tau

                r_1 = c0_1 + c1_1 * t + c2_1 * t * t + c3_1 * t *( t * t);

                r_2 = c0_2 + c1_2 * t + c2_2 * t * t + c3_2 * t *( t * t);

                r_3 = c0_3 + c1_3 * t + c2_3 * t * t + c3_3 * t * (t * t);

                //Debug.Log("v1 = " + v_d1.ToString());

                // calculate velocities
                v_d1 = c1_1 + 2.0f * c2_1 * t + 3.0f * c3_1 * (t * t);

                v_d2 = c1_2 + 2.0f * c2_2 * t + 3.0f * c3_2 * (t * t);

                v_d3 = c1_3 + 2.0f * c2_3 * t + 3.0f * c3_3 * (t * t);


                //generate PID for each dimension(x, y, theta)
                GenPID(r_1, x, ref s_x, ref p_x, ref u_x, Kp, Ki, Kd, v_d1);

                GenPID(r_2, y, ref s_y, ref p_y, ref u_y, Kp, Ki, Kd, v_d2);

                GenPID(r_3, theta, ref s_theta, ref p_theta, ref u_theta, 0.01f, 0.0f, 0.0001f, v_d3);



                u_x_b = u_x * ((float)Math.Cos(theta*((float)Math.PI)/180.0f)) - u_y * ((float)Math.Sin(theta*((float)Math.PI)/180.0f));
                u_y_b = u_y * ((float)Math.Cos(theta*((float)Math.PI)/180.0f)) + u_x * ((float)Math.Sin(theta*((float)Math.PI)/180.0f));



                if(Math.Abs(tar_x - x)< 0.05f&&Math.Abs(tar_y - y)< 0.05f &&  Math.Abs(tar_theta - theta)< 10.0f ) {
                        Debug.Log("Stopping");

                        moving = false;

                        t = 0;

                        serialController.Send(0,0,0,0);

                }



                if(timeElapsed_position>=timeOut_position) {
                        //calculate the output velocities of each motor
                        u[0] = u_x_b - d * u_theta * ((float)Math.PI) / 180.0f;

                        //Debug.Log("ux = " + u_x.ToString());
                        //Debug.Log("uy = " + u_y.ToString());
                        //Debug.Log("omega = " + u_theta.ToString());

                        u[1] = -0.5f * u_x_b - u_y_b * (float)Math.Sin(((float)Math.PI)/3.0f) - d * u_theta * ((float)Math.PI) / 180.0f;

                        u[2] = -0.5f * u_x_b + u_y_b * (float)Math.Sin(((float)Math.PI)/3.0f) - d * u_theta * ((float)Math.PI) / 180.0f;

                        //Send the motor outputs
                        serialController.Send(1, (int)Math.Floor(u[0] * 1000.0f),(int)Math.Floor(u[1] * 1000.0f),(int)Math.Floor(u[2] * 1000.0f));

                        timeElapsed_position=0;

                }

        }

}

/*******************************************/

// cubic trajectory generation

// general cubic trajectory generation(prototype function)

void cubic(float q_i, float q_f, float v_i, float v_f, float tkk,ref float t_kk,

           ref float c_0, ref float c_1, ref float c_2, ref float c_3) {

        c_0 = q_i;

        c_1 = v_i;

        c_2 = 3.0f * q_f - 3.0f * q_i - v_f - 2.0f * v_i;

        c_3 = 2.0f * q_i - 2.0f * q_f + v_f + v_i;

        t_kk = tkk;

}

// x cubic trajectory
void cubic1(float tar_q, float in_v, float tar_v, float tkkk) {

        cubic(x, tar_q, in_v, tar_v, tkkk, ref t_k, ref c0_1, ref c1_1, ref c2_1, ref c3_1);

        tar_x = tar_q;

}

// y cubic trajectory
void cubic2(float tar_q, float in_v, float tar_v, float tkkk) {

        cubic(y, tar_q, in_v, tar_v, tkkk, ref t_k, ref c0_2, ref c1_2, ref c2_2, ref c3_2);

        tar_y = tar_q;

}

// theta cubic trajectory
void cubic3(float tar_q, float in_v, float tar_v, float tkkk) {

        cubic(theta, tar_q, in_v, tar_v, tkkk, ref t_k, ref c0_3, ref c1_3, ref c2_3, ref c3_3);

        tar_theta = tar_q;

}

//Generic PID function
private void GenPID(float pos, float q, ref float s, ref float p, ref float u, float Kp,

                    float Ki, float Kd, float v_d)

{

        float e = pos - q;

        s = s + e;
        //if not using cubic trajectory, then comment the next line and uncomment the next line
        u = (Kp * e) + (Ki * s) + (Kd * (v_d - (e - p)));
        //u = (Kp * e) + (Ki * s) - (Kd * (e - p));

        p = e;

}


//Move to x, y theta using cubic trajectory
public void MoveTo(float x, float y, float theta){

        cubic1(x, v_d1, 0.0f, 1.0f);

        cubic2(y, v_d2, 0.0f, 1.0f);

        cubic3(theta, v_d3, 0.0f, 1.0f);

        moving = true;

        timeElapsed = 0f;

}

}
