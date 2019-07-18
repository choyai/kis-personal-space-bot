using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


//Barebones PID test code.

public class PIDTest : MonoBehaviour
{
    //RobotAgent r = new RobotAgent();

    GameObject[] sbj = new GameObject[4];   //three robots and camera，0:R, 1:G, 2:B, 3:Camera
    GameObject centre;   //the centre of group
	GameObject NextRobotLoc;

	//for test 
	GameObject ViveCtrler;

	public ArduinoSerial serialController;
    // Start is called before the first frame update
    void Start()
    {
        //initializaton
		ViveCtrler = GameObject.Find("Controller (right)");		//location feedback from vive controler!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



        Debug.Log("start!");

		serialController = GameObject.Find("ArduinoSerial").GetComponent<ArduinoSerial>();
		serialController.Send(2,0,0,0);

    }

    float[] location = new float[3];    //(x,y,theta)

	private float timeElapsed;
	private float timeElapsed_potision;
    private float timeOut_position = 0.05f;			//control TIME\
    private float timeOut = 2f;						//control TIME
	private bool sentPos = false;
	private float current_angle=0f;
	private float tar_x, tar_y, tar_theta;
	private bool moving  = false;
	private float t_k;



private float c0_1 = 0.0f, c0_2 = 0.0f, c0_3 = 0.0f, c1_1 = 0.0f, c1_2 = 0.0f, c1_3 = 0.0f, c2_1 = 0.0f, c2_2 = 0.0f, c2_3 = 0.0f, c3_1 = 0.0f, c3_2 = 0.0f, c3_3 = 0.0f;

	private float x, y, theta, u_x, u_y, u_theta, s_x, s_y, s_theta, p_x, p_y, p_theta;
	private float Kp = 0.06f, Ki = 0.0f, Kd = 0.0f;
	private float[] u = new float[3];
	private float d = 25.0f + 150.0f * (float)Math.Cos(((float)Math.PI) / 6);
	private float r_1 =0.0f, r_2 = 0.0f , r_3 =0.0f;
	private float v_d1 = 0.0f, v_d2= 0.0f, v_d3=0.0f;
	private float t = 0.0f;
    // Update is called once per frame
    void Update()
    {
		timeElapsed += Time.deltaTime;
		timeElapsed_potision +=Time.deltaTime;
		x = ViveCtrler.transform.position.x;
		y = ViveCtrler.transform.position.z;
		theta = ViveCtrler.transform.rotation.y;


		        if(Input.GetKeyDown(KeyCode.Space)) {

                Debug.Log("pressed space");

               MoveTo(0.0f, 0.0f, 0.0f);

        }

		if(moving){
		t = timeElapsed / t_k;
		r_1 = c0_1 + c1_1 * t + c2_1 * t * t + c3_1 * t *( t * t);
		 r_2 = c0_2 + c1_2 * t + c2_2 * t * t + c3_2 * t *( t * t);
		 r_3 = c0_3 + c1_3 * t + c2_3 * t * t + c3_3 * t * (t * t);
		 //Debug.Log(r_1.ToString());
    v_d1 = c1_1 + 2.0f * c2_1 * t + 3.0f * c3_1 * (t * t);

    v_d2 = c1_2 + 2.0f * c2_2 * t + 3.0f * c3_2 * (t * t);

    v_d3 = c1_3 + 2.0f * c2_3 * t + 3.0f * c3_3 * (t * t);

		GenPID(tar_x, x, ref s_x, ref p_x, ref u_x, Kp, Ki, Kd, v_d1);
		GenPID(tar_y, y, ref s_y, ref p_y, ref u_y, Kp, Ki, Kd, v_d2);
		GenPID(tar_theta, theta, ref s_theta, ref p_theta, ref u_theta, Kp, Ki, Kd, v_d3);
		
		

		if(Math.Abs(tar_x - x)< 0.05&&Math.Abs(tar_y - y)< 0.05&&Math.Abs(tar_theta - theta)< 0.3){
			moving = false;
			t = 0;
			serialController.Send(0,0,0,0);
		}

		if(timeElapsed_potision>=timeOut_position){
			u[0] = u_x - d * u_theta * ((float)Math.PI) / 18000.0f;
			Debug.Log(u_x.ToString());
			u[1] = -0.5f * u_x - u_y * (float)Math.Sin(((float)Math.PI)/3.0f) - d * u_theta * ((float)Math.PI) / 18000.0f;
			u[2] = -0.5f * u_x + u_y * (float)Math.Sin(((float)Math.PI)/3.0f) - d * u_theta * ((float)Math.PI) / 18000.0f;
			serialController.Send(1, (int)Math.Floor(u[0] * 1000.0f),(int)Math.Floor(u[1] * 1000.0f),(int)Math.Floor(u[2] * 18000.0f));
			

			timeElapsed_potision=0;
	}
	}
	}
	/*******************************************/

// cubic trajectory generation



// general cubic trajectory generation

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
	private void GenPID(float pos, float q, ref float s, ref float p, ref float u, float Kp,

            float Ki, float Kd, float v_d)

{

  float e = pos - q;

  s = s + e;

    u = (Kp * e) + (Ki * s) + (Kd * (v_d - (e - p)));

  //u = (Kp * e) + (Ki * s) - (Kd * (e - p));

  p = e;

}

	public void MoveTo(float x, float y, float theta){
		cubic1(x, 0.0f, 0.0f, 2.5f);
		cubic2(y, 0.0f, 0.0f, 2.5f);
		cubic3(theta, 0.0f, 0.0f, 2.5f);
		moving = true;
		timeElapsed = 0f;
	}
}
