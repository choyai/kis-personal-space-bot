
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;


public class CtrlRobotCorutine : MonoBehaviour
{
GameObject[] sbj = new GameObject[4];       //three robots(participants) and camera，0:R, 1:G, 2:B, 3:Vive controler
GameObject centre;        //the centre of participants
GameObject IdealPos;    //robot's target position
GameObject IdealPosMiddle;      //robot's the middle of targtet position
GameObject TestRobot;

//parameters of navigation model
private double[] V_function = new double[DefineCoroutine.MAXDISTANCE];                                                              //V関数．
private float[] env_x_range = new float[DefineCoroutine.NEIGHBORHOOD_RANGE];                                     //各配列に移動する範囲を記述
private float[] env_y_range = new float[DefineCoroutine.NEIGHBORHOOD_RANGE];
private float[] RelativeLocation = new float[2];
private double[] PREsumedRF = new double[DefineCoroutine.MAXDISTANCE];            //報酬関数の合計，差分をとるための. for renewing V function
//for PID?
private bool sentPos = false;

public ArduinoSerial serialController;
public OmniWheelController pid;

// Start is called before the first frame update
void Start()
{
        //initializaton of navigation model
        InitializeNeighborhood();

        sbj[0] = GameObject.Find("robot_red");
        sbj[1] = GameObject.Find("robot_green");
        sbj[2] = GameObject.Find("robot_blue");
        //sbj[3] = GameObject.Find("Controller (right)");		//location feedback from vive controler!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        sbj[3] = GameObject.Find("Tracker1"); //location feedback from vive controler!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        IdealPos = GameObject.Find("IdealPosition"); //IdealPos.transform.position.x, z, and rotation.y are robot's target!!!!!!
        IdealPosMiddle=GameObject.Find("IdealPositionMiddle");

        //TestRobot=GameObject.Find("VirtualRobot");

        //for test in the static environment (Particioants do not move in this experiment, so that the centre also does not move)
        centre = GameObject.Find("centre");
        centre.transform.position = new Vector3
                                            ((sbj[0].transform.position.x + sbj[1].transform.position.x + sbj[2].transform.position.x) / 3,
                                            (sbj[0].transform.position.y + sbj[1].transform.position.y + sbj[2].transform.position.y) / 3,
                                            (sbj[0].transform.position.z + sbj[1].transform.position.z + sbj[2].transform.position.z) / 3);

        serialController = GameObject.Find("ArduinoSerial").GetComponent<ArduinoSerial>();
        pid = GameObject.Find("RobotCntroler").GetComponent<PIDTest>();
        Debug.Log("start!");
        //StartCoroutine(PID());			//for PID?
        StartCoroutine(Renew());    //renew V
        StartCoroutine(Move());     //generate Q and send command
}

private IEnumerator PID()
{
        while (1==1)
        {
                //send current location
                //serialController.Send(1, (int)Math.Floor(sbj[3].transform.position.x*1000), (int)Math.Floor(sbj[3].transform.position.z*1000), (int) Math.Floor(sbj[3].transform.rotation.y*18000));
                //Debug.Log("x_c="+sbj[3].transform.position.x+", y_c="+sbj[3].transform.position.z+", theta_c="+sbj[3].transform.rotation.y*18000);
                if(!sentPos)
                {
                        //serialController.Send(2, (int)Math.Floor(sbj[3].transform.position.x*1000), (int)Math.Floor(sbj[3].transform.position.z*1000), (int) Math.Floor(sbj[3].transform.rotation.y*18000));
                        //sentPos = true;
                }
                yield return new WaitForSeconds(0.01f);
        }
}

//renew V function
private IEnumerator Renew()
{
        while (1 == 1)
        {
                double[] rf = new double[DefineCoroutine.MAXDISTANCE];
                double[] sumedRF = new double[DefineCoroutine.MAXDISTANCE];      //報酬関数の合計
                float distance = 0;
                sumedRF = PREsumedRF;
                for(int i=0; i<DefineCoroutine.MAXDISTANCE; i++)
                {
                        PREsumedRF[i] = 0;
                }
                //全てのHA間の距離を求めて報酬とする
                for (int i = 0; i < DefineCoroutine.A_NUM - 1; i++)
                {
                        for (int j = i + 1; j < DefineCoroutine.A_NUM - 1; j++)
                        {
                                //i番目とj番目の各HAの距離の導出
                                distance = 100 * CalDistance(sbj[i].transform.position.x, sbj[j].transform.position.x, sbj[i].transform.position.z, sbj[j].transform.position.z);
                                //i番目とj番目の各HAの距離から報酬を算出．
                                rf = CalReward((int)distance);
                                //報酬関数の足し合わせ
                                for (int k = 0; k < DefineCoroutine.MAXDISTANCE; k++)
                                {
                                        sumedRF[k] += rf[k];
                                        PREsumedRF[k] -= 0.9*rf[k];
                                }
                        }
                }
                yield return new WaitForSeconds(0.1f);
                //V関数の最大値を検索．
                double MaxValue = V_function.Max(); //価値関数の最大値
                //足し合わされた報酬関数を用いてV関数の更新
                for (int k = 0; k < DefineCoroutine.MAXDISTANCE; k++)
                {
                        V_function[k] = (1 - DefineCoroutine.ALPHA) * V_function[k] + DefineCoroutine.ALPHA * (sumedRF[k] + DefineCoroutine.GAMMA * MaxValue);
                }
                yield return new WaitForSeconds(0.1f);
        }
}

//renew Q and send command
private IEnumerator Move()
{
        int count==0;

        while (1 == 1)
        {
                //initialization of the target location
                //IdealPos.transform.position = new Vector3
                //  (sbj[3].transform.position.x,
                //   sbj[3].transform.position.y,                           //Ignore Ideal.transform.position.y
                //  sbj[3].transform.position.z);

                /*
                   IdealPos.transform.position = new Vector3
                       (TestRobot.transform.position.x,
                        TestRobot.transform.position.y,                           //Ignore Ideal.transform.position.y
                        TestRobot.transform.position.z);
                 */
                //making a decision
                float maxnum = 0f;
                double MaxValue = 0;
                //V関数の最大値を検索．
                for (int a = 0; a < DefineCoroutine.MAXDISTANCE; a++)
                {
                        if (V_function[a] > MaxValue)
                        {
                                maxnum = (float)a / 100;
                                MaxValue = V_function[a];
                        }
                }//Debug.Log("maxnum="+maxnum);
                yield return new WaitForSeconds(0.25f);

                double[,] q = new double[DefineCoroutine.NEIGHBORHOOD_RANGE, DefineCoroutine.NEIGHBORHOOD_RANGE];    //Q値,a_nmならばx方向にn，y方向にm移動
                float delta_dis = 0;                                                                //各HAとの実際の距離を格納．
                //近傍に移動できる範囲の値を代入する．
                for (int a = 0; a < DefineCoroutine.NEIGHBORHOOD_RANGE; a++)
                {
                        for (int b = 0; b < DefineCoroutine.NEIGHBORHOOD_RANGE; b++)
                        {
                                //q値の初期化
                                q[a, b] = 1;
                                //RAのぶんを除いた他のAgentとの距離を用いて導出
                                for (int k = 0; k < DefineCoroutine.A_NUM - 1; k++)
                                {
                                        //DefineCoroutine.A_NUM-1番目はRAを示す．RAとi番目のHAとの距離の導出及びスケーリング，反転を行って足し合わせている．
                                        //例えば，LT[DefineCoroutine.A_NUM-1, 0] + nはRAのx座標からnだけ移動した位置を示している．
                                        //Q値をガウス関数を用いて表現
                                        //delta_dis = CalDistance(sbj[k].transform.position.x, sbj[DefineCoroutine.A_NUM - 1].transform.position.x + env_x_range[a], sbj[k].transform.position.z, sbj[DefineCoroutine.A_NUM - 1].transform.position.z + env_y_range[b]) - maxnum;
                                        delta_dis = CalDistance(sbj[k].transform.position.x, sbj[3].transform.position.x + env_x_range[a], sbj[k].transform.position.z, sbj[3].transform.position.z + env_y_range[b]) - maxnum;
                                        if (delta_dis < -1000)
                                        {
                                                q[a, b] = 0;
                                        }
                                        q[a, b] *= Math.Exp(-1 * Math.Pow(delta_dis, 2) / DefineCoroutine.SIGMA_Q);
                                }
                        }
                }
                yield return new WaitForSeconds(0.25f);

                //Q値の最大値から移動すべき距離を導出
                double maxQ = -1 * DefineCoroutine.MAXDISTANCE;                //最大値探索のための変数．初期値としてマイナスの値を使用．
                for (int i2 = 0; i2 < DefineCoroutine.NEIGHBORHOOD_RANGE; i2++)
                {
                        for (int j2 = 0; j2 < DefineCoroutine.NEIGHBORHOOD_RANGE; j2++)
                        {
                                if (maxQ < q[i2, j2])
                                {
                                        RelativeLocation[0] = env_x_range[i2];
                                        RelativeLocation[1] = env_y_range[j2];
                                        maxQ = q[i2, j2];
                                }
                        }
                }

                if(RelativeLocation[0]<0.0001 && RelativeLocation[1] < 0.0001)
                {
                        InitializeV();
                }
                //command to the robot, Substitute the absolute position of robot's target location for IdelPos. (x,y)
                IdealPos.transform.position = new Vector3
                                                      (sbj[3].transform.position.x + RelativeLocation[0],
                                                      sbj[3].transform.position.y, //Ignore Ideal.transform.position.y
                                                      sbj[3].transform.position.z + RelativeLocation[1]);

                if (count==0)
                {
                        IdealPosMiddle.transform.position = new Vector3
                                                                    (sbj[3].transform.position.x + RelativeLocation[0]/2,
                                                                    sbj[3].transform.position.y, //Ignore Ideal.transform.position.y
                                                                    sbj[3].transform.position.z + RelativeLocation[1]/2);
                        count++;
                }

                //IdealPos.transform.position.x =IdealPos.transform.position.x + RelativeLocation[0];
                //IdealPos.transform.position.z =IdealPos.transform.position.z + RelativeLocation[1];

                //IdealPos.transform.position = new Vector3
                //    (IdealPos.transform.position.x + RelativeLocation[0],
                //     IdealPos.transform.position.y,							//Ignore Ideal.transform.position.y
                //     IdealPos.transform.position.z + RelativeLocation[1]);
                /*
                   TestRobot.transform.position = new Vector3
                       (IdealPos.transform.position.x + RelativeLocation[0],
                        IdealPos.transform.position.y,							//Ignore Ideal.transform.position.y
                        IdealPos.transform.position.z + RelativeLocation[1]);
                 */

                Debug.Log("RelativeLocationX=" + RelativeLocation[0] + ", RelativeLocationY=" + RelativeLocation[1]);
                //Debug.Log("rotation.y=" + IdealPos.transform.rotation.y);
                //Ideal position object looks at centre of group. THETA is driven here!!!!!
                IdealPos.transform.LookAt(centre.transform);
                //TestRobot.transform.LookAt(centre.transform);
                pid.MoveTo(IdealPos.transform.position.x, IdealPos.transform.position.z, IdealPos.transform.rotation.eulerAngles.y);
                //Debug.Log("move(x_t="+ sbj[3].transform.position.x+RelativeLocation[0]+ ", y_t=" + sbj[3].transform.position.z+RelativeLocation[1]+") and theta_t="+IdealPos.transform.rotation.y);
                Debug.Log("idealTARGET: x_t=" + IdealPos.transform.position.x+ ", y_t=" + IdealPos.transform.position.z+ ", theta_t=" + IdealPos.transform.rotation.y);
                Debug.Log("CurrentPosition("+sbj[3].transform.position.x+", "+sbj[3].transform.position.z+")");
                //sentPos = false;
                yield return new WaitForSeconds(0.25f);
        }
}

//報酬の計算
//exp{-(a-a_n)^2/2}の出力,R(s;s_k,SIGMA)においてs_kは他HAの位置情報
private double[] CalReward(int x_n)
{
        //Debug.Log(x_n);
        double[] rf = new double[DefineCoroutine.MAXDISTANCE];

        for (int i = 0; i < DefineCoroutine.MAXDISTANCE; i++)
        {
                rf[i] = Math.Exp(-Math.Pow((i - x_n), 2) / (2 * DefineCoroutine.SIGMA_V));
        }

        return rf;
}

//距離の導出
private float CalDistance(float x1, float x2, float y1, float y2)
{
        return (float)Math.Pow(Math.Pow(x1 - x2, 2) + Math.Pow(y1 - y2, 2), 0.5);
}

//V関数の初期化
public void InitializeV()
{
        for (int i = 0; i < V_function.Length; i++)
        {
                V_function[i] = 0;
        }
}

//近傍の初期化
public void InitializeNeighborhood()
{
        //近傍に移動できる範囲の値を代入する．
        for (int a = 0; a < DefineCoroutine.NEIGHBORHOOD_RANGE; a++)
        {
                for (int b = 0; b < DefineCoroutine.NEIGHBORHOOD_RANGE; b++)
                {
                        env_x_range[a] = -1 * (int)Math.Floor((double)DefineCoroutine.NEIGHBORHOOD_RANGE / 2) - 1 + (a + 1);
                        env_y_range[b] = -1 * (int)Math.Floor((double)DefineCoroutine.NEIGHBORHOOD_RANGE / 2) - 1 + (b + 1);

                        env_x_range[a] =  env_x_range[a] / (DefineCoroutine.NEIGHBORHOOD_RANGE - 1);
                        env_y_range[b] =  env_y_range[b] / (DefineCoroutine.NEIGHBORHOOD_RANGE - 1);
                }
        }
}

}
