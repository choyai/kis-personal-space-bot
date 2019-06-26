using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;

public class SendSerialScript : MonoBehaviour
{
// Start is called before the first frame update
void Start()
{
        SendSerialCommand(1, 0, 100);
}

// Update is called once per frame
void Update()
{

}
void OnGUI(){
        if(Event.current.Equals(Event.KeyboardEvent("g")))
        {
                Debug.Log("it wasn't me, said the GUI");
                SendSerialCommand(1, 0, 100);
        }
}
void SendSerialCommand(int id, int x, int y){
        char[] BufferArr = new char[10];
        BufferArr[0] = (char)255;
        BufferArr[1] = (char)255;
        BufferArr[2] = (char)id;
        if(x < 0) {
                BufferArr[3] = (char)1;
                x = 0-x;
        }
        else{
                BufferArr[3] = (char)0;
        }
        if(y < 0) {
                BufferArr[6] = (char)1;
                y = 0-y;
        }
        else{
                BufferArr[6] = (char)0;
        }
        int[] splitx = SplitLargeInt(x);
        int[] splity = SplitLargeInt(y);
        BufferArr[4] = (char)splitx[0];
        BufferArr[5] = (char)splitx[1];
        BufferArr[7] = (char)splity[0];
        BufferArr[8] = (char)splity[1];
        int sum = 0;
        for(int i = 0; i<9; i++) {
                sum = sum + BufferArr[i];
        }
        BufferArr[9] = (char)sum;
        // BufferArr[10] = '\r';
        // BufferArr[11] = '\n';
        string sendString = new string(BufferArr);
        Serial.WriteLn(sendString);
}
int[] SplitLargeInt(int Value){
        int msB = (Value/256) % 256;
        int lsB = Value % 256;
        int[] arr = {msB, lsB};
        return arr;
}
}
