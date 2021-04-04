using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;
using System;
using System.Text;

public class Arduino : MonoBehaviour
{
public static Encoding ascii = Encoding.ASCII;
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
void SendSerialCommand(Byte id, int x, int y){

        Byte[] BufferArr = new Byte[11];
        BufferArr[0] = 255;
        BufferArr[1] = 255;
        BufferArr[2] = id;
        if(x < 0) {
                BufferArr[3] = 1;
                x = 0-x;
        }
        else{
                BufferArr[3] = 0;
        }
        if(y < 0) {
                BufferArr[6] = 1;
                y = 0-y;
        }
        else{
                BufferArr[6] = 0;
        }
        int[] splitx = SplitLargeInt(x);
        int[] splity = SplitLargeInt(y);
        BufferArr[4] = (Byte)splitx[0];
        BufferArr[5] = (Byte)splitx[1];
        BufferArr[7] = (Byte)splity[0];
        BufferArr[8] = (Byte)splity[1];
        int sum = 0;
        for(int i = 0; i<9; i++) {
                sum = sum + BufferArr[i];
        }
        BufferArr[9] = (Byte)sum;
        Debug.Log("writing:");
        foreach(Byte b in BufferArr) {
                Debug.Log(b);
        }
        BufferArr[10] = 10;
        // BufferArr[11] = '\n';
        string sendString = ascii.GetString(BufferArr);
        // Debug.Log();
}
int[] SplitLargeInt(int Value){
        int msB = (Value/256) % 256;
        int lsB = Value % 256;
        int[] arr = {msB, lsB};
        return arr;
}
}
