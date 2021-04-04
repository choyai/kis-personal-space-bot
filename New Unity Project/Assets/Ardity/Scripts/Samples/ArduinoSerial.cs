/**
 * Ardity (Serial Communication for Arduino + Unity)
 * Author: Daniel Wilches <dwilches@gmail.com>
 *
 * This work is released under the Creative Commons Attributions license.
 * https://creativecommons.org/licenses/by/2.0/
 */

using UnityEngine;
using System.Collections;
using System.Text;

/**
 * Sample for reading using polling by yourself, and writing too.
 */
public class ArduinoSerial : MonoBehaviour
{
public SerialControllerCustomDelimiter serialController;

// Initialization
void Start()
{
        serialController = GameObject.Find("SerialController").GetComponent<SerialControllerCustomDelimiter>();

        Debug.Log("Press the SPACEBAR to execute some action");
}

// Executed each frame
void Update()
{
        //---------------------------------------------------------------------
        // Send data
        //---------------------------------------------------------------------

        // If you press one of these keys send it to the serial device. A
        // sample serial device that accepts this input is given in the README.
        // if (Input.GetKeyDown(KeyCode.Space))
        // {
        //         Debug.Log("Sending some action");
        //         // Sends a 65 (ascii for 'A') followed by an space (ascii 32, which
        //         // is configured in the controller of our scene as the separator).
        //         byte[] BytesToSend = SendSerialCommand(1, 0, 100);
        //         byte[] actualSent = new byte[11];
        //         for(int i = 0; i<11; i++) {
        //                 actualSent[i] = BytesToSend[i];
        //         }
        //         serialController.SendSerialMessage(actualSent);
        // }
        if(Input.GetKeyDown(KeyCode.Space)) {
                Debug.Log("pressed space");
                Send(1, 0, 100, 30);
        }
        //---------------------------------------------------------------------
        // Receive data
        //---------------------------------------------------------------------

        byte[] message = serialController.ReadSerialMessage();

        if (message == null)
                return;

        StringBuilder sb = new StringBuilder();
        foreach (byte b in message)
                sb.AppendFormat("(#{0}={1})    ", b, (char)b);
        Debug.Log("Received some bytes, printing their ascii codes: " + sb);
}

public void Send(byte id, int x, int y, int theta){
        byte[] BytesToSend = SendSerialCommand(id, x, y, theta);
        byte[] actualSent = new byte[14];
        for(int i = 0; i<14; i++) {
                actualSent[i] = BytesToSend[i];
        }
        serialController.SendSerialMessage(actualSent);
}

byte[] SendSerialCommand(byte id, int x, int y, int theta){

        byte[] BufferArr = new byte[14];
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
        if(theta < 0) {
                BufferArr[9] = 1;
                theta = 0-theta;
        }
        else{
                BufferArr[9]= 0;
        }
        int[] splitx = SplitLargeInt(x);
        int[] splity = SplitLargeInt(y);
        int[] splitw = SplitLargeInt(theta);

        BufferArr[4] = (byte)splitx[0];
        BufferArr[5] = (byte)splitx[1];
        BufferArr[7] = (byte)splity[0];
        BufferArr[8] = (byte)splity[1];
        BufferArr[10] = (byte)splitw[0];
        BufferArr[11] = (byte)splitw[1];

        int sum = 0;
        for(int i = 0; i<12; i++) {
                sum = sum + BufferArr[i];
        }
        BufferArr[12] = (byte)sum;
        // Debug.Log("writing:");
        // foreach(byte b in BufferArr) {
        //         Debug.Log(b);
        // }
        BufferArr[13] = 10;
        // BufferArr[11] = '\n';
        // Debug.Log();
        return BufferArr;
}

int[] SplitLargeInt(int Value){
        int msB = (Value/256) % 256;
        int lsB = Value % 256;
        int[] arr = {msB, lsB};
        return arr;
}
}
