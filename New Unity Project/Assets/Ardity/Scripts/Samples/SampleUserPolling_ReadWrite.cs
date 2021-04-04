/**
 * Ardity (Serial Communication for Arduino + Unity)
 * Author: Daniel Wilches <dwilches@gmail.com>
 *
 * This work is released under the Creative Commons Attributions license.
 * https://creativecommons.org/licenses/by/2.0/
 */

using UnityEngine;
using System.Collections;

/**
 * Sample for reading using polling by yourself, and writing too.
 */
public class SampleUserPolling_ReadWrite : MonoBehaviour
{
public SerialController serialController;

// Initialization
void Start()
{
        serialController = GameObject.Find("SerialController").GetComponent<SerialController>();

        Debug.Log("Press A or Z to execute some actions");
}

// Executed each frame
void Update()
{
        //---------------------------------------------------------------------
        // Send data
        //---------------------------------------------------------------------

        // If you press one of these keys send it to the serial device. A
        // sample serial device that accepts this input is given in the README.
        if (Input.GetKeyDown(KeyCode.A))
        {
                Debug.Log("Sending A");
                byte[] BytesToSend = SendSerialCommand(1, 0, 100);
                // serialController.SendSerialMessage((string)BytesToSend);
        }

        if (Input.GetKeyDown(KeyCode.Z))
        {
                Debug.Log("Sending Z");
                serialController.SendSerialMessage("Z");
        }


        //---------------------------------------------------------------------
        // Receive data
        //---------------------------------------------------------------------

        string message = serialController.ReadSerialMessage();

        if (message == null)
                return;

        // Check if the message is plain data or a connect/disconnect event.
        if (ReferenceEquals(message, SerialController.SERIAL_DEVICE_CONNECTED))
                Debug.Log("Connection established");
        else if (ReferenceEquals(message, SerialController.SERIAL_DEVICE_DISCONNECTED))
                Debug.Log("Connection attempt failed or disconnection detected");
        else
                Debug.Log("Message arrived: " + message);
}
byte[] SendSerialCommand(byte id, int x, int y){

        byte[] BufferArr = new byte[11];
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
        BufferArr[4] = (byte)splitx[0];
        BufferArr[5] = (byte)splitx[1];
        BufferArr[7] = (byte)splity[0];
        BufferArr[8] = (byte)splity[1];
        int sum = 0;
        for(int i = 0; i<9; i++) {
                sum = sum + BufferArr[i];
        }
        BufferArr[9] = (byte)sum;
        // Debug.Log("writing:");
        // foreach(byte b in BufferArr) {
        //         Debug.Log(b);
        // }
        BufferArr[10] = 10;
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
