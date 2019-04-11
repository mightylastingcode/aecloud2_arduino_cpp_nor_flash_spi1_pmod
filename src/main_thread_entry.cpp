/*********************************************************************
This is the main thread for placing Arduino Sketch code to run on Renesas AE Cloud2 S5D9 board

Created on: September 17, 2018
First Released on: March 19, 2019
Author: Michael Li (michael.li@miketechuniverse.com)

An Arduino sketch is placed inside the section that is defined by two comment lines.  The section has the
example of setup() and loop() functions.   You can replace them with your example code.  It is very simple.
Most common functions like Serial and Wire functions are supported except of the SPI functions which need to
be modified to run.


The MIT License (MIT)


Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.


*********************************************************************/



#include "main_thread.h"
#include <Arduino.h>
#include <Wire.h>
#include <Wire1.h>
#include "SERIAL.h"
#include "SERIAL1.h"
#include "SPI.h"
#include "SPI1.h"
#include <stdio.h>


//====================== Your Arduino Example Sketch Begin ===========//
SERIAL1 Serial  = SERIAL1();   //UART 1
SPI1    SPI = SPI1();   // SPI 1 port


#define  READ_DEV_ID_CMD                0x9F  // 2 to 20 bytes
#define  READ_STATUS_REG_CMD            0x05  // 2 bytes
#define  READ_FLAG_STATUS_REGISTER_CMD      0x70  // 1 byte
#define  CLEAR_FLAG_STATUS_REGISTER_CMD     0x50  // 1 byte
#define  WRITE_ENABLE_CMD               0x06  // 1 byte

void display_status_register(char statusbyte);
void display_flag_status_register(char statusbyte);
void display_ID_data(char *data);

void setup() {
    char data[40], buf[40];
    char cmd[2];   // only the first byte is used.

    Serial.begin(9600);
    //while(!Serial);
    Serial.println("\n\nbegin uart1...\n\n");
    SPI.begin(SPI1_SSL0);


    // read chip ID : Expect 0x20, 0xBA, 0x19
    Serial.println("==============================");
    cmd[0] = READ_DEV_ID_CMD;
    SPI.readwrite_transfer(cmd, data, 3);
    Serial.print("Chip ID read = ");
    for (int i=0; i<3; i++) {
        Serial.print(data[i] & 0x00FF,HEX);
        if (i<3)
            Serial.print(",");
        else
            Serial.println(" ");
    }
    display_ID_data(data);

    Serial.println("==============================");
    cmd[0] = READ_STATUS_REG_CMD;
    SPI.readwrite_transfer(cmd, data, 1);
    Serial.print("Status Register's Value = ");
    Serial.print(data[0] & 0x00FF,HEX);
    Serial.println(" HEX");
    display_status_register(data[0]);

    Serial.println("==============================");
    cmd[0] = READ_FLAG_STATUS_REGISTER_CMD;
    SPI.readwrite_transfer(cmd, data, 1);
    Serial.print("Flag Status Register's Value = ");
    Serial.print(data[0] & 0x00FF,HEX);
    Serial.println(" HEX");
    display_flag_status_register(data[0]);

    Serial.println("==============================");
    Serial.println("Send Write enable command.");
    cmd[0] = WRITE_ENABLE_CMD;
    SPI.write_transfer(cmd,1);

    Serial.println("==============================");
    cmd[0] = READ_STATUS_REG_CMD;
    SPI.readwrite_transfer(cmd, data, 1);
    Serial.print("Status Register's Value = ");
    Serial.print(data[0] & 0x00FF,HEX);
    Serial.println(" HEX");
    display_status_register(data[0]);

    Serial.println("==============================");
    cmd[0] = READ_FLAG_STATUS_REGISTER_CMD;
    SPI.readwrite_transfer(cmd, data, 1);
    Serial.print("Flag Status Register's Value = ");
    Serial.print(data[0] & 0x00FF,HEX);
    Serial.println(" HEX");
    display_flag_status_register(data[0]);

    Serial.println("==============================");
    Serial.println("Send CLEAR_FLAG_STATUS command.");
    cmd[0] = CLEAR_FLAG_STATUS_REGISTER_CMD;
    SPI.write_transfer(cmd,1);

    Serial.println("==============================");
    cmd[0] = READ_STATUS_REG_CMD;
    SPI.readwrite_transfer(cmd, data, 1);
    Serial.print("Status Register's Value = ");
    Serial.print(data[0] & 0x00FF,HEX);
    Serial.println(" HEX");
    display_status_register(data[0]);

    Serial.println("==============================");
    cmd[0] = READ_FLAG_STATUS_REGISTER_CMD;
    SPI.readwrite_transfer(cmd, data, 1);
    Serial.print("Flag Status Register's Value = ");
    Serial.print(data[0] & 0x00FF,HEX);
    Serial.println(" HEX");
    display_flag_status_register(data[0]);

}

void loop() {
    char data[40], buf[40];
    char cmd[2];   // only the first byte is used.


    delay(1000);                  // waits for a second

}

void display_status_register(char statusbyte) {
    if (statusbyte & 0x80)
        Serial.println("Bit 7 Status Register write allowed = No");
    else
        Serial.println("Bit 7 Status Register write allowed = Yes");
    if (statusbyte & 0x20)
        Serial.println("Bit 5 T/B = Bottom");
    else
        Serial.println("Bit 5 T/B = Top");
    Serial.print("Bit 6,4,3,2 Block Protection = ");
    Serial.print(statusbyte & 0x40,DEC);
    Serial.print(statusbyte & 0x10,DEC);
    Serial.print(statusbyte & 0x08,DEC);
    Serial.print(statusbyte & 0x04,DEC);
    Serial.println(" ");
    if (statusbyte & 0x02)
        Serial.println("Bit 1 Write enable latch = ON");
    else
        Serial.println("Bit 1 Write enable latch = OFF");
    if (statusbyte & 0x01)
        Serial.println("Bit 0 Write in progress = True (Busy)");
    else
        Serial.println("Bit 0 Write in progress = False (Ready)");

}


void display_flag_status_register(char statusbyte) {
    if (statusbyte & 0x80)
        Serial.println("Bit 7 Program/Erase op = Ready");
    else
        Serial.println("Bit 7 Program/Erase op = Busy");
    if (statusbyte & 0x40)
        Serial.println("Bit 6 Erase Suspend = ON");
    else
        Serial.println("Bit 6 Erase Suspend = OFF");
    if (statusbyte & 0x20)
        Serial.println("Bit 5 Erase Status = Fail or Protection Error");
    else
        Serial.println("Bit 5 Erase Status = Clear");
    if (statusbyte & 0x10)
        Serial.println("Bit 4 Program Status = Fail or Protection Error");
    else
        Serial.println("Bit 4 Program Status = Clear");
    if (statusbyte & 0x08)
        Serial.println("Bit 3 VPP = Disable");
    else
        Serial.println("Bit 3 VPP = Enable");
    if (statusbyte & 0x04)
        Serial.println("Bit 2 Program Suspend = ON");
    else
        Serial.println("Bit 2 Program Suspend = OFF");
    if (statusbyte & 0x02)
        Serial.println("Bit 1 Protection = Fail or Error");
    else
        Serial.println("Bit 1 Protection = Clear");
    if (statusbyte & 0x01)
        Serial.println("Bit 0 Addr = 4 bytes (256Mb)");
    else
        Serial.println("Bit 0 Addr = 3 bytes (128Mb)");

}


void display_ID_data(char *data) {

     Serial.print("Manufacture ID = ");
     Serial.print(data[0] & 0x00FF,HEX);
     Serial.println(" HEX");

     Serial.print("Device ID Memory Type = ");
     Serial.print(data[1] & 0x00FF,HEX);
     Serial.println(" HEX");

     Serial.print("Device ID Memory Capacity = ");
     Serial.print(data[2] & 0x00FF,HEX);
     Serial.println(" HEX");

}
//====================== Your Arduino Example Sketch End ===========//



//============================= Renesas Main Thread entry function ====================================//
void main_thread_entry(void)
{
   system_setup();
   setup();

    while (1) {
        loop();
        delay(1);  // required for the thread if delay()does not exist in the loop() function.
    }
}

