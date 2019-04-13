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

#include "N25Q256A.h"

//====================== Your Arduino Example Sketch Begin ===========//
SERIAL1 Serial  = SERIAL1();   //UART 1
SPI1    SPI = SPI1();   // SPI 1 port  needed by N25Q256A.  Use spi pmod port.
N25Q256A  norflash = N25Q256A();


void setup() {
    char str[NAME_STR_LEN];
    char data[256+10];  // add overhead (10) to cover command,address,dummy clocks

    byte sect_add = 0;
    byte pgm_add  = 0;
    byte byte_add = 0;
    status_reg_t        status_reg;
    flagstatus_reg_t    flagstatus_reg;

    Serial.begin(9600);
    //while(!Serial);
    Serial.println("\n\nbegin uart1...\n\n");

    norflash.begin();

    norflash.get_manufacturer_name(str);
    Serial.print("Vendor:");
    Serial.println(str);
    norflash.get_device_type(str);
    Serial.print("Memory type:");
    Serial.println(str);
    Serial.print("Memory size (Mbits):");
    Serial.println(norflash.get_device_size());

    if (norflash.get_dev_support_flag())
        Serial.println("This memory device is supported.");
    else
        Serial.println("This memory device is not supported.");
    /////////////////////////////////////////////////////////////
    // Read device ID by user
    norflash.read_DEV_ID(data);
    Serial.print("Chip ID read = ");
    for (int i=0; i<DEV_ID_DATA_LEN_MAX; i++) {
        Serial.print(data[i] & 0x00FF,HEX);
        if (i<DEV_ID_DATA_LEN_MAX-1)
            Serial.print(",");
        else
            Serial.println(" ");
    }
    /////////////////////////////////////////////////////////////
    status_reg.status_byte = norflash.read_status_register();
    Serial.print("Status Register's Value = ");
    Serial.print((char)(status_reg.status_byte) & 0x00FF,HEX);
    Serial.println(" HEX");
    norflash.display_status_register(status_reg);
    /////////////////////////////////////////////////////////////
    norflash.write_enable();
    /////////////////////////////////////////////////////////////
    status_reg.status_byte = norflash.read_status_register();
    Serial.print("Status Register's Value = ");
    Serial.print((char)(status_reg.status_byte) & 0x00FF,HEX);
    Serial.println(" HEX");
    norflash.display_status_register(status_reg);
    /////////////////////////////////////////////////////////////
    /*
    norflash.write_disable();
    /////////////////////////////////////////////////////////////
    status_reg.status_byte = norflash.read_status_register();
    Serial.print("Status Register's Value = ");
    Serial.print((char)(status_reg.status_byte) & 0x00FF,HEX);
    Serial.println(" HEX");
    norflash.display_status_register(status_reg);
    /////////////////////////////////////////////////////////////
    flagstatus_reg.flagstatus_byte = norflash.read_flag_status_register();
    Serial.print("Flag Status Register's Value = ");
    Serial.print((char)(flagstatus_reg.flagstatus_byte) & 0x00FF,HEX);
    Serial.println(" HEX");
    norflash.display_flag_status_register(flagstatus_reg);
    /////////////////////////////////////////////////////////////
     *
     */
    if (status_reg.status_bitname.write_en) {
        Serial.println("Issue page program command.");
        sect_add = 0x00;
        pgm_add =  0x00;
        byte_add = 0x00;

        for (int i=0; i<256; i++)
            data[i] = i + 0x80;
        norflash.page_program(data, sect_add, pgm_add, byte_add, 256);
    } else {
        Serial.println("No page program because write enable is off.");
    }

    int  counter = 0;
    do {
         status_reg.status_byte = norflash.read_status_register();
         Serial.println((char)(status_reg.status_byte) & 0x00FF,HEX);
     } while ((status_reg.status_bitname.busy == 1) && (counter++ < 200));  // timeout for 200 check.
     norflash.display_status_register(status_reg);
     /////////////////////////////////////////////////////////////
     flagstatus_reg.flagstatus_byte = norflash.read_flag_status_register();
     Serial.print("Flag Status Register's Value = ");
     Serial.print((char)(flagstatus_reg.flagstatus_byte) & 0x00FF,HEX);
     Serial.println(" HEX");
     norflash.display_flag_status_register(flagstatus_reg);
     /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
    sect_add = 0;
    pgm_add = 0x00;
    byte_add = 0;
    //norflash.read_array_data(data, sect_add, pgm_add, byte_add);
    norflash.fastread_array_data(data, sect_add, pgm_add, byte_add);

    Serial.print("read data = ");
    Serial.println(data[0] & 0x00FF,HEX);

    for (int i=0; i< 256; i++) {
           Serial.print(data[i] & 0x00FF,HEX);
           Serial.print(", ");
           if (i%10 == 9) Serial.println(" ");
     }
     Serial.println(" ");

     //norflash.read_array_data(data, sect_add, pgm_add, byte_add, 256);
     norflash.fastread_array_data(data, sect_add, pgm_add, byte_add,256);

     Serial.print("read data = ");
     Serial.println(data[0] & 0x00FF,HEX);

     for (int i=0; i< 256; i++) {
            Serial.print(data[i] & 0x00FF,HEX);
            Serial.print(", ");
            if (i%10 == 9) Serial.println(" ");
      }
      Serial.println(" ");

      /////////////////////////////////////////////////////////////
      sect_add = 0;
      pgm_add = 0x0E;
      byte_add = 0;
      //norflash.read_array_data(data, sect_add, pgm_add, byte_add);
      norflash.fastread_array_data(data, sect_add, pgm_add, byte_add);

      Serial.print("read data = ");
      Serial.println(data[0] & 0x00FF,HEX);

      for (int i=0; i< 256; i++) {
             Serial.print(data[i] & 0x00FF,HEX);
             Serial.print(", ");
             if (i%10 == 9) Serial.println(" ");
       }
       Serial.println(" ");

       //norflash.read_array_data(data, sect_add, pgm_add, byte_add, 256);
       norflash.fastread_array_data(data, sect_add, pgm_add, byte_add,256);

       Serial.print("read data = ");
       Serial.println(data[0] & 0x00FF,HEX);

       for (int i=0; i< 256; i++) {
              Serial.print(data[i] & 0x00FF,HEX);
              Serial.print(", ");
              if (i%10 == 9) Serial.println(" ");
        }
        Serial.println(" ");
      /////////////////////////////////////////////////////////////
    }

void loop() {
    delay(1000);                  // waits for a second
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

