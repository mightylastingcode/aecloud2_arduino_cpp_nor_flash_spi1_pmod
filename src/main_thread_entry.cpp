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

void display_device_info (void);
void display_device_id_data(char *data);
void display_status_reg_data(status_reg_t status_reg);
void display_flag_status_reg_data(flagstatus_reg_t flagstatus_reg);
int  page_program_with_data(const char *data, byte sect_add, byte pgm_add, byte byte_add, int len);
void display_array_data(char *data, int len);

void setup() {
    char str[NAME_STR_LEN];   // a string to print an output
    char data[256+10];        // Flash read or write data buffer
                              // add overhead (10) to cover command,address,dummy clocks

    byte sect_add = 0;
    byte pgm_add  = 0;
    byte byte_add = 0;
    status_reg_t        status_reg;
    flagstatus_reg_t    flagstatus_reg;

    // Initialization
    Serial.begin(9600); // Serial Object
    Serial.println("\n\nbegin uart1...\n\n");
    norflash.begin();   // Flash Memory Object


    // Get Device information
    display_device_info();

    // Read device ID by user
    norflash.read_DEV_ID(data);
    display_device_id_data(data);

    // Read status register
    status_reg.status_byte = norflash.read_status_register();
    display_status_reg_data(status_reg);

    // Write Enable
    norflash.write_enable();

    // Read status register
    status_reg.status_byte = norflash.read_status_register();
    display_status_reg_data(status_reg);

    // Write Disable
    norflash.write_disable();

    // Read status register
    status_reg.status_byte = norflash.read_status_register();
    display_status_reg_data(status_reg);

    // Read flag status register
    flagstatus_reg.flagstatus_byte = norflash.read_flag_status_register();
    display_flag_status_reg_data(flagstatus_reg);

    // Program a page
    sect_add = 0x00;   // Set the page address and byte address
    pgm_add =  0x00;
    byte_add = 0x00;

    for (int i=0; i<256; i++)   // Initialize the program data.
        data[i] = i + 0x80;

    norflash.write_enable();
    int ret = page_program_with_data(data,sect_add,pgm_add,byte_add,256);
    if (!ret)
        Serial.println("Program Success!\n");
    else if (ret == -1)
        Serial.println("Program Failure!  Write enable flag is not set.\n");
    else if (ret == -2)
        Serial.println("Program Failure!  Program error flag is asserted.\n");

    //norflash.read_array_data(data, sect_add, pgm_add, byte_add);
    norflash.fastread_array_data(data, sect_add, pgm_add, byte_add);  // read 1 byte
    display_array_data(data, 1);

    //norflash.read_array_data(data, sect_add, pgm_add, byte_add, 256);
    norflash.fastread_array_data(data, sect_add, pgm_add, byte_add,256);  // read 256 bytes
    display_array_data(data, 256);

    }

void loop() {
    delay(1000);                  // waits for a second
}

void display_device_info (void){
    char str[NAME_STR_LEN];

    norflash.get_manufacturer_name(str);
    Serial.print("Vendor:");
    Serial.println(str);
    norflash.get_device_type(str);
    Serial.print("Memory type:");
    Serial.println(str);
    Serial.print("Memory size (Mbits):");
    Serial.println(norflash.get_device_size());

    if (norflash.get_dev_support_flag())
        Serial.println("This memory device is supported.\n");
    else
        Serial.println("This memory device is not supported.\n");
}

void display_device_id_data(char *data) {
    Serial.print("Chip ID read = ");
    for (int i=0; i<DEV_ID_DATA_LEN_MAX; i++) {
        Serial.print(data[i] & 0x00FF,HEX);
        if (i<DEV_ID_DATA_LEN_MAX-1)
            Serial.print(",");
        else
            Serial.println(" \n");
    }
}

void display_status_reg_data(status_reg_t status_reg) {
    Serial.print("Status Register's Value = ");
    Serial.print((char)(status_reg.status_byte) & 0x00FF,HEX);
    Serial.println(" HEX");
    norflash.display_status_register(status_reg);
    Serial.println(" ");
}

void display_flag_status_reg_data(flagstatus_reg_t flagstatus_reg){
    Serial.print("Flag Status Register's Value = ");
    Serial.print((char)(flagstatus_reg.flagstatus_byte) & 0x00FF,HEX);
    Serial.println(" HEX");
    norflash.display_flag_status_register(flagstatus_reg);
    Serial.println(" ");
}


// return status (0 success, -1 fail (write disable), -2 fail (pgm error)
int page_program_with_data(const char *data, byte sect_add, byte pgm_add, byte byte_add, int len) {
    status_reg_t status_reg;
    flagstatus_reg_t flagstatus_reg;

    // Read status register
    status_reg.status_byte = norflash.read_status_register();
    display_status_reg_data(status_reg);

    // Page program
      if (status_reg.status_bitname.write_en) {
          Serial.println("Issue page program command.");
          norflash.page_program(data, sect_add, pgm_add, byte_add, len);
      } else {
          Serial.println("No page program because write enable is off.");
          return -1;
      }

      int  counter = 0;
      do {
           status_reg.status_byte = norflash.read_status_register();
           Serial.println((char)(status_reg.status_byte) & 0x00FF,HEX);
       } while ((status_reg.status_bitname.busy == 1) && (counter++ < 200));  // timeout for 200 check.
       norflash.display_status_register(status_reg);
       display_status_reg_data(status_reg);

       flagstatus_reg.flagstatus_byte = norflash.read_flag_status_register();
       display_flag_status_reg_data(flagstatus_reg);

       if (flagstatus_reg.flagstatus_bitname.pgm_err_status) {
           Serial.println("Fail: Program Error.");
           return -2;
       } else {
           Serial.println("Pass: Program Clear.");
       }

       Serial.println("Clear Flag Status Register");
       norflash.clear_flag_status_register();
       return 0;
}

void display_array_data(char *data, int len) {
    Serial.print("read data = ");
    Serial.println(data[0] & 0x00FF,HEX);

    for (int i=0; i< len; i++) {
           Serial.print(data[i] & 0x00FF,HEX);
           Serial.print(", ");
           if (i%10 == 9) Serial.println(" ");
     }
     Serial.println(" ");
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
