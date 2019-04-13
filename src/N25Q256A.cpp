/*
 *


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
*/


#include "N25Q256A.h"
#include "SPI1.h"
#include "SPI.h"
#include "Arduino.h"

#include "SERIAL1.h"
extern SERIAL1 Serial;   //UART 1 Serial communication


// use this declaration for this spi device that owns the spi port exclusively
//SPI1 SPI = SPI1(); // SPI pmod por
//SPI0 SPI = SPI0();

// use this declaration for two spi devices that share the same spi port.
extern  SPI1 SPI;  // SPI pmod port
//extern SPI0 SPI;  // Arduino header port


N25Q256A::N25Q256A() {
}

boolean N25Q256A::begin(void) {
  char data[DEV_ID_DATA_LEN_MAX];
  SPI.begin(SPI1_SSL0);
  N25Q256A::read_DEV_ID(data);
  return true;
}

void N25Q256A::read_DEV_ID(char *data, int len) {
    char cmd[2];
    cmd[0] = READ_DEV_ID_CMD;
    SPI.readwrite_transfer(cmd, data, len);
    N25Q256A::_manufacturer_id_code = data[0];
    N25Q256A::_device_id_mem_type_code = data[1];
    N25Q256A::_device_id_mem_size_code = data[2];
    N25Q256A::process_DEV_ID();
}

// Read array bytes for general cases.
// input:  erase sector address (8 bits), program page address (8 bits), byte address (8 bits), len (255 max)
// output: data 1. array size required = len + READHEADER + 1
//
void N25Q256A::read_array_data(char *data, byte sect_add, byte pgm_add, byte byte_add, int len) {
    char cmd[10];

    cmd[0] = ARRAY_READ_CMD;
    cmd[1] = sect_add;   //A<24:16> - Erase Sectors
                         //A<15:12> - Erase Subsectors
    cmd[2] = pgm_add;    //A<11:8> -  Program Pages
    cmd[3] = byte_add;   //A<7:0> bytes in Page address
    cmd[4] = 0x85; //First DATA IN

    N25Q256A::print_cmd("ARRAY_READ", cmd);

    if (len <= 256) {
        SPI.readwrite_transfer(cmd, data, READHEADER + len);
        for (int i=0; i< len; i++) {
            data[i] = data[READHEADER+i];
        }
    }
}

// Fast Read array bytes for general cases.
// input:  erase sector address (8 bits), program page address (8 bits), byte address (8 bits), len (255 max)
// output: data 1. array size required = len + READHEADER + 1
//
// Assume: Default setting for number of dummy clocks = 8 clocks
void N25Q256A::fastread_array_data(char *data, byte sect_add, byte pgm_add, byte byte_add, int len) {
    char cmd[10];

    cmd[0] = ARRAY_FASTREAD_CMD;
    cmd[1] = sect_add;   //A<24:16> - Erase Sectors
                         //A<15:12> - Erase Subsectors
    cmd[2] = pgm_add;    //A<11:8> -  Program Pages
    cmd[3] = byte_add;   //A<7:0> bytes in Page address
    cmd[4] = 0x85; //First DATA IN

    N25Q256A::print_cmd("ARRAY_FASTREAD",cmd);

    if (len <= 256) {
        SPI.readwrite_transfer(cmd, data, FASTREADHEADER + len);
        for (int i=0; i< len; i++) {
            data[i] = data[FASTREADHEADER+i];
        }
    }
}

byte N25Q256A::read_status_register(void){
    char cmd[5];
    char data[5];

    cmd[0] = READ_STATUS_REG_CMD;
    cmd[1] = 0;
    cmd[2] = 0;
    cmd[3] = 0;
    N25Q256A::print_cmd("READ_STATUS_REG",cmd);
    SPI.readwrite_transfer(cmd, data, 1);
    return data[0];
}

byte N25Q256A::read_flag_status_register(void) {
    char cmd[5];
    char data[5];

    cmd[0] = READ_FLAG_STATUS_REGISTER_CMD;
    cmd[1] = 0;
    cmd[2] = 0;
    cmd[3] = 0;
    N25Q256A::print_cmd("READ_FLAG_STATUS_REGISTER",cmd);
    SPI.readwrite_transfer(cmd, data, 1);
    return data[0];
}

void N25Q256A::clear_flag_status_register(void) {
    char cmd[5];
    char data[5];

    cmd[0] = CLEAR_FLAG_STATUS_REGISTER_CMD;
    cmd[1] = 0;
    cmd[2] = 0;
    cmd[3] = 0;
    N25Q256A::print_cmd("CLEAR_FLAG_STATUS_REGISTER",cmd);
    SPI.write_transfer(cmd,1);
}

void N25Q256A::write_enable(void) {
    char cmd[5];
    char data[5];

    cmd[0] = WRITE_ENABLE_CMD;
    cmd[1] = 0;
    cmd[2] = 0;
    cmd[3] = 0;
    N25Q256A::print_cmd("WRITE_ENABLE",cmd);
    SPI.write_transfer(cmd,1);

}

void N25Q256A::write_disable(void) {
    char cmd[5];
    char data[5];

    cmd[0] = WRITE_DISABLE_CMD;
    cmd[1] = 0;
    cmd[2] = 0;
    cmd[3] = 0;
    N25Q256A::print_cmd("WRITE_DISABLE",cmd);
    SPI.write_transfer(cmd,1);

}

void N25Q256A::page_program(const char *data, byte sect_add, byte pgm_add, byte byte_add, int len){
     char cmd[PGM_DATA_LEN_MAX+PGM_DATA_LEN_MAX];
     cmd[0] = PAGE_PROGRAM_CMD;
     cmd[1] = sect_add;   //A<24:16> - Erase Sectors
                          //A<15:12> - Erase Subsectors
     cmd[2] = pgm_add;    //A<11:8> -  Program Pages
     cmd[3] = byte_add;   //A<7:0> bytes in Page address

     if (len > 1){
         for (int i=0; i<len; i++)
             cmd[i+4] = data[i];
         Serial.println("Page Program");
         if (len <= 256)
             SPI.write_transfer(cmd, PGMHEADER+len);
     }
}


void subsector_erase(byte sect_add, byte subsect_add) {
    char cmd[5];

    cmd[0] = SUBSECTOR_ERASE_CMD;
    cmd[1] = sect_add;                    //A<24:16> - Erase Sectors
    cmd[2] = (subsect_add << 4) & 0x00F0;  //A<15:12> - Erase Subsectors
                                          //A<11:8> = Page program address = 00
    cmd[3] = 0x00;  //A<7:0> bytes in Page address = 00
    SPI.write_transfer(cmd, SUBSECTERASEHEADER);
}

void N25Q256A::display_status_register(status_reg_t reg){
    if (reg.status_bitname.statusreg_wr_enb)
        Serial.println("Bit 7 Status Register write allowed = No");
    else
        Serial.println("Bit 7 Status Register write allowed = Yes");
    if (reg.status_bitname.topbot)
        Serial.println("Bit 5 T/B = Bottom");
    else
        Serial.println("Bit 5 T/B = Top");
    Serial.print("Bit 6,4,3,2 Block Protection = ");
    Serial.print((char)reg.status_bits.b6);
    Serial.print((char)reg.status_bits.b4);
    Serial.print((char)reg.status_bits.b3);
    Serial.print((char)reg.status_bits.b2);
    Serial.println(" ");
    if (reg.status_bitname.write_en)
        Serial.println("Bit 1 Write enable latch = ON");
    else
        Serial.println("Bit 1 Write enable latch = OFF");
    if (reg.status_bitname.busy)
        Serial.println("Bit 0 Write in progress = True (Busy)");
    else
        Serial.println("Bit 0 Write in progress = False (Ready)");

}

void N25Q256A::display_flag_status_register(flagstatus_reg_t reg) {
    if (reg.flagstatus_bitname.pgmersready)
        Serial.println("Bit 7 Program/Erase op = Ready");
    else
        Serial.println("Bit 7 Program/Erase op = Busy");
    if (reg.flagstatus_bitname.ers_suspend)
        Serial.println("Bit 6 Erase Suspend = ON");
    else
        Serial.println("Bit 6 Erase Suspend = OFF");
    if (reg.flagstatus_bitname.ers_err_status)
        Serial.println("Bit 5 Erase Status = Fail or Protection Error");
    else
        Serial.println("Bit 5 Erase Status = Clear");
    if (reg.flagstatus_bitname.pgm_err_status)
        Serial.println("Bit 4 Program Status = Fail or Protection Error");
    else
        Serial.println("Bit 4 Program Status = Clear");
    if (reg.flagstatus_bitname.vpp_disable)
        Serial.println("Bit 3 VPP = Disable");
    else
        Serial.println("Bit 3 VPP = Enable");
    if (reg.flagstatus_bitname.pgm_suspend)
        Serial.println("Bit 2 Program Suspend = ON");
    else
        Serial.println("Bit 2 Program Suspend = OFF");
    if (reg.flagstatus_bitname.protect_err)
        Serial.println("Bit 1 Protection = Fail or Error");
    else
        Serial.println("Bit 1 Protection = Clear");
    if (reg.flagstatus_bitname.extend_addr)
        Serial.println("Bit 0 Addr = 4 bytes (256Mb)");
    else
        Serial.println("Bit 0 Addr = 3 bytes (128Mb)");
}

void N25Q256A::print_cmd (char* name, char *cmd) {
    Serial.print(name);
    Serial.print(": ");
    Serial.print(cmd[0] & 0x00FF,HEX);
    Serial.print(", ");
    Serial.print(cmd[1] & 0x00FF,HEX);
    Serial.print(", ");
    Serial.print(cmd[2] & 0x00FF,HEX);
    Serial.print(", ");
    Serial.print(cmd[3] & 0x00FF,HEX);
    Serial.println(" ");
}
void N25Q256A::process_DEV_ID(void) {
    N25Q256A::valid_dev_support = true;

    if (N25Q256A::_manufacturer_id_code == MICRON) {
        strcpy(N25Q256A::_manufacturer_name,"Micron");
    } else {
        strcpy(N25Q256A::_manufacturer_name,"Unknown");
    }
    if (N25Q256A::_device_id_mem_type_code == SPINOR) {
        strcpy(N25Q256A::_device_id_mem_type ,"SPI NOR Flash");
    } else {
        strcpy(N25Q256A::_device_id_mem_type,"Unknown");
    }
    if (N25Q256A::_device_id_mem_size_code == MEM256M) {
        N25Q256A::_device_id_mem_size = 256;
    } else {
        N25Q256A::_device_id_mem_size = 0;
    }

    if (N25Q256A::_manufacturer_id_code == MICRON && N25Q256A::_device_id_mem_type_code == SPINOR && N25Q256A::_device_id_mem_size_code == MEM256M) {
        N25Q256A::valid_dev_support = true;
    } else {
        N25Q256A::valid_dev_support = false;
    }
}


void N25Q256A::get_manufacturer_name(char *name, int len){
    strcpy(name, N25Q256A::_manufacturer_name);
}

void N25Q256A::get_device_type(char *name, int len) {
    strcpy(name, N25Q256A::_device_id_mem_type);
}

int  N25Q256A::get_device_size(void){
    return N25Q256A::_device_id_mem_size;
}
boolean N25Q256A::get_dev_support_flag(void) {
    return N25Q256A::valid_dev_support;
}
