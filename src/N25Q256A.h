/*

 Created on: April 4, 2019
 First Released on: April 4, 2019
 Author: Michael Li (michael.li@miketechuniverse.com)


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



#ifndef _N25Q256A_H_
#define _N25Q256A_H_

#include "hal_data.h"  // type
#include "Arduino.h"  // byte type

typedef bool boolean;

//---------------------------------------------------------------
#define  READ_DEV_ID_CMD                0x9F  // 1 to many bytes
#define  ARRAY_READ_CMD                 0x03  // 1+ bytes
#define  ARRAY_FASTREAD_CMD             0x0B  // 1+ bytes
#define  READ_STATUS_REG_CMD            0x05  // 1 data byte
#define  WRITE_ENABLE_CMD               0x06  // 1 byte

//---------------------------------------------------------------
// Support device below.
#define  MICRON     0x20
#define  SPINOR     0xBA
#define  MEM256M    0x19
//---------------------------------------------------------------
// limit device Id data read length.
#define  DEV_ID_DATA_LEN_MAX   21   // 1 command byte + 20 data byte

#define  READHEADER            3   // 3 address bytes
#define  FASTREADHEADER        4   // 3 address bytes + 1 dummy bytes
#define  READ_DATA_LEN_MIN     1  // 3 address bytes + 1 command byte + 1 data byte
//---------------------------------------------------------------
// used by methods
#define  NAME_STR_LEN         20
//---------------------------------------------------------------

typedef struct {
     unsigned busy: 1,
     write_en: 1,
     blockprotect_bit210: 3,
     topbot: 1,
     blockprotect_bit3: 1,
     statusreg_wr_enb: 1;
 } statusbyte_bitname_t;

 typedef struct {
     unsigned b0: 1,
     b1: 1,
     b2: 1,
     b3: 1,
     b4: 1,
     b5: 1,
     b6: 1,
     b7: 1;
 } byte_bits_t;

 typedef union {
     unsigned char status_byte;
     byte_bits_t   status_bits;
     statusbyte_bitname_t status_bitname;
 } status_reg_t;
 //---------------------------------------------------------------


class N25Q256A {
  public:
    N25Q256A();
    boolean begin(void);
    void read_DEV_ID(char *data, int len=DEV_ID_DATA_LEN_MAX);
    void read_array_data(char *data, byte sect_add, byte pgm_add, byte byte_add, int len=READ_DATA_LEN_MIN);
    void fastread_array_data(char *data, byte sect_add, byte pgm_add, byte byte_add, int len=READ_DATA_LEN_MIN);
    byte read_status_register(void);
    void display_status_register(status_reg_t reg);
    void write_enable(void);


    void get_manufacturer_name(char *name, int len=NAME_STR_LEN);
    void get_device_type(char *name, int len=NAME_STR_LEN);
    int  get_device_size(void);
    boolean get_dev_support_flag(void);
  private:
    void process_DEV_ID(void);
    void print_cmd (char* name, char *cmd);

    // Protected data to be read through valid class function only
    // These data should be modified by users because other functions
    // may depend on them to work properly.

    uint8_t _manufacturer_id_code    = 0;
    uint8_t _device_id_mem_type_code = 0;
    uint8_t _device_id_mem_size_code = 0;

    char _device_id_mem_type[NAME_STR_LEN];  // SPI Serial Flash (for valid support check)
    uint16_t _device_id_mem_size           = 0;   // MB unit  (for size check)
    char _manufacturer_name[NAME_STR_LEN];  // Vendor Name

    boolean valid_dev_support;       // based on device ID and manufacture ID

};


#endif
