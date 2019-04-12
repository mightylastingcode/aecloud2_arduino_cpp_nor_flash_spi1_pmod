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

typedef bool boolean;

//---------------------------------------------------------------
#define  READ_DEV_ID_CMD                0x9F  // 1 to many bytes
//---------------------------------------------------------------
// Support device below.
#define  MICRON     0x20
#define  SPINOR     0xBA
#define  MEM256M    0x19
//---------------------------------------------------------------
// limit device Id data read length.
#define  DEV_ID_DATA_LEN      21   // 1 command byte + 20 data byte
//---------------------------------------------------------------
// used by methods
#define  NAME_STR_LEN         20
//---------------------------------------------------------------

class N25Q256A {
  public:
    N25Q256A();
    boolean begin(void);
    boolean read_DEV_ID(char *data, int len=DEV_ID_DATA_LEN);


    void get_manufacturer_name(char *name, int len=NAME_STR_LEN);
    void get_device_type(char *name, int len=NAME_STR_LEN);
    int  get_device_size(void);
    boolean get_dev_support_flag(void);
  private:
    void process_DEV_ID(void);

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
