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
  char data[DEV_ID_DATA_LEN];
  SPI.begin(SPI1_SSL0);
  N25Q256A::read_DEV_ID(data);

  /*
  Serial.print("Chip ID read = ");
  for (int i=0; i<DEV_ID_DATA_LEN; i++) {
      Serial.print(data[i] & 0x00FF,HEX);
      if (i<DEV_ID_DATA_LEN-1)
          Serial.print(",");
      else
          Serial.println(" ");
  }
  */
  return true;
}

boolean N25Q256A::read_DEV_ID(char *data, int len) {
    char cmd[2];
    cmd[0] = READ_DEV_ID_CMD;
    SPI.readwrite_transfer(cmd, data, len);
    N25Q256A::_manufacturer_id_code = data[0];
    N25Q256A::_device_id_mem_type_code = data[1];
    N25Q256A::_device_id_mem_size_code = data[2];
    N25Q256A::process_DEV_ID();
    return true;
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
