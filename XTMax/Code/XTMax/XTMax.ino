//
//
//  File Name   :  XTMax.ino
//  Used on     : 
//  Author      :  Ted Fried, MicroCore Labs
//  Creation    :  9/7/2024
//
//   Description:
//   ============
//   
//  Multi-function 8-bit ISA card using a Teensy 4.1.
//
//------------------------------------------------------------------------
//
// Modification History:
// =====================
//
// Revision 1 9/7/2024
// Initial revision
//
// Revision 2 10/5/2024
// Added support for SD to Parallel interface
//
// Revision 3 10/11/2024
// Added variable wait states for Expanded RAM
//  - For 4.77 Mhz, can be changed to zero wait states for Write cycles and two for Read cycles
//
// Revision 4 11/11/2024
// - Updated MicroSD support and conventional memory to 640 KB
//
// Revision 5 11/26/2024
// - XTMax automatically configured addition to conventional memory to 640 KB
//
// Revision 6 12/14/2024
// - Made SD LPT base a # define
//
//------------------------------------------------------------------------
//
// Copyright (c) 2024 Ted Fried
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//------------------------------------------------------------------------
                                                 


#include <stdint.h>
#include <stdio.h>


// Teensy 4.1 pin assignments
//
#define PIN_BCLK            34     
#define PIN_BALE            5
#define PIN_AEN             29     
#define PIN_CHRDY_OE_n      28
#define PIN_CHRDY_OUT       6
#define PIN_REFRESH         32     
#define PIN_MEMWR_n         33 
#define PIN_MEMRD_n         4
#define PIN_IOWR_n          3
#define PIN_IORD_n          2
   
#define PIN_MUX_DATA_n      31     
#define PIN_DATA_OE_n       30     
#define PIN_MUX_ADDR_n      9
#define PIN_TRIG_OUT        35
 
#define PIN_ADDR19          27
#define PIN_ADDR18          26
#define PIN_ADDR17          39
#define PIN_ADDR16          38
#define PIN_ADDR15          21
#define PIN_ADDR14          20
#define PIN_ADDR13          23
#define PIN_ADDR12          22
#define PIN_ADDR11          16
#define PIN_ADDR10          17
#define PIN_ADDR9           41
#define PIN_ADDR8           40
#define PIN_AD7             15
#define PIN_AD6             14
#define PIN_AD5             18
#define PIN_AD4             19
#define PIN_AD3             25
#define PIN_AD2             24
#define PIN_AD1             0
#define PIN_AD0             1

#define PIN_DOUT7           37
#define PIN_DOUT6           36
#define PIN_DOUT5           7
#define PIN_DOUT4           8
#define PIN_DOUT3           13
#define PIN_DOUT2           11  // temp spi_mosi
#define PIN_DOUT1           12  // temp spi_cs
#define PIN_DOUT0           10  // temp spi clk


#define ADDRESS_DATA_GPIO6_UNSCRAMBLE   ( ((gpio6_int&0xFFFF0000)>>12) | ((gpio6_int&0x3000)>>10) | ((gpio6_int&0xC)>>2) )

#define GPIO7_DATA_OUT_UNSCRAMBLE        ( ( (isa_data_out&0xF0)<<12) | (isa_data_out&0x0F) )


#define DATA_OE_n_LOW      0x0
#define DATA_OE_n_HIGH     0x00800000
 
#define TRIG_OUT_LOW       0x0
#define TRIG_OUT_HIGH      0x10000000
                           
#define MUX_DATA_n_LOW     0x0
#define MUX_DATA_n_HIGH    0x00400000
                           
#define CHRDY_OUT_LOW      0x0
#define CHRDY_OUT_HIGH     0x00000400
                           
#define CHRDY_OE_n_LOW     0x0
#define CHRDY_OE_n_HIGH    0x00040000
                           
#define MUX_ADDR_n_LOW     0x0
#define MUX_ADDR_n_HIGH    0x00000800
 
   
// --------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------


uint32_t  trigger_out = 0;
uint32_t  gpio6_int = 0;
uint32_t  gpio9_int = 0;
uint32_t  isa_address = 0;
uint32_t  page_base_address = 0;
uint32_t  psram_address = 0;
uint32_t  sd_pin_outputs = 0;
uint32_t  databit_out = 0;

uint8_t   data_in = 0;
uint8_t   isa_data_out = 0;
uint8_t   lpt_data = 0;
uint8_t   lpt_status = 0x6F;
uint8_t   lpt_control = 0xEC;
uint8_t   nibble_in =0;
uint8_t   nibble_out =0;
uint8_t   read_byte =0;
uint8_t   reg_0x260 =0;
uint8_t   reg_0x261 =0;
uint8_t   reg_0x262 =0;
uint8_t   reg_0x263 =0;
uint8_t   spi_shift_out =0;
uint8_t   sd_spi_datain =0;
uint32_t  sd_spi_cs_n = 0x0;
uint32_t  sd_spi_dataout =0;


struct Record {
  uint8_t type;
  uint32_t addr;
};

#define MAX_RECORDS 32768
Record records[MAX_RECORDS];
uint32_t record_index = 0;

// --------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------

// Setup Teensy 4.1 IO's
//
void setup() {
 
  pinMode(PIN_BCLK,           INPUT);   
  pinMode(PIN_BALE,           INPUT);
  pinMode(PIN_AEN,            INPUT);   
  pinMode(PIN_CHRDY_OE_n,     OUTPUT);
  pinMode(PIN_CHRDY_OUT,      OUTPUT);
  pinMode(PIN_REFRESH,        INPUT);   
  pinMode(PIN_MEMWR_n,        INPUT);
  pinMode(PIN_MEMRD_n,        INPUT);
  pinMode(PIN_IOWR_n,         INPUT);
  pinMode(PIN_IORD_n,         INPUT);
 
  pinMode(PIN_MUX_DATA_n,     OUTPUT);   
  pinMode(PIN_DATA_OE_n,      OUTPUT);   
  pinMode(PIN_MUX_ADDR_n,     OUTPUT);
  pinMode(PIN_TRIG_OUT,       OUTPUT);
 
  pinMode(PIN_ADDR19,         INPUT);
  pinMode(PIN_ADDR18,         INPUT);
  pinMode(PIN_ADDR17,         INPUT);
  pinMode(PIN_ADDR16,         INPUT);
  pinMode(PIN_ADDR15,         INPUT);
  pinMode(PIN_ADDR14,         INPUT);
  pinMode(PIN_ADDR13,         INPUT);
  pinMode(PIN_ADDR12,         INPUT);
  pinMode(PIN_ADDR11,         INPUT);
  pinMode(PIN_ADDR10,         INPUT);
  pinMode(PIN_ADDR9,          INPUT);
  pinMode(PIN_ADDR8,          INPUT);
  pinMode(PIN_AD7,            INPUT);
  pinMode(PIN_AD6,            INPUT);
  pinMode(PIN_AD5,            INPUT);
  pinMode(PIN_AD4,            INPUT);
  pinMode(PIN_AD3,            INPUT);
  pinMode(PIN_AD2,            INPUT);
  pinMode(PIN_AD1,            INPUT);
  pinMode(PIN_AD0,            INPUT);
 
  pinMode(PIN_DOUT7,          OUTPUT);
  pinMode(PIN_DOUT6,          OUTPUT);
  pinMode(PIN_DOUT5,          OUTPUT);
  pinMode(PIN_DOUT4,          OUTPUT);
  pinMode(PIN_DOUT3,          OUTPUT);
  pinMode(PIN_DOUT2,          OUTPUT);
  pinMode(PIN_DOUT1,          OUTPUT);
  pinMode(PIN_DOUT0,          OUTPUT);
 
  digitalWriteFast(PIN_CHRDY_OE_n,   0x1);
  digitalWriteFast(PIN_CHRDY_OUT,    0x0);
 
  digitalWriteFast(PIN_DATA_OE_n,    0x1);
  digitalWriteFast(PIN_MUX_ADDR_n,   0x0);
  digitalWriteFast(PIN_MUX_DATA_n,   0x1);

  Serial.begin(9600);
}


// --------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------

inline void Mem_Read_Cycle() {
  isa_address = ADDRESS_DATA_GPIO6_UNSCRAMBLE;

  records[record_index].type = 0;
  records[record_index++].addr = isa_address;
}


// --------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------

inline void Mem_Write_Cycle() {
  isa_address = ADDRESS_DATA_GPIO6_UNSCRAMBLE;

  records[record_index].type = 1;
  records[record_index++].addr = isa_address;
}


// --------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------

inline void IO_Read_Cycle() {
  isa_address = 0xFFFF & ADDRESS_DATA_GPIO6_UNSCRAMBLE;

  records[record_index].type = 2;
  records[record_index++].addr = isa_address;
}


// --------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------

inline void IO_Write_Cycle() {
  isa_address = 0xFFFF & ADDRESS_DATA_GPIO6_UNSCRAMBLE;

  records[record_index].type = 3;
  records[record_index++].addr = isa_address;
}


   
// -------------------------------------------------
//
// Main loop
//
// -------------------------------------------------
void loop() {
  while (1) {
  gpio6_int = GPIO6_DR;
  gpio9_int = GPIO9_DR;

       if ((gpio9_int&0x80000010)==0)  IO_Read_Cycle();  // Isolate and check AEN and IO Rd/Wr
  else if ((gpio9_int&0x80000020)==0)  IO_Write_Cycle();
  else if ((gpio9_int&0x00000040)==0)  Mem_Read_Cycle();
  else if ((gpio9_int&0x00000080)==0)  Mem_Write_Cycle();

  if (record_index >= MAX_RECORDS) {
    record_index = 0;
  }

  if (Serial.available() && Serial.read() == 'r') {
    Serial.println("BEGIN");
    for (uint32_t i = 0; i < record_index; i++) {
      Serial.print(i, DEC);
      Serial.print(": ");
      switch (records[i].type) {
        case 2:
          Serial.print("IOR ");
          break;
        case 3:
          Serial.print("IOW ");
          break;
        case 0:
          Serial.print("MMR ");
          break;
        case 1:
          Serial.print("MMW ");
          break;
      }
      Serial.println(records[i].addr, HEX);
    }
    Serial.println("END");
    Serial.flush();
  }
  }
}
