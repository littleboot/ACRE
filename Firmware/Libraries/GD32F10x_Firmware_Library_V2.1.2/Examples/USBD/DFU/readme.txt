/*!
    \file  readme.txt
    \brief description of the USB DFU(device firmware upgrade) demo.

    \version 2014-12-26, V1.0.0, firmware for GD32F10x
    \version 2017-06-20, V2.0.0, firmware for GD32F10x
    \version 2018-07-31, V2.1.0, firmware for GD32F10x
*/

/*
    Copyright (c) 2018, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

  This demo is based on the GD32F10x-EVAL board,it presents the implementation 
of a device firmware upgrade (DFU) capability in the GD32 USB device.

  It follows the DFU class specification defined by the USB Implementers Forum for 
reprogramming an application through USB-FS-Device.

  The DFU principle is particularly well suited to USB-FS-Device applications that 
need to be reprogrammed in the field.

  To test the demo, you need a configuration hex image or bin image . The hex image 
and the bin image should set application address at 0x8004000. You can refer to 
"../Utilities/Binary/DFU_Images" folder and use the hex images and bin images in it.

  You need install the corresponding GD DFU Driver with your PC operation system. 

  Once the configuration *.hex image is generated, it can be downloaded into 
internal flash memory using the GD tool "GD MCU Dfu Tool" available for download 
from www.gd32mcu.21ic.com.

  The supported memory for this example is the internal flash memory, you can also
add a new memory interface if you have extral memory.

After each device reset, hold down the TAMPER key on the GD32F10x-EVAL board.
