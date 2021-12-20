/*!
    \file  usbd_conf.h
    \brief usb device driver basic configuration

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

#ifndef USBD_CONF_H
#define USBD_CONF_H

#include "gd32f10x.h"
#include "gd32f10x_eval.h"

#define USBD_CFG_MAX_NUM                  1
#define USBD_ITF_MAX_NUM                  1

/* USB feature -- Self Powered */
/* #define USBD_SELF_POWERED */

/* USB user string supported */
#define USB_SUPPORT_USER_STRING_DESC

/* DFU maximum data packet size */
#define TRANSFER_SIZE                     2048

/* memory address from where user application will be loaded, which represents 
   the dfu code protected against write and erase operations.*/
#define APP_LOADED_ADDR                   0x08008000

/* Make sure the corresponding memory where the DFU code should not be loaded
   cannot be erased or overwritten by DFU application. */
#define IS_PROTECTED_AREA(addr)  (uint8_t)(((addr >= 0x08000000) && (addr < (APP_LOADED_ADDR)))? 1 : 0)

/* DFU endpoint define */
#define DFU_IN_EP                         EP0_IN
#define DFU_OUT_EP                        EP0_OUT

/* Endpoint count used by the DFU device */
#define EP_COUNT                          (1)

/* Base address of the allocation buffer, used for buffer descriptor table and packet memory */
#define BUFFER_ADDRESS                    (0x0000)

#define USB_STRING_COUNT                  6

#define USB_PULLUP                        GPIOD
#define USB_PULLUP_PIN                    GPIO_PIN_13
#define RCC_AHBPeriph_GPIO_PULLUP         RCU_GPIOD

#endif /* USBD_CONF_H */
