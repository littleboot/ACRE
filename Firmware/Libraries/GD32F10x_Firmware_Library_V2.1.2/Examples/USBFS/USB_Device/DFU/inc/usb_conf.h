/*!
    \file  usb_conf.h
    \brief USBFS driver basic configuration

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

#ifndef USB_CONF_H
#define USB_CONF_H

#include "gd32f10x.h"
#include "gd32f10x_eval.h" 

#ifdef USE_USBFS
    #define USBFS_CORE
#endif /* USE_USBFS */

#ifdef USE_USBHS
    #define USBHS_CORE
#endif /* USE_USBHS */

#ifdef USBFS_CORE
    #define RX_FIFO_FS_SIZE                         128
    #define TX0_FIFO_FS_SIZE                        64
    #define TX1_FIFO_FS_SIZE                        128
    #define TX2_FIFO_FS_SIZE                        0
    #define TX3_FIFO_FS_SIZE                        0

// #define USBFS_LOW_PWR_MGMT_SUPPORT
// #define USBFS_SOF_OUTPUT_ENABLED
#endif /* USBFS_CORE */

#ifdef USBHS_CORE
    #define RX_FIFO_HS_SIZE                          512
    #define TX0_FIFO_HS_SIZE                         128
    #define TX1_FIFO_HS_SIZE                         372
    #define TX2_FIFO_HS_SIZE                         0
    #define TX3_FIFO_HS_SIZE                         0
    #define TX4_FIFO_HS_SIZE                         0
    #define TX5_FIFO_HS_SIZE                         0

    #ifdef USE_ULPI_PHY
        #define USB_ULPI_PHY_ENABLED
    #endif

    #ifdef USE_EMBEDDED_PHY
        #define USB_EMBEDDED_PHY_ENABLED
    #endif

//    #define USBHS_INTERNAL_DMA_ENABLED
    #define USBHS_DEDICATED_EP1_ENABLED
    #define USBHS_LOW_PWR_MGMT_SUPPORT
    #define USBHS_SOF_OUTPUT_ENABLED
#endif /* USBHS_CORE */

//#define VBUS_SENSING_ENABLED

//#define USE_HOST_MODE
#define USE_DEVICE_MODE
//#define USE_OTG_MODE

/* In HS mode and when the DMA is used, all variables and data structures dealing
   with the DMA during the transaction process should be 4-bytes aligned */
#ifdef USBHS_INTERNAL_DMA_ENABLED
    #if defined   (__GNUC__)            /* GNU Compiler */
        #define __ALIGN_END             __attribute__ ((aligned (4)))
        #define __ALIGN_BEGIN
    #else
        #define __ALIGN_END

        #if defined   (__CC_ARM)        /* ARM Compiler */
            #define __ALIGN_BEGIN       __align(4)
        #elif defined (__ICCARM__)      /* IAR Compiler */
            #define __ALIGN_BEGIN 
        #elif defined  (__TASKING__)    /* TASKING Compiler */
            #define __ALIGN_BEGIN       __align(4) 
        #endif /* __CC_ARM */  
    #endif /* __GNUC__ */ 
#else
    #define __ALIGN_BEGIN
    #define __ALIGN_END   
#endif /* USBHS_INTERNAL_DMA_ENABLED */

#endif /* USB_CONF_H */

