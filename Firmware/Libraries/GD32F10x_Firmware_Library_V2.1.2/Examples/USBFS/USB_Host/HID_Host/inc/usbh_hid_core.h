/*!
    \file  usbh_hid_core.h
    \brief header file for usbh_hid_core.c

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

#ifndef USBH_HID_CORE_H
#define USBH_HID_CORE_H

#include "usbh_core.h"
#include "usbh_hcs.h"

#define HID_MIN_POLL                  10U
#define HID_HANDLE_TABLE_SIZE         4U
#define HID_REQ_HANDLE_TABLE_SIZE     5U

#define USBH_HID_REQ_GET_REPORT       0x01U
#define USBH_HID_GET_IDLE             0x02U
#define USBH_HID_GET_PROTOCOL         0x03U
#define USBH_HID_SET_REPORT           0x09U
#define USBH_HID_SET_IDLE             0x0AU
#define USBH_HID_SET_PROTOCOL         0x0BU

#define USB_HID_DESC_SIZE             9U

/* the enum of HID state */
typedef enum
{
    HID_IDLE= 0,
    HID_GET_DATA,
    HID_SYNC,
    HID_POLL,
    HID_ERROR
}hid_sate_enum;

/* the enum of HID REQ state */
typedef enum
{
    HID_REQ_IDLE = 0,
    HID_REQ_GET_HID_DESC,
    HID_REQ_GET_REPORT_DESC,
    HID_REQ_SET_IDLE,
    HID_REQ_SET_PROTOCOL,
}hid_ctrl_state_enum;

/* the HID callback type define */
typedef struct
{
    void (*init)   (void);
    void (*decode) (uint8_t *data);
}hid_cb_struct;

typedef struct
{
    usbh_status_enum (*class_req_polling) (usb_core_handle_struct *pudev, usbh_host_struct *puhost, void *pustate);
    usbh_status_enum (*class_polling)     (usb_core_handle_struct *pudev, usbh_host_struct *puhost, void *pustate);
}class_polling_fun_cb_struct;

/* structure for HID process */
typedef struct
{
    uint8_t              buf[64];
    uint8_t              hc_in_num;
    uint8_t              hc_out_num;
    uint8_t              ep_addr;
    uint8_t              hid_int_out_ep;
    uint8_t              hid_int_in_ep;
    uint16_t             length;
    uint16_t             poll;
    __IO uint16_t        timer;
    hid_sate_enum        state; 
    hid_ctrl_state_enum  ctl_state;
    hid_cb_struct       *cb;
}hid_machine_struct;

typedef struct
{
    uint8_t   bLength;
    uint8_t   bDescriptorType;
    uint16_t  bcdHID;
    uint8_t   bCountryCode;
    uint8_t   bNumDescriptors;
    uint8_t   bReportDescriptorType;
    uint16_t  wItemLength;
}usbh_hid_desc_struct;

/* function declarations */
/* the polling function of HID REQ state */
usbh_status_enum hid_req_state_polling_fun (usb_core_handle_struct *pudev, usbh_host_struct *puhost, void *pustate);
/* the polling function of HID state */
usbh_status_enum hid_state_polling_fun (usb_core_handle_struct *pudev, usbh_host_struct *puhost, void *pustate);
/* de-initialize the host channels used for the HID class */
void usbh_hid_interface_deinit (usb_core_handle_struct *pudev, void *phost);
/* initialize the HID class */
usbh_status_enum usbh_hid_interface_init (usb_core_handle_struct *pudev, void *phost);
#endif  /* USBH_HID_CORE_H */
