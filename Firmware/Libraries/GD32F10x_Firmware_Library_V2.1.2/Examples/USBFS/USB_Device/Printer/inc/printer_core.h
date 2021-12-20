/*!
    \file  printer_core.h
    \brief the header file of USB printer device class core functions

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

#ifndef PRINTER_CORE_H
#define PRINTER_CORE_H

#include "usbd_std.h"

#define DEVICE_ID_LEN                   103

#define USB_PRINTER_CONFIG_DESC_SIZE    32

#define GET_DEVICE_ID                   0x00
#define GET_PORT_STATUS                 0x01
#define SOFT_RESET                      0x02

/* USB configuration descriptor struct */
typedef struct
{
    usb_descriptor_configuration_struct        Config;
    usb_descriptor_interface_struct            Printer_Interface;
    usb_descriptor_endpoint_struct             Printer_IN_Endpoint;
    usb_descriptor_endpoint_struct             Printer_OUT_Endpoint;
} usb_descriptor_configuration_set_struct;

extern uint8_t* usbd_strings[USB_STRING_COUNT];
extern const usb_descriptor_device_struct device_descripter;
extern const usb_descriptor_configuration_set_struct configuration_descriptor;

/* function declarations */
/* initialize the printer device */
uint8_t printer_init         (void *pudev, uint8_t config_index);
/* de-initialize the printer device */
uint8_t printer_deinit       (void *pudev, uint8_t config_index);
/* handle the printer class-specific requests */
uint8_t printer_req_handler  (void *pudev, usb_device_req_struct *req);
/* handle data Stage */
uint8_t printer_data_handler (void *pudev, usb_dir_enum rx_tx, uint8_t ep_id);

#endif  /* PRINTER_CORE_H */
