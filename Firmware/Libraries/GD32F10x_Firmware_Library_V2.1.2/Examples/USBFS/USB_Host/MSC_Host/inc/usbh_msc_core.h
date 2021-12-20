/*!
    \file  usbh_msc_core.h
    \brief this file contains all the prototypes for the usbh_msc_core.c

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

#ifndef USBH_MSC_CORE_H
#define USBH_MSC_CORE_H

#include "usbh_core.h"
#include "usbh_ctrl.h"
#include "usbh_hcs.h"
#include "usbh_msc_core.h"
#include "usbh_msc_scsi.h"
#include "usbh_msc_bot.h"

/* structure for MSC process */
typedef struct
{
    uint8_t              hc_num_in; 
    uint8_t              hc_num_out; 
    uint8_t              msc_bulk_out_ep;
    uint8_t              msc_bulk_in_ep;
    uint16_t             msc_bulk_in_ep_size;
    uint16_t             msc_bulk_out_ep_size;
    uint8_t              buff[USBH_MSC_MPS_SIZE];
    uint8_t              maxLun;
}msc_machine_struct; 

typedef struct
{
    usbh_status_enum (*class_req_polling)  (usb_core_handle_struct *pudev, usbh_host_struct *puhost, void *pustate);
    usbh_status_enum (*class_polling)      (usb_core_handle_struct *pudev, usbh_host_struct *puhost, void *pustate);
}class_polling_fun_cb_struct;

#define USB_REQ_BOT_RESET           0xFFU
#define USB_REQ_GET_MAX_LUN         0xFEU

extern msc_machine_struct           msc_machine;
extern uint8_t                      msc_error_count;

/* function declarations */
/* de-initialize interface by freeing host channels allocated to interface */
extern void usbh_msc_interface_deinit (usb_core_handle_struct *pudev, void *puhost);
/* interface initialization for MSC class */
extern usbh_status_enum usbh_msc_interface_init (usb_core_handle_struct *pudev, void *puhost);
/* clear or disable a specific feature. */
extern usbh_status_enum usbh_clear_feature (usb_core_handle_struct *pudev,
                                            usbh_host_struct *puhost,
                                            uint8_t ep_addr, 
                                            uint8_t hc_num);
#endif  /* USBH_MSC_CORE_H */
