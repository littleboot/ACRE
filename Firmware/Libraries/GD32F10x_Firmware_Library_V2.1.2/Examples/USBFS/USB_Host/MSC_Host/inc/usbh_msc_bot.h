/*!
    \file  usbh_msc_bot.h
    \brief header file for usbh_msc_bot.c

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

#ifndef USBH_MSC_BOT_H
#define USBH_MSC_BOT_H

#include <stdint.h>
#include "usb_core.h"
#include "usbh_core.h"

typedef union 
{
    struct
    {
        uint32_t cbw_signature;
        uint32_t cbw_tag;
        uint32_t cbw_transfer_length;
        uint8_t  cbw_flags;
        uint8_t  cbw_lun; 
        uint8_t  cbw_length;
        uint8_t  cbwcb[16];
    }field;

    uint8_t cbw_array[31];
}host_cbwpkt_union;

typedef enum
{
    USBH_MSC_BOT_INIT_STATE = 0U,
    USBH_MSC_BOT_RESET,
    USBH_MSC_GET_MAX_LUN,
    USBH_MSC_TEST_UNIT_READY,
    USBH_MSC_READ_CAPACITY10,
    USBH_MSC_MODE_SENSE6,
    USBH_MSC_REQUEST_SENSE,
    USBH_MSC_BOT_USB_TRANSFERS,
    USBH_MSC_DEFAULT_APPLI_STATE,
    USBH_MSC_CTRL_ERROR_STATE,
    USBH_MSC_UNRECOVERED_STATE
}msc_state_enum;

typedef struct 
{
    uint8_t msc_state;
    uint8_t msc_state_bkp;
    uint8_t msc_state_current;
    uint8_t cmd_state_machine;
    uint8_t bot_state;
    uint8_t bot_state_bkp;
    uint8_t* p_rx_tx_buff;
    uint16_t data_length;
    uint8_t bot_xfer_error_count;
    uint8_t bot_xfer_status;
}usbh_botxfer_struct;

typedef union 
{
    struct
    {
        uint32_t csw_signature;
        uint32_t csw_tag;
        uint32_t csw_dataresidue;
        uint8_t  csw_status;
    }field;

    uint8_t csw_array[13];
}host_cswpkt_union;

#define USBH_MSC_SEND_CBW                   1U
#define USBH_MSC_SENT_CBW                   2U
#define USBH_MSC_BOT_DATAIN_STATE           3U
#define USBH_MSC_BOT_DATAOUT_STATE          4U
#define USBH_MSC_RECEIVE_CSW_STATE          5U
#define USBH_MSC_DECODE_CSW                 6U
#define USBH_MSC_BOT_ERROR_IN               7U
#define USBH_MSC_BOT_ERROR_OUT              8U

#define USBH_MSC_BOT_CBW_SIGNATURE          0x43425355U
#define USBH_MSC_BOT_CBW_TAG                0x20304050U
#define USBH_MSC_BOT_CSW_SIGNATURE          0x53425355U
#define USBH_MSC_CSW_DATA_LENGTH            0x000DU
#define USBH_MSC_BOT_CBW_PACKET_LENGTH      31U
#define USBH_MSC_CSW_LENGTH                 13U
#define USBH_MSC_CSW_MAX_LENGTH             63U

/* CSW status definitions */
#define USBH_MSC_CSW_CMD_PASSED             0x00U
#define USBH_MSC_CSW_CMD_FAILED             0x01U
#define USBH_MSC_CSW_PHASE_ERROR            0x02U

#define USBH_MSC_SEND_CSW_DISABLE           0U
#define USBH_MSC_SEND_CSW_ENABLE            1U

#define USBH_MSC_DIR_IN                     0U
#define USBH_MSC_DIR_OUT                    1U
#define USBH_MSC_BOTH_DIR                   2U

#define USBH_MSC_PAGE_LENGTH                512U


#define CBW_CB_LENGTH                       16U
#define CBW_LENGTH                          10U
#define CBW_LENGTH_TEST_UNIT_READY          6U

#define USB_REQ_BOT_RESET                   0xFFU
#define USB_REQ_GET_MAX_LUN                 0xFEU

#define MAX_BULK_STALL_COUNT_LIMIT          0x40U   /* if stall is seen on bulk 
                                            endpoint continously, this means 
                                            that device and host has phase error
                                            hence a reset is needed */

extern usbh_botxfer_struct usbh_msc_botxfer_param;
extern host_cbwpkt_union usbh_msc_cbw_data;
extern host_cswpkt_union usbh_msc_csw_data;

/* function declarations */
/* initialize the mass storage parameters */
void usbh_msc_init (usb_core_handle_struct *pudev);
/* manage the different states of BOT transfer and updates the status to upper layer */
void usbh_msc_handle_botxfer (usb_core_handle_struct *pudev,
                              usbh_host_struct *puhost ,
                              usbh_state_handle_struct* pustate);
/* decode the CSW received by the device and updates the same to upper layer */
uint8_t usbh_msc_decode_csw (usb_core_handle_struct *pudev, usbh_host_struct *puhost);
/* manages the different error handling for STALL */
usbh_status_enum usbh_msc_bot_abort (usb_core_handle_struct *pudev, 
                                     usbh_host_struct *puhost,
                                     usbh_state_handle_struct* pustate,
                                     uint8_t direction);
#endif /* USBH_MSC_BOT_H */
