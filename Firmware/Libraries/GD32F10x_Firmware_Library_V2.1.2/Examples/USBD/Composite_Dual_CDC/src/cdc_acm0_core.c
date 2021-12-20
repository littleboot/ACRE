/*!
    \file  cdc_acm0_core.c
    \brief USB dual CDC ACM device

    \version 2014-12-26, V1.0.0, firmware for GD32F10x
    \version 2017-06-20, V2.0.0, firmware for GD32F10x
    \version 2018-07-31, V2.1.0, firmware for GD32F10x
*/
/*
    Copyright (C) 2018 GigaDevice

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

#include "cdc_acm0_core.h"
#include "usbd_int.h"

static uint32_t cdc_cmd = 0xFFU;
static __IO uint32_t usbd_cdc_altset = 0U;
extern uint32_t pre_packet_send;
uint8_t usb_cmd0_buffer[CDC_ACM_CMD_PACKET_SIZE];

extern uint8_t packet_sent ;
extern uint8_t packet_receive ;
extern uint32_t receive_length ;

line_coding_struct linecoding0 =
{
    115200, /* baud rate     */
    0x00,   /* stop bits - 1 */
    0x00,   /* parity - none */
    0x08    /* num of bits 8 */
};

/*!
    \brief      handle CDC ACM data
    \param[in]  pudev: pointer to USB device instance
    \param[in]  rx_tx: data transfer direction:
      \arg        USBD_TX
      \arg        USBD_RX
    \param[in]  ep_id: endpoint identifier
    \param[out] none
    \retval     USB device operation status
*/
usbd_status_enum  cdc_acm0_data_handler (void *pudev, usbd_dir_enum rx_tx, uint8_t ep_id)
{
    if ((USBD_TX == rx_tx) && ((CDC_ACM0_DATA_IN_EP & 0x7F) == ep_id)) {
        usb_ep_struct *ep = &((usbd_core_handle_struct *)(pudev))->in_ep[ep_id];
        
        if (ep->trs_count == ep->maxpacket) {
            usbd_ep_tx(pudev, ep_id, NULL, 0U);
        } else {
            packet_sent = 1U;
            pre_packet_send = 1U;
        }
        return USBD_OK;
    } else if ((USBD_RX == rx_tx) && ((EP0_OUT & 0x7FU) == ep_id)) {
        cdc_acm0_EP0_RxReady (pudev);
    } else if ((USBD_RX == rx_tx) && ((CDC_ACM0_DATA_OUT_EP & 0x7FU) == ep_id)) {
        packet_receive = 1U;
        receive_length = usbd_rx_count_get(pudev, CDC_ACM0_DATA_OUT_EP);
        return USBD_OK;
    } else {

    }

    return USBD_FAIL;
}

/*!
    \brief      handle the CDC ACM class-specific requests
    \param[in]  pudev: pointer to USB device instance
    \param[in]  req: device class-specific request
    \param[out] none
    \retval     USB device operation status
*/
usbd_status_enum cdc_acm0_req_handler (void *pudev, usb_device_req_struct *req)
{
    uint16_t len = CDC_ACM_DESC_SIZE;
    uint8_t  *pbuf= (uint8_t*)(&configuration_descriptor) + 9;

    switch (req->bmRequestType & USB_REQ_MASK) {
    case USB_CLASS_REQ:
        switch (req->bRequest) {
        case SEND_ENCAPSULATED_COMMAND:
            break;
        case GET_ENCAPSULATED_RESPONSE:
            break;
        case SET_COMM_FEATURE:
            break;
        case GET_COMM_FEATURE:
            break;
        case CLEAR_COMM_FEATURE:
            break;
        case SET_LINE_CODING:
            /* set the value of the current command to be processed */
            cdc_cmd = req->bRequest;
            /* enable EP0 prepare to receive command data packet */
            usbd_ep_rx (pudev, EP0_OUT, usb_cmd0_buffer, req->wLength);
            break;
        case GET_LINE_CODING:
            usb_cmd0_buffer[0] = (uint8_t)(linecoding0.dwDTERate);
            usb_cmd0_buffer[1] = (uint8_t)(linecoding0.dwDTERate >> 8);
            usb_cmd0_buffer[2] = (uint8_t)(linecoding0.dwDTERate >> 16);
            usb_cmd0_buffer[3] = (uint8_t)(linecoding0.dwDTERate >> 24);
            usb_cmd0_buffer[4] = linecoding0.bCharFormat;
            usb_cmd0_buffer[5] = linecoding0.bParityType;
            usb_cmd0_buffer[6] = linecoding0.bDataBits;
            /* send the request data to the host */
            usbd_ep_tx (pudev, EP0_IN, usb_cmd0_buffer, req->wLength);
            break;
        case SET_CONTROL_LINE_STATE:
            /* detect DTE present */
            if(req->wValue == 0x0001){
                /* operation reserved */
            }
            break;
        case SEND_BREAK:
            break;
        default:
            break;
        }
        break;
    case USB_STANDARD_REQ:
        /* standard device request */
        switch(req->bRequest) {
        case USBREQ_GET_INTERFACE:
            usbd_ep_tx(pudev, EP0_IN, (uint8_t *)&usbd_cdc_altset, 1);
            break;
        case USBREQ_SET_INTERFACE:
            if ((uint8_t)(req->wValue) < USBD_ITF_MAX_NUM) {
                usbd_cdc_altset = (uint8_t)(req->wValue);
            } else {
                /* call the error management function (command will be nacked */
                usbd_enum_error (pudev, req);
            }
            break;
        case USBREQ_GET_DESCRIPTOR:
            if(CDC_ACM_DESC_TYPE == (req->wValue >> 8)){
                len = MIN(CDC_ACM_DESC_SIZE, req->wLength);
                pbuf = (uint8_t*)(&configuration_descriptor) + 9 + (9 * USBD_ITF_MAX_NUM);
            }

            usbd_ep_tx(pudev, EP0_IN, pbuf, len);
            break;
        default:
            break;
        }
    default:
        break;
    }

    return USBD_OK;
}

/*!
    \brief      command data received on control endpoint
    \param[in]  pudev: pointer to USB device instance
    \param[out] none
    \retval     USB device operation status
*/
usbd_status_enum cdc_acm0_EP0_RxReady (void *pudev)
{
    if (NO_CMD != cdc_cmd) {
        /* process the command data */
        linecoding0.dwDTERate = (uint32_t)(usb_cmd0_buffer[0] | 
                                         (usb_cmd0_buffer[1] << 8) |
                                         (usb_cmd0_buffer[2] << 16) |
                                         (usb_cmd0_buffer[3] << 24));

        linecoding0.bCharFormat = usb_cmd0_buffer[4];
        linecoding0.bParityType = usb_cmd0_buffer[5];
        linecoding0.bDataBits = usb_cmd0_buffer[6];

        packet_receive = 1;

        cdc_cmd = NO_CMD;
    }

    return USBD_OK;
}
