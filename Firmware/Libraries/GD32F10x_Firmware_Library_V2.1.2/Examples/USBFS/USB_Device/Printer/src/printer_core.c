/*!
    \file  printer_core.c
    \brief USB printer device class core functions

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

#include "printer_core.h"
#include "usbd_int.h"

#define USBD_VID                     0x28e9
#define USBD_PID                     0x028d

/* printer port status: paper not empty/selected/no error */
static uint8_t g_port_status = 0x18;
static uint32_t g_usbd_printer_altset = 0;

uint8_t g_printer_data_buf[PRINTER_OUT_PACKET];

usbd_int_cb_struct *usbd_int_fops = NULL;

__ALIGN_BEGIN uint8_t DEVICE_ID[DEVICE_ID_LEN] __ALIGN_END =
{
    0x00, 0x67,
    'M', 'A', 'N', 'U', 'F', 'A', 'C', 'T', 'U', 'R', 'E', 'R', ':',
    'G', 'I', 'G', 'A', ' ', 'D', 'E', 'V', 'I', 'C', 'E', '-', ';',
    'C', 'O', 'M', 'M', 'A', 'N', 'D', ' ', 'S', 'E', 'T', ':',
    'P', 'C', 'L', ',', 'M', 'P', 'L', ';',
    'M', 'O', 'D', 'E', 'L', ':',
    'L', 'a', 's', 'e', 'r', 'B', 'e', 'a', 'm', '?', ';',
    'C', 'O', 'M', 'M', 'E', 'N', 'T', ':',
    'G', 'o', 'o', 'd', ' ', '!', ';',
    'A', 'C', 'T', 'I', 'V', 'E', ' ', 'C', 'O', 'M', 'M', 'A', 'N', 'D', ' ', 'S', 'E', 'T', ':',
    'P', 'C', 'L', ';'
};

/* note:it should use the c99 standard when compiling the below codes */
/* USB standard device descriptor */
__ALIGN_BEGIN const usb_descriptor_device_struct device_descripter __ALIGN_END =
{
    .Header = 
     {
         .bLength = USB_DEVICE_DESC_SIZE, 
         .bDescriptorType = USB_DESCTYPE_DEVICE
     },
    .bcdUSB = 0x0200,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = USB_MAX_EP0_SIZE,
    .idVendor = USBD_VID,
    .idProduct = USBD_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = USBD_MFC_STR_IDX,
    .iProduct = USBD_PRODUCT_STR_IDX,
    .iSerialNumber = USBD_SERIAL_STR_IDX,
    .bNumberConfigurations = USBD_CFG_MAX_NUM
};

/* USB device configuration descriptor */
__ALIGN_BEGIN const usb_descriptor_configuration_set_struct configuration_descriptor __ALIGN_END = 
{
    .Config = 
    {
        .Header = 
         {
             .bLength = sizeof(usb_descriptor_configuration_struct), 
             .bDescriptorType = USB_DESCTYPE_CONFIGURATION 
         },
        .wTotalLength = USB_PRINTER_CONFIG_DESC_SIZE,
        .bNumInterfaces = 0x01,
        .bConfigurationValue = 0x01,
        .iConfiguration = 0x00,
        .bmAttributes = 0xA0,
        .bMaxPower = 0x32
    },

    .Printer_Interface = 
    {
        .Header = 
         {
             .bLength = sizeof(usb_descriptor_interface_struct), 
             .bDescriptorType = USB_DESCTYPE_INTERFACE 
         },
        .bInterfaceNumber = 0x00,
        .bAlternateSetting = 0x00,
        .bNumEndpoints = 0x02,
        .bInterfaceClass = 0x07,
        .bInterfaceSubClass = 0x01,
        .bInterfaceProtocol = 0x02,
        .iInterface = 0x00
    },

    .Printer_IN_Endpoint = 
    {
        .Header = 
         {
             .bLength = sizeof(usb_descriptor_endpoint_struct), 
             .bDescriptorType = USB_DESCTYPE_ENDPOINT 
         },
        .bEndpointAddress = PRINTER_IN_EP,
        .bmAttributes = 0x02,
        .wMaxPacketSize = PRINTER_IN_PACKET,
        .bInterval = 0x00
    },

    .Printer_OUT_Endpoint = 
    {
        .Header = 
         {
             .bLength = sizeof(usb_descriptor_endpoint_struct), 
             .bDescriptorType = USB_DESCTYPE_ENDPOINT 
         },
        .bEndpointAddress = PRINTER_OUT_EP,
        .bmAttributes = 0x02,
        .wMaxPacketSize = PRINTER_OUT_PACKET,
        .bInterval = 0x00
    },
};

/* USB language ID Descriptor */
__ALIGN_BEGIN const usb_descriptor_language_id_struct usbd_language_id_desc __ALIGN_END = 
{
    .Header = 
     {
         .bLength = sizeof(usb_descriptor_language_id_struct), 
         .bDescriptorType = USB_DESCTYPE_STRING
     },
    .wLANGID = ENG_LANGID
};

__ALIGN_BEGIN uint8_t* usbd_strings[] __ALIGN_END = 
{
    [USBD_LANGID_STR_IDX] = (uint8_t *)&usbd_language_id_desc,
    [USBD_MFC_STR_IDX] = USBD_STRING_DESC("GigaDevice"),
    [USBD_PRODUCT_STR_IDX] = USBD_STRING_DESC("GD32 USB Printer in FS Mode"),
    [USBD_SERIAL_STR_IDX] = USBD_STRING_DESC("GD32F20X-2.0.0-5f9e10dma")
};

/*!
    \brief      initialize the printer device
    \param[in]  pudev: pointer to usb device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     usb device operation status
*/
uint8_t printer_init (void *pudev, uint8_t config_index)
{
    /* initialize Tx endpoint */
    usbd_ep_init(pudev, &(configuration_descriptor.Printer_IN_Endpoint));

    /* initialize Rx endpoint */
    usbd_ep_init(pudev, &(configuration_descriptor.Printer_OUT_Endpoint));

    return USBD_OK;

}

/*!
    \brief      de-initialize the printer device
    \param[in]  pudev: pointer to usb device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     usb device operation status
*/
uint8_t  printer_deinit (void *pudev, uint8_t config_index)
{
    /* deinitialize HID endpoints */
    usbd_ep_deinit (pudev, PRINTER_IN_EP);
    usbd_ep_deinit (pudev, PRINTER_OUT_EP);

    return USBD_OK;
}

/*!
    \brief      handle the printer class-specific requests
    \param[in]  pudev: pointer to usb device instance
    \param[in]  req: device class-specific request
    \param[out] none
    \retval     usb device operation status
*/
uint8_t printer_req_handler (void *pudev, usb_device_req_struct *req)
{
    switch (req->bmRequestType & USB_REQ_MASK) {
    case USB_CLASS_REQ:
        switch (req->bRequest) {
        case GET_DEVICE_ID:
            usbd_ep_tx (pudev, EP0_IN, DEVICE_ID, DEVICE_ID_LEN);
            break;

        case GET_PORT_STATUS:
            usbd_ep_tx (pudev, EP0_IN, (uint8_t *)&g_port_status, 1);
            break;

        case SOFT_RESET:
            usbd_ep_rx(pudev, PRINTER_OUT_EP, g_printer_data_buf, PRINTER_OUT_PACKET);
            break;

        default:
            usbd_enum_error (pudev, req);
            return USBD_FAIL; 
    }
    case USB_STANDARD_REQ:
        /* standard device request */
        switch(req->bRequest) {
        case USBREQ_GET_INTERFACE:
            usbd_ep_tx (pudev, EP0_IN, (uint8_t *)&g_usbd_printer_altset, 1);
            break;
        case USBREQ_SET_INTERFACE:
            g_usbd_printer_altset = (uint8_t)(req->wValue);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }

    return USBD_OK;
}

/*!
    \brief      handle data stage
    \param[in]  pudev: pointer to usb device instance
    \param[in]  rx_tx: the flag of Rx or Tx
    \param[in]  ep_id: the endpoint ID
    \param[out] none
    \retval     usb device operation status
*/
uint8_t  printer_data_handler (void *pudev, usb_dir_enum rx_tx, uint8_t ep_id)
{
    if ((USB_TX == rx_tx) && ((PRINTER_IN_EP & 0x7F) == ep_id)) {
      
        return USBD_OK;
    } else if ((USB_RX == rx_tx) && ((PRINTER_OUT_EP & 0x7F) == ep_id)) {
      
        return USBD_OK;
    }
    return USBD_FAIL;
}
