/*!
    \file  dual_cdc_acm_core.c
    \brief CDC ACM driver

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

#include "dual_cdc_acm_core.h"
#include "usbd_int.h"

#define USBD_VID                          0x28E9
#define USBD_PID                          0x028B

uint8_t usb_data_buffer[CDC_ACM_DATA_PACKET_SIZE];
uint8_t usb_cmd_buffer[CDC_ACM_CMD_PACKET_SIZE];
uint8_t packet_sent = 1U;
uint8_t packet_receive = 0U;
uint32_t receive_length = 0U;
usbd_int_cb_struct *usbd_int_fops = NULL;
static uint32_t cdc_cmd = 0xFFU;
static __IO uint32_t usbd_cdc_altset = 0U;

extern uint32_t pre_packet_send;
extern usbd_status_enum cdc_acm0_req_handler(void *pudev, usb_device_req_struct *req);
extern usbd_status_enum cdc_acm0_data_handler(void *pudev, usbd_dir_enum rx_tx, uint8_t ep_id);
extern usbd_status_enum cdc_acm0_EP0_RxReady(void  *pudev);
extern usbd_status_enum cdc_acm1_req_handler(void *pudev, usb_device_req_struct *req);
extern usbd_status_enum cdc_acm1_data_handler(void *pudev, usbd_dir_enum rx_tx, uint8_t ep_id);
extern usbd_status_enum cdc_acm1_EP0_RxReady(void  *pudev);

line_coding_struct linecoding =
{
    115200, /* baud rate     */
    0x00,   /* stop bits - 1 */
    0x00,   /* parity - none */
    0x08    /* num of bits 8 */
};

/* note:it should use the C99 standard when compiling the below codes */
/* USB standard device descriptor */
const usb_descriptor_device_struct device_descriptor =
{
    .Header = 
     {
         .bLength = USB_DEVICE_DESC_SIZE, 
         .bDescriptorType = USB_DESCTYPE_DEVICE
     },
    .bcdUSB = 0x0200,
    .bDeviceClass = 0xEF,
    .bDeviceSubClass = 0x02,
    .bDeviceProtocol = 0x01,
    .bMaxPacketSize0 = USBD_EP0_MAX_SIZE,
    .idVendor = USBD_VID,
    .idProduct = USBD_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = USBD_MFC_STR_IDX,
    .iProduct = USBD_PRODUCT_STR_IDX,
    .iSerialNumber = USBD_SERIAL_STR_IDX,
    .bNumberConfigurations = USBD_CFG_MAX_NUM
};

/* USB device configuration descriptor */
usb_descriptor_configuration_set_struct configuration_descriptor = 
{
    .config = 
    {
        .Header = 
         {
            .bLength = sizeof(usb_descriptor_configuration_struct), 
            .bDescriptorType = USB_DESCTYPE_CONFIGURATION
         },
        .wTotalLength = USB_CDC_ACM_CONFIG_DESC_SIZE,
        .bNumInterfaces = 0x04,
        .bConfigurationValue = 0x01,
        .iConfiguration = 0x00,
        .bmAttributes = 0x80,
        .bMaxPower = 0x32
    },

    .cdc_iad0 = 
    {
            .Header = 
         {
             .bLength = sizeof(usb_descriptor_iad_struct), 
             .bDescriptorType = 0x0B 
         },
         .bFirstInterface = 0x00,
         .bInterfaceCount = 0x02,
         .bFunctionClass = 0x02,
         .bFunctionSubClass = 0x02,
         .bFunctionProtocol = 0x01,
         .iFunction = 0x00
    },
    
    .cdc_loopback_interface0 = 
    {
        .Header = 
         {
             .bLength = sizeof(usb_descriptor_interface_struct), 
             .bDescriptorType = USB_DESCTYPE_INTERFACE 
         },
        .bInterfaceNumber = 0x00,
        .bAlternateSetting = 0x00,
        .bNumEndpoints = 0x01,
        .bInterfaceClass = 0x02,
        .bInterfaceSubClass = 0x02,
        .bInterfaceProtocol = 0x01,
        .iInterface = 0x00
    },

    .cdc_loopback_header0 = 
    {
        .Header =
         {
            .bLength = sizeof(usb_descriptor_header_function_struct), 
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype = 0x00,
        .bcdCDC = 0x0110
    },

    .cdc_loopback_call_managment0 = 
    {
        .Header = 
         {
            .bLength = sizeof(usb_descriptor_call_managment_function_struct), 
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype = 0x01,
        .bmCapabilities = 0x00,
        .bDataInterface = 0x01
    },

    .cdc_loopback_acm0 = 
    {
        .Header = 
         {
            .bLength = sizeof(usb_descriptor_acm_function_struct), 
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype = 0x02,
        .bmCapabilities = 0x02,
    },

    .cdc_loopback_union0 = 
    {
        .Header = 
         {
            .bLength = sizeof(usb_descriptor_union_function_struct), 
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype = 0x06,
        .bMasterInterface = 0x00,
        .bSlaveInterface0 = 0x01,
    },

    .cdc_loopback_cmd_endpoint0 = 
    {
        .Header = 
         {
            .bLength = sizeof(usb_descriptor_endpoint_struct), 
            .bDescriptorType = USB_DESCTYPE_ENDPOINT
         },
        .bEndpointAddress = CDC_ACM0_CMD_EP,
        .bmAttributes = 0x03,
        .wMaxPacketSize = CDC_ACM_CMD_PACKET_SIZE,
        .bInterval = 0x0A
    },

    .cdc_loopback_data_interface0 = 
    {
        .Header = 
         {
            .bLength = sizeof(usb_descriptor_interface_struct), 
            .bDescriptorType = USB_DESCTYPE_INTERFACE
         },
        .bInterfaceNumber = 0x01,
        .bAlternateSetting = 0x00,
        .bNumEndpoints = 0x02,
        .bInterfaceClass = 0x0A,
        .bInterfaceSubClass = 0x00,
        .bInterfaceProtocol = 0x00,
        .iInterface = 0x00
    },

    .cdc_loopback_out_endpoint0 = 
    {
        .Header = 
         {
             .bLength = sizeof(usb_descriptor_endpoint_struct), 
             .bDescriptorType = USB_DESCTYPE_ENDPOINT 
         },
        .bEndpointAddress = CDC_ACM0_DATA_OUT_EP,
        .bmAttributes = 0x02,
        .wMaxPacketSize = CDC_ACM_DATA_PACKET_SIZE,
        .bInterval = 0x00
    },

    .cdc_loopback_in_endpoint0 = 
    {
        .Header = 
         {
             .bLength = sizeof(usb_descriptor_endpoint_struct), 
             .bDescriptorType = USB_DESCTYPE_ENDPOINT 
         },
        .bEndpointAddress = CDC_ACM0_DATA_IN_EP,
        .bmAttributes = 0x02,
        .wMaxPacketSize = CDC_ACM_DATA_PACKET_SIZE,
        .bInterval = 0x00
    },
    
    .cdc_iad1 = 
    {
            .Header = 
         {
             .bLength = sizeof(usb_descriptor_iad_struct), 
             .bDescriptorType = 0x0B 
         },
         .bFirstInterface = 0x02,
         .bInterfaceCount = 0x02,
         .bFunctionClass = 0x02,
         .bFunctionSubClass = 0x02,
         .bFunctionProtocol = 0x01,
         .iFunction = 0x00
    },
    
    .cdc_loopback_interface1 = 
    {
        .Header = 
         {
             .bLength = sizeof(usb_descriptor_interface_struct), 
             .bDescriptorType = USB_DESCTYPE_INTERFACE 
         },
        .bInterfaceNumber = 0x02,
        .bAlternateSetting = 0x00,
        .bNumEndpoints = 0x01,
        .bInterfaceClass = 0x02,
        .bInterfaceSubClass = 0x02,
        .bInterfaceProtocol = 0x01,
        .iInterface = 0x00
    },

    .cdc_loopback_header1 = 
    {
        .Header =
         {
            .bLength = sizeof(usb_descriptor_header_function_struct), 
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype = 0x00,
        .bcdCDC = 0x0110
    },

    .cdc_loopback_call_managment1 = 
    {
        .Header = 
         {
            .bLength = sizeof(usb_descriptor_call_managment_function_struct), 
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype = 0x01,
        .bmCapabilities = 0x00,
        .bDataInterface = 0x03
    },

    .cdc_loopback_acm1 = 
    {
        .Header = 
         {
            .bLength = sizeof(usb_descriptor_acm_function_struct), 
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype = 0x02,
        .bmCapabilities = 0x02,
    },

    .cdc_loopback_union1 = 
    {
        .Header = 
         {
            .bLength = sizeof(usb_descriptor_union_function_struct), 
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype = 0x06,
        .bMasterInterface = 0x02,
        .bSlaveInterface0 = 0x03,
    },

    .cdc_loopback_cmd_endpoint1 = 
    {
        .Header = 
         {
            .bLength = sizeof(usb_descriptor_endpoint_struct), 
            .bDescriptorType = USB_DESCTYPE_ENDPOINT
         },
        .bEndpointAddress = CDC_ACM1_CMD_EP,
        .bmAttributes = 0x03,
        .wMaxPacketSize = CDC_ACM_CMD_PACKET_SIZE,
        .bInterval = 0x0A
    },

    .cdc_loopback_data_interface1 = 
    {
        .Header = 
         {
            .bLength = sizeof(usb_descriptor_interface_struct), 
            .bDescriptorType = USB_DESCTYPE_INTERFACE
         },
        .bInterfaceNumber = 0x03,
        .bAlternateSetting = 0x00,
        .bNumEndpoints = 0x02,
        .bInterfaceClass = 0x0A,
        .bInterfaceSubClass = 0x00,
        .bInterfaceProtocol = 0x00,
        .iInterface = 0x00
    },

    .cdc_loopback_out_endpoint1 = 
    {
        .Header = 
         {
             .bLength = sizeof(usb_descriptor_endpoint_struct), 
             .bDescriptorType = USB_DESCTYPE_ENDPOINT 
         },
        .bEndpointAddress = CDC_ACM1_DATA_OUT_EP,
        .bmAttributes = 0x02,
        .wMaxPacketSize = CDC_ACM_DATA_PACKET_SIZE,
        .bInterval = 0x00
    },

    .cdc_loopback_in_endpoint1 = 
    {
        .Header = 
         {
             .bLength = sizeof(usb_descriptor_endpoint_struct), 
             .bDescriptorType = USB_DESCTYPE_ENDPOINT 
         },
        .bEndpointAddress = CDC_ACM1_DATA_IN_EP,
        .bmAttributes = 0x02,
        .wMaxPacketSize = CDC_ACM_DATA_PACKET_SIZE,
        .bInterval = 0x00
    }
};

/* USB language ID Descriptor */
const usb_descriptor_language_id_struct usbd_language_id_desc = 
{
    .Header = 
     {
         .bLength = sizeof(usb_descriptor_language_id_struct), 
         .bDescriptorType = USB_DESCTYPE_STRING
     },
    .wLANGID = ENG_LANGID
};

void *const usbd_strings[] = 
{
    [USBD_LANGID_STR_IDX] = (uint8_t *)&usbd_language_id_desc,
    [USBD_MFC_STR_IDX] = USBD_STRING_DESC("GigaDevice"),
    [USBD_PRODUCT_STR_IDX] = USBD_STRING_DESC("GD32 USB Dual CDC ACM "),
    [USBD_SERIAL_STR_IDX] = USBD_STRING_DESC("GD32Fxxx -7z8x9yer")
};

/*!
    \brief      initialize the CDC ACM device
    \param[in]  pudev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
usbd_status_enum dual_cdc_acm_init (void *pudev, uint8_t config_index)
{
    /* initialize the data Tx/Rx endpoint */
    usbd_ep_init(pudev, ENDP_SNG_BUF, &(configuration_descriptor.cdc_loopback_in_endpoint0));
    usbd_ep_init(pudev, ENDP_SNG_BUF, &(configuration_descriptor.cdc_loopback_out_endpoint0));
    usbd_ep_init(pudev, ENDP_SNG_BUF, &(configuration_descriptor.cdc_loopback_in_endpoint1));
    usbd_ep_init(pudev, ENDP_SNG_BUF, &(configuration_descriptor.cdc_loopback_out_endpoint1));

    /* initialize the command Tx endpoint */
    usbd_ep_init(pudev, ENDP_SNG_BUF, &(configuration_descriptor.cdc_loopback_cmd_endpoint0));
    usbd_ep_init(pudev, ENDP_SNG_BUF, &(configuration_descriptor.cdc_loopback_cmd_endpoint1));

    return USBD_OK;
}

/*!
    \brief      de-initialize the CDC ACM device
    \param[in]  pudev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
usbd_status_enum dual_cdc_acm_deinit (void *pudev, uint8_t config_index)
{
    /* deinitialize the data Tx/Rx endpoint */
    usbd_ep_deinit(pudev, CDC_ACM0_DATA_IN_EP);
    usbd_ep_deinit(pudev, CDC_ACM0_DATA_OUT_EP);
    usbd_ep_deinit(pudev, CDC_ACM1_DATA_IN_EP);
    usbd_ep_deinit(pudev, CDC_ACM1_DATA_OUT_EP);

    /* deinitialize the command Tx endpoint */
    usbd_ep_deinit(pudev, CDC_ACM0_CMD_EP);
    usbd_ep_deinit(pudev, CDC_ACM1_CMD_EP);

    return USBD_OK;
}

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
usbd_status_enum  dual_cdc_acm_data_handler (void *pudev, usbd_dir_enum rx_tx, uint8_t ep_id)
{
    if (((CDC_ACM0_DATA_IN_EP & 0x7F) == ep_id) || (ep_id == (CDC_ACM0_DATA_OUT_EP & 0x7F))) {
        return cdc_acm0_data_handler(pudev, rx_tx, ep_id);
    } else if (((CDC_ACM1_DATA_IN_EP & 0x7F) == ep_id) || (ep_id == (CDC_ACM1_DATA_OUT_EP & 0x7F))) {
        return cdc_acm1_data_handler(pudev, rx_tx, ep_id);
    } else if ((USBD_RX == rx_tx) && (EP0_OUT & 0x7F) == ep_id) {
        cdc_acm0_EP0_RxReady(pudev);
        cdc_acm1_EP0_RxReady(pudev);
        return USBD_OK;
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
usbd_status_enum dual_cdc_acm_req_handler (void *pudev, usb_device_req_struct *req)
{
    if ((req->wIndex & 0xFF) == 0x00) {
        return cdc_acm0_req_handler(pudev, req);
    } else if ((req->wIndex & 0xFF) == 0x02)  {
        return cdc_acm1_req_handler(pudev, req);
    }


    return USBD_OK;
}

/*!
    \brief      receive CDC ACM data
    \param[in]  pudev: pointer to USB device instance
    \param[out] none
    \retval     USB device operation status
*/
void cdc_acm_data_receive(void *pudev)
{
    packet_receive = 0;
    usbd_ep_rx(pudev, CDC_ACM0_DATA_OUT_EP, (uint8_t*)(usb_data_buffer), CDC_ACM_DATA_PACKET_SIZE);
    usbd_ep_rx(pudev, CDC_ACM1_DATA_OUT_EP, (uint8_t*)(usb_data_buffer), CDC_ACM_DATA_PACKET_SIZE);
}

/*!
    \brief      send CDC ACM data
    \param[in]  pudev: pointer to USB device instance
    \param[out] none
    \retval     USB device operation status
*/
void cdc_acm_data_send (void *pudev, uint32_t data_len)
{
    /* limit the transfer data length */
    if (data_len <= CDC_ACM_DATA_PACKET_SIZE) {
        packet_sent = 0;
        usbd_ep_tx(pudev, CDC_ACM0_DATA_IN_EP, (uint8_t*)(usb_data_buffer), data_len);
        usbd_ep_tx(pudev, CDC_ACM1_DATA_IN_EP, (uint8_t*)(usb_data_buffer), data_len);
    }
}

/*!
    \brief      command data received on control endpoint
    \param[in]  pudev: pointer to USB device instance
    \param[out] none
    \retval     USB device operation status
*/
usbd_status_enum cdc_acm_EP0_RxReady (void *pudev)
{
    if (NO_CMD != cdc_cmd) {
        /* process the command data */
        linecoding.dwDTERate = (uint32_t)(usb_cmd_buffer[0] | 
                                         (usb_cmd_buffer[1] << 8) |
                                         (usb_cmd_buffer[2] << 16) |
                                         (usb_cmd_buffer[3] << 24));

        linecoding.bCharFormat = usb_cmd_buffer[4];
        linecoding.bParityType = usb_cmd_buffer[5];
        linecoding.bDataBits = usb_cmd_buffer[6];

        packet_receive = 1;

        cdc_cmd = NO_CMD;
    }

    return USBD_OK;
}
