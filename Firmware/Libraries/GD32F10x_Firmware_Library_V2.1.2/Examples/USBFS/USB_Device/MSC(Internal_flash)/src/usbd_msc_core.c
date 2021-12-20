/*!
    \file  usbd_msc_core.c
    \brief USB MSC device class core driver

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

#include "usbd_std.h"
#include "usbd_msc_core.h"
#include "usbd_flash_access.h"
#include "usbd_bbb_scsi.h"
#include "usbd_int.h"

#define USBD_VID                     0x28E9
#define USBD_PID                     0x028F

extern bbb_cbw_struct bbb_cbw;
extern bbb_csw_struct bbb_csw;
extern uint8_t g_bbb_transport_stage;

static uint8_t usbd_msc_maxlun = 0;
static uint8_t usbd_msc_altset = 0;

usbd_int_cb_struct *usbd_int_fops = NULL;

/* note: it should use the C99 standard when compiling the below codes */
/* USB standard device descriptor */
__ALIGN_BEGIN const usb_descriptor_device_struct device_descripter __ALIGN_END =
{
    .Header ={.bLength = USB_DEVICE_DESC_SIZE, .bDescriptorType = USB_DESCTYPE_DEVICE},
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
    .config = 
    {
        .Header = 
        {
            .bLength = sizeof(usb_descriptor_configuration_struct),
            .bDescriptorType = USB_DESCTYPE_CONFIGURATION
        },
        .wTotalLength = USB_MSC_CONFIG_DESC_SIZE,
        .bNumInterfaces = 0x01,
        .bConfigurationValue = 0x01,
        .iConfiguration = 0x04,
        .bmAttributes = 0xC0,
        .bMaxPower = 0x32
    },

    .msc_interface = 
    {
        .Header = 
        {
            .bLength = sizeof(usb_descriptor_interface_struct), 
            .bDescriptorType = USB_DESCTYPE_INTERFACE
        },
        .bInterfaceNumber = 0x00,
        .bAlternateSetting = 0x00,
        .bNumEndpoints = 0x02,
        .bInterfaceClass = 0x08,
        .bInterfaceSubClass = 0x06,
        .bInterfaceProtocol = 0x50,
        .iInterface = 0x00
    },

    .msc_in_endpoint = 
    {
        .Header = 
        {
            .bLength = sizeof(usb_descriptor_endpoint_struct), 
            .bDescriptorType = USB_DESCTYPE_ENDPOINT
        },
        .bEndpointAddress = MSC_IN_EP,
        .bmAttributes = 0x02,
        .wMaxPacketSize = MSC_EPIN_SIZE,
        .bInterval = 0x00
    },

    .msc_out_endpoint = 
    {
        .Header = 
        {
            .bLength = sizeof(usb_descriptor_endpoint_struct), 
            .bDescriptorType = USB_DESCTYPE_ENDPOINT
        },
        .bEndpointAddress = MSC_OUT_EP,
        .bmAttributes = 0x02,
        .wMaxPacketSize = MSC_EPOUT_SIZE,
        .bInterval = 0x00
    }
};

/* USB language ID descriptor */
__ALIGN_BEGIN const usb_descriptor_language_id_struct usbd_language_id_desc __ALIGN_END = 
{
    .Header = 
     {
         .bLength = sizeof(usb_descriptor_language_id_struct), 
         .bDescriptorType = USB_DESCTYPE_STRING
     },
    .wLANGID = ENG_LANGID
};

/* usb string descriptor */
__ALIGN_BEGIN uint8_t* usbd_strings[] __ALIGN_END = 
{
    [USBD_LANGID_STR_IDX] = (uint8_t *)&usbd_language_id_desc,
    [USBD_MFC_STR_IDX] = USBD_STRING_DESC("GigaDevice"),
    [USBD_PRODUCT_STR_IDX] = USBD_STRING_DESC("GD32 USB MSC in FS Mode"),
    [USBD_SERIAL_STR_IDX] = USBD_STRING_DESC("GD32F10X-V2.0.0-7z8y2wu")
};

/*!
    \brief      initialize the MSC device
    \param[in]  pudev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
uint8_t msc_init (void *pudev, uint8_t config_index)
{
    /* initialize Tx endpoint */
    usbd_ep_init(pudev, &(configuration_descriptor.msc_in_endpoint));

    /* initialize Rx endpoint */
    usbd_ep_init(pudev, &(configuration_descriptor.msc_out_endpoint));

    flash_init ();

    g_bbb_transport_stage = COMMAND_STAGE;

    /* prepare endpoint to receive first bbb_cbw */
    usbd_ep_rx (pudev,MSC_OUT_EP,(uint8_t *)&bbb_cbw, BBB_CBW_LENGTH);

    return USBD_OK;

}

/*!
    \brief      de-initialize the MSC device
    \param[in]  pudev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
uint8_t  msc_deinit (void *pudev, uint8_t config_index)
{
    /* deinitialize MSC endpoints */
    usbd_ep_deinit (pudev, MSC_IN_EP);
    usbd_ep_deinit (pudev, MSC_OUT_EP);

    g_bbb_transport_stage = COMMAND_STAGE;

    /* prepare endpoint to receive first bbb_cbw */
    usbd_ep_rx (pudev,MSC_OUT_EP,(uint8_t *)&bbb_cbw, BBB_CBW_LENGTH);

    return USBD_OK;
}

/*!
    \brief      handle the MSC class-specific and standard requests
    \param[in]  pudev: pointer to USB device instance
    \param[in]  req: device class-specific request
    \param[out] none
    \retval     USB device operation status
*/
uint8_t msc_req_handler (void *pudev, usb_device_req_struct *req)
{
    switch (req->bmRequestType & USB_REQ_MASK) {
    case USB_CLASS_REQ:
        switch (req->bRequest) {
        case BBB_GET_MAX_LUN:
            usbd_msc_maxlun = STORAGE_LUN_NUM - 1;
            g_bbb_transport_stage = COMMAND_STAGE;
            usbd_ctltx (pudev, (uint8_t *)&usbd_msc_maxlun, 1);
            break;
        case BBB_RESET :
            g_bbb_transport_stage = COMMAND_STAGE;

            /* prepare endpoint to receive first bbb_cbw */
            usbd_ep_rx (pudev, MSC_OUT_EP, (uint8_t *)&bbb_cbw, BBB_CBW_LENGTH);
            break;
        default:
            usbd_enum_error(pudev, req);
            return USBD_FAIL; 
        }
        break;
    case USB_STANDARD_REQ:
        /* standard device request */
        switch(req->bRequest) {
        case USBREQ_GET_INTERFACE:
            usbd_ctltx (pudev, (uint8_t *)&usbd_msc_altset, 1);
            break;
        case USBREQ_SET_INTERFACE:
            usbd_msc_altset = (uint8_t)(req->wValue);
            break;
        case USBREQ_CLEAR_FEATURE:
            usbd_ep_stall(pudev, MSC_IN_EP);
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
    \brief      handle data stage
    \param[in]  pudev: pointer to USB device instance
    \param[in]  rx_tx: the flag of Rx or Tx
    \param[in]  ep_id: the endpoint ID
    \param[out] none
    \retval     USB device operation status
*/
uint8_t msc_data_handler (void *pudev, usb_dir_enum rx_tx, uint8_t ep_id)
{
    if ((USB_TX == rx_tx) && ((MSC_IN_EP & 0x7F) == ep_id)) {
        if (COMMAND_STAGE != g_bbb_transport_stage) {
            process_scsi_command(pudev); 
        }

        return USBD_OK;
    } else if ((USB_RX == rx_tx) && ((MSC_OUT_EP & 0x7F) == ep_id)) {
        if (COMMAND_STAGE == g_bbb_transport_stage) {
            scsi_command_parse();
            process_scsi_command(pudev);
        } else if (DATA_OUT_STAGE == g_bbb_transport_stage) {
            process_scsi_command(pudev); 
        }

        return USBD_OK;
    }

    return USBD_FAIL;
}
