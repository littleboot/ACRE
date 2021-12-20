/*!
    \file  usbh_hid_core.c 
    \brief this file implements the functions for the HID core state process

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

#include "usbh_hid_core.h"
#include "usbh_hid_mouse.h"
#include "usbh_hid_keybd.h"
#include "stdio.h"
#include "usbh_ctrl.h"
#include "usbh_std.h"

hid_machine_struct hid_machine;
usb_setup_union hid_setup;
usbh_hid_desc_struct hid_desc;

uint8_t start_toggle = 0U;

extern usbh_state_handle_struct usbh_state_core;
extern usbh_host_struct usb_host;

class_polling_fun_cb_struct class_polling_cb = 
{
    hid_req_state_polling_fun,
    hid_state_polling_fun,
};

static void hid_req_set_idle        (usb_core_handle_struct *pudev,
                                     usbh_host_struct *phost,
                                     uint8_t duration,
                                     uint8_t report_id);
static void hid_req_set_protocol    (usb_core_handle_struct *pudev,
                                     usbh_host_struct *phost,
                                     uint8_t protocol);
static void hid_clear_feature       (usbh_host_struct *puhost, uint16_t index);
static void usbh_hid_desc_parse     (usbh_hid_desc_struct *phid_desc, uint8_t *buf);

/*!
    \brief      the polling function of HID REQ state
    \param[in]  pudev: pointer to usb device
    \param[in]  puhost: pointer to usb host
    \param[in]  pustate: pointer to usb state driver
    \param[out] none
    \retval     none
*/
usbh_status_enum hid_req_state_polling_fun (usb_core_handle_struct *pudev, usbh_host_struct *puhost, void *pustate)
{
    usbh_status_enum exe_state = USBH_BUSY;
    backup_state_struct *pubs;
    pubs = &puhost->usbh_backup_state;
  
    if (hcd_is_device_connected(pudev)) {
        switch (pubs->class_req_backup_state) {
          case HID_REQ_IDLE:
              pubs->class_req_backup_state = HID_REQ_GET_HID_DESC;
              break;
          
          case HID_REQ_GET_HID_DESC:
              if (CTRL_IDLE == puhost->usbh_backup_state.ctrl_backup_state) {
                  usbh_enum_desc_get (pudev, 
                                      puhost, 
                                      pudev->host.rx_buffer, 
                                      USB_REQTYPE_INTERFACE | USB_STANDARD_REQ, 
                                      USB_HIDDESC, 
                                      USB_HID_DESC_SIZE);
              }
              
              /*  to determine whether a control transfer is complete */
              if (USBH_OK == ctrl_state_polling_fun(pudev, puhost, pustate)) {
                  /* parse the hid descripter */
                  usbh_hid_desc_parse(&hid_desc, pudev->host.rx_buffer);
                  pubs->class_req_backup_state = HID_REQ_GET_REPORT_DESC;
              }
            break;
              
          case HID_REQ_GET_REPORT_DESC:

              if (CTRL_IDLE == puhost->usbh_backup_state.ctrl_backup_state) {
                  usbh_enum_desc_get(pudev, 
                                     puhost, 
                                     pudev->host.rx_buffer, 
                                     USB_REQTYPE_INTERFACE | USB_STANDARD_REQ, 
                                     USB_HIDREPDESC, 
                                     hid_desc.wItemLength);
              }
              /*  to determine whether a control transfer is complete */
              if (USBH_OK == ctrl_state_polling_fun(pudev, puhost, pustate)) {
                  pubs->class_req_backup_state = HID_REQ_SET_IDLE;
              }
            break;
              
          case HID_REQ_SET_IDLE:

              if (CTRL_IDLE == puhost->usbh_backup_state.ctrl_backup_state) {
                  hid_req_set_idle(pudev, puhost, 0U, 0U);
              }

              /*  to determine whether a control transfer is complete */
              if (USBH_OK == ctrl_state_polling_fun(pudev, puhost, pustate)) {
                  pubs->class_req_backup_state = HID_REQ_SET_PROTOCOL;
              }
            break;
              
          case HID_REQ_SET_PROTOCOL:
              if(CTRL_IDLE == puhost->usbh_backup_state.ctrl_backup_state) {
                  hid_req_set_protocol(pudev, puhost, 0U);
              }

              /*  to determine whether a control transfer is complete */
              if (USBH_OK == ctrl_state_polling_fun(pudev, puhost, pustate)) {
                  exe_state = USBH_OK;
              }
            break;

          default:
            break;
        }
    }
    return exe_state;
}

/*!
    \brief      the polling function of HID state
    \param[in]  pudev: pointer to usb device
    \param[in]  puhost: pointer to usb host
    \param[in]  pustate: pointer to usb state driver
    \param[out] none
    \retval     none
*/
usbh_status_enum hid_state_polling_fun (usb_core_handle_struct *pudev, usbh_host_struct *puhost, void *pustate)
{
    backup_state_struct *pubs;
    uint32_t frame_count;
    pubs = &puhost->usbh_backup_state;

    if (hcd_is_device_connected(pudev)) {
        switch (pubs->class_backup_state) {
        case HID_IDLE:
            hid_machine.cb->init();
            pubs->class_backup_state = HID_SYNC;
            break;

        case HID_SYNC:
            if (1 == USB_EVEN_FRAME()) {
                pubs->class_backup_state = HID_GET_DATA;
            }
            break;

        case HID_GET_DATA:
            usbh_xfer (pudev, 
                       hid_machine.buf,
                       hid_machine.hc_in_num,
                       hid_machine.length);

            start_toggle = 1U;
            hid_machine.timer = (uint16_t)USB_CURRENT_FRAME_GET();
            pubs->class_backup_state = HID_POLL;
            break;

        case HID_POLL:
            frame_count = (uint16_t)USB_CURRENT_FRAME_GET();

            if((frame_count > hid_machine.timer) && 
                ((frame_count - hid_machine.timer) >= hid_machine.poll)) {
                pubs->class_backup_state = HID_GET_DATA;
            } else if ((frame_count < hid_machine.timer) && 
                        ((frame_count + 0x3FFFU - hid_machine.timer) >= hid_machine.poll)) {
                pubs->class_backup_state = HID_GET_DATA;
            } else if (URB_DONE == hcd_urb_state_get(pudev, hid_machine.hc_in_num)) {
                /* handle data once */
                if (1U == start_toggle) {
                    start_toggle = 0U;
                    hid_machine.cb->decode(hid_machine.buf);
                }
                /* IN endpoint stalled */
            } else if (URB_STALL == hcd_urb_state_get(pudev, hid_machine.hc_in_num)) {
                /* issue clear feature on interrupt IN endpoint */ 
                if (CTRL_IDLE == puhost->usbh_backup_state.ctrl_backup_state) {
                    hid_clear_feature(puhost, (uint16_t)hid_machine.ep_addr);
                }

                if (USBH_OK == ctrl_state_polling_fun(pudev, puhost, pustate)) {
                    pubs->class_backup_state = HID_GET_DATA;
                }
            } else if (URB_ERROR == hcd_urb_state_get(pudev, hid_machine.hc_in_num)) {
                pudev->host.host_channel[hid_machine.hc_in_num].data_tg_in ^= 1U;
            } else {
                  /* no operation */
            }
            break;
 
        default:
             break;
        }
    }
    return USBH_OK;
}


/*!
    \brief      clear feature in hid stage
    \param[in]  puhost: pointer to USB host
    \param[in]  index: the index
    \param[out] none
    \retval     none
*/
static void hid_clear_feature (usbh_host_struct *puhost, uint16_t index)
{
    usb_setup_union *p_setup = &(puhost->control.setup);

    p_setup->b.bmRequestType = USB_DIR_OUT | USB_REQTYPE_ENDPOINT | USB_STANDARD_REQ;
    p_setup->b.bRequest = USBREQ_CLEAR_FEATURE;
    p_setup->b.wValue = FEATURE_SELECTOR_ENDPOINT;
    p_setup->b.wIndex = index;
    p_setup->b.wLength = 0U;
  
    puhost->control.buff = 0U;
    puhost->control.length = 0U;
}

/*!
    \brief      parse the HID descriptor
    \param[in]  hid_desc: pointer to HID descriptor
    \param[in]  buf: pointer to buffer where the source descriptor is available
    \param[out] none
    \retval     none
*/
static void usbh_hid_desc_parse (usbh_hid_desc_struct *phid_desc, uint8_t *buf)
{
    phid_desc->bLength = *(uint8_t *)(buf + 0U);
    phid_desc->bDescriptorType = *(uint8_t *)(buf + 1U);
    phid_desc->bcdHID = SWAPBYTE(buf + 2U);
    phid_desc->bCountryCode = *(uint8_t *)(buf + 4U);
    phid_desc->bNumDescriptors = *(uint8_t *)(buf + 5U);
    phid_desc->bReportDescriptorType = *(uint8_t *)(buf + 6U);
    phid_desc->wItemLength = SWAPBYTE(buf + 7U);
}

/*!
    \brief      initialize the HID class
    \param[in]  pudev: pointer to usb device
    \param[in]  phost: pointer to usb host
    \param[out] none
    \retval     none
*/
usbh_status_enum usbh_hid_interface_init (usb_core_handle_struct *pudev, void *phost)
{
    uint8_t max_ep = 0U;
    uint8_t ep_addr = 0U;
    usbh_host_struct *pphost = phost;

    uint8_t num = 0U;
    usbh_status_enum status = USBH_BUSY;
    hid_machine.state = HID_ERROR;

    if (HID_BOOT_CODE == pphost->device.itf_desc[0].bInterfaceSubClass) {
        /*decode bootclass protocl: mouse or keyboard*/
        if (HID_KEYBRD_BOOT_CODE == pphost->device.itf_desc[0].bInterfaceProtocol) {
            hid_machine.cb = &HID_KEYBRD_cb;
        } else if (HID_MOUSE_BOOT_CODE == pphost->device.itf_desc[0].bInterfaceProtocol) {
            hid_machine.cb = &hid_mouse_cb;
        } else {
            /* no operation */
        }

        hid_machine.state = HID_IDLE;
        hid_machine.ctl_state = HID_REQ_IDLE;
        hid_machine.ep_addr = pphost->device.ep_desc[0][0].bEndpointAddress;
        hid_machine.length = pphost->device.ep_desc[0][0].wMaxPacketSize;
        hid_machine.poll = pphost->device.ep_desc[0][0].bInterval;

        if (hid_machine.poll < HID_MIN_POLL) {
            hid_machine.poll = HID_MIN_POLL;
        }

        /* check fo available number of endpoints */
        /* find the number of eps in the interface descriptor */
        /* choose the lower number in order not to overrun the buffer allocated */
        max_ep = ((pphost->device.itf_desc[0].bNumEndpoints <= USBH_MAX_EP_NUM) ? 
                pphost->device.itf_desc[0].bNumEndpoints : USBH_MAX_EP_NUM);

        /* decode endpoint IN and OUT address from interface descriptor */
        for (num = 0U; num < max_ep; num++) {
            ep_addr = pphost->device.ep_desc[0][num].bEndpointAddress;

            if (ep_addr & 0x80U) {
                hid_machine.hid_int_in_ep = ep_addr;
                hid_machine.hc_in_num = usbh_channel_alloc(pudev, ep_addr);

                /* open channel for IN endpoint */
                usbh_channel_open (pudev,
                                   hid_machine.hc_in_num,
                                   pphost->device.address,
                                   pphost->device.speed,
                                   USB_EPTYPE_INTR,
                                   hid_machine.length); 
            } else {
                hid_machine.hid_int_out_ep = ep_addr;
                hid_machine.hc_out_num  = usbh_channel_alloc(pudev, ep_addr);

                /* open channel for OUT endpoint */
                usbh_channel_open (pudev,
                                   hid_machine.hc_out_num,
                                   pphost->device.address,
                                   pphost->device.speed,
                                   USB_EPTYPE_INTR,
                                   hid_machine.length);
            }
        }

        start_toggle = 0U;
        status = USBH_OK; 
    } else {
        pphost->usr_cb->device_not_supported();
    }
 
    return status;
}

/*!
    \brief      de-initialize the host channels used for the HID class
    \param[in]  pudev: pointer to usb device
    \param[in]  phost: pointer to usb host
    \param[out] none
    \retval     none
*/
void usbh_hid_interface_deinit (usb_core_handle_struct *pudev, void *phost)
{
    usb_host.usbh_backup_state.class_backup_state = 0U;
    usb_host.usbh_backup_state.class_req_backup_state = 0U;
    if (0x00U != hid_machine.hc_in_num) {
        usb_hostchannel_halt (pudev, hid_machine.hc_in_num);
        usbh_channel_free (pudev, hid_machine.hc_in_num);
        hid_machine.hc_in_num = 0U;     /* reset the channel as free */
    }

    if (0x00U != hid_machine.hc_out_num) {
        usb_hostchannel_halt (pudev, hid_machine.hc_out_num);
        usbh_channel_free (pudev, hid_machine.hc_out_num);
        hid_machine.hc_out_num = 0U;    /* reset the channel as free */
    }

    start_toggle = 0U;
}

/*!
    \brief      set idle in hid req stage
    \param[in]  pudev: pointer to USB device
    \param[in]  puhost: pointer to USB host
    \param[in]  duration: duration for HID Idle request
    \param[in]  report_id: targetted report ID for Set Idle request
    \param[out] none
    \retval     none
*/
static void hid_req_set_idle(usb_core_handle_struct *pudev,
                             usbh_host_struct *phost,
                             uint8_t duration,
                             uint8_t report_id)
{
    usb_setup_union *setup = &(phost->control.setup);

    setup->b.bmRequestType = USB_DIR_OUT | USB_REQTYPE_INTERFACE | USB_CLASS_REQ;
    setup->b.bRequest = USBH_HID_SET_IDLE;
    setup->b.wValue = (uint16_t)((uint16_t)duration << 8U) | (uint16_t)report_id;
    setup->b.wIndex = 0U;
    setup->b.wLength = 0U;
}

/*!
    \brief      set protocol in hid req stage
    \param[in]  pudev: pointer to USB device
    \param[in]  puhost: pointer to USB host
    \param[in]  protocol: boot/report protocol
    \param[out] none
    \retval     none
*/
static void hid_req_set_protocol(usb_core_handle_struct *pudev,
                                 usbh_host_struct *phost,
                                 uint8_t protocol)
{
    usb_setup_union *setup = &(phost->control.setup);

    setup->b.bmRequestType = USB_DIR_OUT | USB_REQTYPE_INTERFACE | USB_CLASS_REQ;
    setup->b.bRequest = USBH_HID_SET_PROTOCOL;

    if (0U != protocol) {
        /* boot protocol */
        setup->b.wValue = 0U;
    } else {
        /*report protocol*/
        setup->b.wValue = 1U;
    }

    setup->b.wIndex = 0U;
    setup->b.wLength = 0U;
}
