/*!
    \file  usbh_msc_core.c
    \brief this file implements the MSC class driver functions

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

#include "usbh_msc_core.h"
#include "usbh_msc_scsi.h"
#include "usbh_msc_bot.h"
#include "usbh_core.h"
#include "usbh_ctrl.h"
#include "usbh_std.h"

#define USBH_MSC_ERROR_RETRY_LIMIT 10U

extern usbh_host_struct usb_host;
uint8_t msc_error_count = 0U;

msc_machine_struct msc_machine;

static usbh_status_enum usbh_msc_max_lun_get      (usb_core_handle_struct *pudev, usbh_host_struct *puhost);

static usbh_status_enum msc_req_state_polling_fun (usb_core_handle_struct *pudev, usbh_host_struct *puhost, void *pustate);
static usbh_status_enum msc_state_polling_fun     (usb_core_handle_struct *pudev, usbh_host_struct *puhost, void *pustate);

class_polling_fun_cb_struct class_polling_cb = 
{
    msc_req_state_polling_fun,
    msc_state_polling_fun,
};

void usbh_msc_error_handle(uint8_t status);

/*!
    \brief      interface initialization for MSC class.
    \param[in]  pudev: pointer to selected usb otg device
    \param[in]  puhost: pointer to usb host
    \param[out] none
    \retval     status of class request handled.
*/
usbh_status_enum usbh_msc_interface_init (usb_core_handle_struct *pudev, void *puhost)
{
    usbh_host_struct *pphost = puhost;

    if ((MSC_CLASS == pphost->device.itf_desc[0].bInterfaceClass) && \
            (MSC_PROTOCOL == pphost->device.itf_desc[0].bInterfaceProtocol)) {
        if (pphost->device.ep_desc[0][0].bEndpointAddress & 0x80U) {
            msc_machine.msc_bulk_in_ep = (pphost->device.ep_desc[0][0].bEndpointAddress);
            msc_machine.msc_bulk_in_ep_size  = pphost->device.ep_desc[0][0].wMaxPacketSize;
        } else {
            msc_machine.msc_bulk_out_ep = (pphost->device.ep_desc[0][0].bEndpointAddress);
            msc_machine.msc_bulk_out_ep_size  = pphost->device.ep_desc[0] [0].wMaxPacketSize;
        }

        if (pphost->device.ep_desc[0][1].bEndpointAddress & 0x80U) {
            msc_machine.msc_bulk_in_ep = (pphost->device.ep_desc[0][1].bEndpointAddress);
            msc_machine.msc_bulk_in_ep_size  = pphost->device.ep_desc[0][1].wMaxPacketSize;
        } else {
            msc_machine.msc_bulk_out_ep = (pphost->device.ep_desc[0][1].bEndpointAddress);
            msc_machine.msc_bulk_out_ep_size  = pphost->device.ep_desc[0][1].wMaxPacketSize;
        }

        msc_machine.hc_num_out = usbh_channel_alloc(pudev, 
                                                    msc_machine.msc_bulk_out_ep);

        msc_machine.hc_num_in = usbh_channel_alloc(pudev,
                                                   msc_machine.msc_bulk_in_ep);  

        /* open the new channels */
        usbh_channel_open (pudev,
                           msc_machine.hc_num_out,
                           pphost->device.address,
                           pphost->device.speed,
                           USB_EPTYPE_BULK,
                           msc_machine.msc_bulk_out_ep_size);  

        usbh_channel_open (pudev,
                           msc_machine.hc_num_in,
                           pphost->device.address,
                           pphost->device.speed,
                           USB_EPTYPE_BULK,
                           msc_machine.msc_bulk_in_ep_size);
    } else {
        pphost->usr_cb->device_not_supported(); 
    }

    return USBH_OK;
}

/*!
    \brief      de-initialize interface by freeing host channels allocated to interface
    \param[in]  pudev: pointer to selected usb otg device
    \param[in]  puhost: pointer to usb host
    \param[out] none
    \retval     none
*/
void usbh_msc_interface_deinit (usb_core_handle_struct *pudev, void *puhost)
{
    usb_host.usbh_backup_state.class_backup_state = 0;
    usb_host.usbh_backup_state.class_req_backup_state = 0;
    if (msc_machine.hc_num_out) {
        usb_hostchannel_halt(pudev, msc_machine.hc_num_out);
        usbh_channel_free (pudev, msc_machine.hc_num_out);
        msc_machine.hc_num_out = 0U;
    }

    if (msc_machine.hc_num_in) {
        usb_hostchannel_halt(pudev, msc_machine.hc_num_in);
        usbh_channel_free (pudev, msc_machine.hc_num_in);
        msc_machine.hc_num_in = 0U;
    }
}

/*!
    \brief      initialize the MSC state machine
    \param[in]  pudev: pointer to selected usb otg device
    \param[in]  puhost: pointer to usb host
    \param[out] none
    \retval     status of class request handled
*/
static usbh_status_enum msc_req_state_polling_fun (usb_core_handle_struct *pudev, usbh_host_struct *puhost, void *pustate)
{
    usbh_status_enum status = USBH_OK;
    usbh_msc_botxfer_param.msc_state = USBH_MSC_BOT_INIT_STATE;

    return status; 
}

/*!
    \brief      MSC state machine handler 
    \param[in]  pudev: pointer to selected usb otg device
    \param[in]  puhost: pointer to usb host
    \param[out] none
    \retval     host operation status
*/
static usbh_status_enum msc_state_polling_fun (usb_core_handle_struct *pudev, usbh_host_struct *puhost, void *pustate)
{
    usbh_host_struct *pphost = puhost;
    usbh_status_enum status = USBH_BUSY;
    uint8_t msc_status = USBH_MSC_BUSY;
    uint8_t appli_status = 0U;

    static uint8_t max_lun_exceed = FALSE;

    if (hcd_is_device_connected(pudev)) {
        switch (usbh_msc_botxfer_param.msc_state) {
            case USBH_MSC_BOT_INIT_STATE:
                usbh_msc_init(pudev);
                usbh_msc_botxfer_param.msc_state = USBH_MSC_BOT_RESET;
                break;

            case USBH_MSC_BOT_RESET:
                status = USBH_OK;
                if (USBH_OK == status) {
                    usbh_msc_botxfer_param.msc_state = USBH_MSC_GET_MAX_LUN;
                }

                break;

            case USBH_MSC_GET_MAX_LUN:
              
            
                if (CTRL_IDLE == puhost->usbh_backup_state.ctrl_backup_state) {
                    usbh_msc_max_lun_get(pudev, puhost);
                }
                status = ctrl_state_polling_fun(pudev, puhost, pustate);
                if (USBH_OK == status) {
                    msc_machine.maxLun = *(msc_machine.buff);

                    /* if device has more that one logical unit then it is not supported */
                    if ((msc_machine.maxLun > 0) && (FALSE == max_lun_exceed)) {
                        max_lun_exceed = TRUE;
                        pphost->usr_cb->device_not_supported();

                        break;
                    }

                    usbh_msc_botxfer_param.msc_state = USBH_MSC_TEST_UNIT_READY;
                }

                if (USBH_NOT_SUPPORTED == status) {
                    /* if the command has failed, then we need to move to next state, after
                       STALL condition is cleared by control-transfer */
                    usbh_msc_botxfer_param.msc_state_bkp = USBH_MSC_TEST_UNIT_READY; 

                    /* a clear feature should be issued here */
                    usbh_msc_botxfer_param.msc_state = USBH_MSC_CTRL_ERROR_STATE;
                }
                break;

            case USBH_MSC_CTRL_ERROR_STATE:
              
                if (CTRL_IDLE == puhost->usbh_backup_state.ctrl_backup_state) {
                    usbh_clear_feature(pudev,
                                         puhost,
                                         0x00U,
                                         pphost->control.hc_out_num);
                }
                status = ctrl_state_polling_fun(pudev, puhost, pustate);

                if (USBH_OK == status) {
                    /* if GetMaxLun request not support, assume single lun configuration */
                    msc_machine.maxLun = 0U;  

                    usbh_msc_botxfer_param.msc_state = usbh_msc_botxfer_param.msc_state_bkp;     
                }
                break;

            case USBH_MSC_TEST_UNIT_READY:

                /* issue SCSI command TestUnitReady */ 
                msc_status = usbh_msc_test_unit_ready(pudev);

                if (USBH_MSC_OK == msc_status) {
                    usbh_msc_botxfer_param.msc_state = USBH_MSC_READ_CAPACITY10;
                    msc_error_count = 0U;
                    status = USBH_OK;
                } else {
                    usbh_msc_error_handle(msc_status);
                }
                break;

            case USBH_MSC_READ_CAPACITY10:

                /* issue READ_CAPACITY10 SCSI command */
                msc_status = usbh_msc_read_capacity10(pudev);
                if (USBH_MSC_OK == msc_status) {
                    usbh_msc_botxfer_param.msc_state = USBH_MSC_MODE_SENSE6;
                    msc_error_count = 0U;
                    status = USBH_OK;
                } else {
                    usbh_msc_error_handle(msc_status);
                }
                break;

            case USBH_MSC_MODE_SENSE6:

                /* issue ModeSense6 SCSI command for detecting if device is write-protected */
                msc_status = usbh_msc_mode_sense6(pudev);

                if (USBH_MSC_OK == msc_status) {
                    usbh_msc_botxfer_param.msc_state = USBH_MSC_DEFAULT_APPLI_STATE;
                    msc_error_count = 0U;
                    status = USBH_OK;
                } else {
                    usbh_msc_error_handle(msc_status);
                }
                break;

            case USBH_MSC_REQUEST_SENSE:
                /* issue requestsense SCSI command for retreiving error code */
                msc_status = usbh_msc_request_sense(pudev);
                if (USBH_MSC_OK == msc_status) {
                    usbh_msc_botxfer_param.msc_state = usbh_msc_botxfer_param.msc_state_bkp;
                    status = USBH_OK;
                } else {
                    usbh_msc_error_handle(msc_status);
                }
                break;

            case USBH_MSC_BOT_USB_TRANSFERS:
                /* process the BOT state machine */
                usbh_msc_handle_botxfer(pudev, puhost, pustate);
                break;

            case USBH_MSC_DEFAULT_APPLI_STATE:
                /* process application callback for MSC */
                appli_status = pphost->usr_cb->user_application(pudev, 0U);
                if (0U == appli_status) {
                    usbh_msc_botxfer_param.msc_state = USBH_MSC_DEFAULT_APPLI_STATE;
                } else if (1U == appli_status) {
                    /* de-init requested from application layer */
                    status = USBH_APPLY_DEINIT;
                }
                break;

            case USBH_MSC_UNRECOVERED_STATE:
                status = USBH_UNRECOVERED_ERROR;
                break;

            default:
                break; 
        }
    }

    return status;
}

/*!
    \brief      get max lun of the mass storage device 
    \param[in]  pudev: pointer to selected usb otg device
    \param[in]  puhost: pointer to usb host
    \param[out] none
    \retval     USB control transfer status
*/
static usbh_status_enum usbh_msc_max_lun_get(usb_core_handle_struct *pudev , usbh_host_struct *puhost)
{
    usb_setup_union *p_setup = &(puhost->control.setup);

    p_setup->b.bmRequestType = USB_DIR_IN | USB_CLASS_REQ | USB_REQTYPE_INTERFACE;
    p_setup->b.bRequest = USB_REQ_GET_MAX_LUN;
    p_setup->b.wValue = 0U;
    p_setup->b.wIndex = 0U;
    p_setup->b.wLength = 1U;
  
    puhost->control.buff = msc_machine.buff;
    puhost->control.length = 1U;

    return USBH_OK;
}

/*!
    \brief      handling errors occuring during the MSC state machine
    \param[in]  status: error status
    \param[out] none
    \retval     none
*/
void usbh_msc_error_handle(uint8_t status)
{
    if (USBH_MSC_FAIL == status) {
        msc_error_count++;
        if (msc_error_count < USBH_MSC_ERROR_RETRY_LIMIT) {
            /* try MSC level error recovery, issue the request sense to get drive error reason  */
            usbh_msc_botxfer_param.msc_state = USBH_MSC_REQUEST_SENSE;
            usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
        } else {
            /* error trials exceeded the limit, go to unrecovered state */
            usbh_msc_botxfer_param.msc_state = USBH_MSC_UNRECOVERED_STATE;
        }
    } else if (USBH_MSC_PHASE_ERROR == status) {
        /* phase error, go to unrecoovered state */
        usbh_msc_botxfer_param.msc_state = USBH_MSC_UNRECOVERED_STATE;
    } else if (USBH_MSC_BUSY == status) {
        /* no change in state */
    }
}

/*!
    \brief      clear or disable a specific feature
    \param[in]  pudev: pointer to selected usb otg device
    \param[in]  puhost: pointer to usb host
    \param[in]  ep_addr: endpoint address 
    \param[in]  hc_num: host channel number 
    \param[out] none
    \retval     host operation status
*/
usbh_status_enum usbh_clear_feature(usb_core_handle_struct *pudev,
                                    usbh_host_struct *puhost,
                                    uint8_t ep_addr, 
                                    uint8_t hc_num)
{
    usb_setup_union *p_setup = &(puhost->control.setup);

    p_setup->b.bmRequestType = USB_DIR_OUT | USB_REQTYPE_ENDPOINT | USB_STANDARD_REQ;
    p_setup->b.bRequest = USBREQ_CLEAR_FEATURE;
    p_setup->b.wValue = FEATURE_SELECTOR_ENDPOINT;
    p_setup->b.wIndex = ep_addr;
    p_setup->b.wLength = 0U;

    if (USB_DIR_IN == (ep_addr & USB_DIR_MASK)) {
        /* endpoint type is IN */
        pudev->host.host_channel[hc_num].data_tg_in = 0U;
    } else {
        /* endpoint type is OUT */
        pudev->host.host_channel[hc_num].data_tg_out = 0U;
    }

    puhost->control.buff = 0U;
    puhost->control.length = 0U;

    return USBH_OK;
}
