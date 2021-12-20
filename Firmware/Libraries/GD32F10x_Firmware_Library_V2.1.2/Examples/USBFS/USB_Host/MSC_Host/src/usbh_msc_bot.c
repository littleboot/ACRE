/*!
    \file  usbh_msc_bot.c 
    \brief this file includes the mass storage related functions

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
#include "usbh_ctrl.h"
#include "usbh_std.h"

host_cbwpkt_union usbh_msc_cbw_data;
host_cswpkt_union usbh_msc_csw_data;

/* keeps count of STALL error cases*/
static uint32_t bot_stall_error_count;

usbh_botxfer_struct usbh_msc_botxfer_param;
  
/*!
    \brief      initialize the mass storage parameters
    \param[in]  pudev: pointer to selected usb otg device
    \param[out] none
    \retval     none
*/
void usbh_msc_init (usb_core_handle_struct *pudev)
{
    if (hcd_is_device_connected(pudev)) {
        usbh_msc_cbw_data.field.cbw_signature = USBH_MSC_BOT_CBW_SIGNATURE;
        usbh_msc_cbw_data.field.cbw_tag = USBH_MSC_BOT_CBW_TAG;
        usbh_msc_cbw_data.field.cbw_lun = 0U;  /* only one LUN is supported */
        usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;  
    }

    bot_stall_error_count = 0U;
    msc_error_count = 0U;
}

/*!
    \brief      manage the different states of BOT transfer and updates the status to upper layer
    \param[in]  pudev: pointer to selected usb otg device
    \param[in]  puhost: pointer to usb host
    \param[in]  pustate: pointer to usb state
    \param[out] none
    \retval     none
*/
void usbh_msc_handle_botxfer (usb_core_handle_struct *pudev ,usbh_host_struct *puhost , usbh_state_handle_struct* pustate)
{
    uint8_t xfer_direction, index;
    static uint32_t remaining_data_length;
    static uint8_t *datapointer, *datapointer_prev;
    static uint8_t error_direction;
    usbh_status_enum status;

    urb_state_enum urb_status = URB_IDLE;

    if (hcd_is_device_connected(pudev)) {
        switch (usbh_msc_botxfer_param.bot_state) {
            case USBH_MSC_SEND_CBW:
                /* send CBW */
                usbh_xfer (pudev,
                           &usbh_msc_cbw_data.cbw_array[0], 
                           msc_machine.hc_num_out, 
                           USBH_MSC_BOT_CBW_PACKET_LENGTH);

                usbh_msc_botxfer_param.bot_state_bkp = USBH_MSC_SEND_CBW;
                usbh_msc_botxfer_param.bot_state = USBH_MSC_SENT_CBW;
                break;

            case USBH_MSC_SENT_CBW:
                urb_status = hcd_urb_state_get(pudev, msc_machine.hc_num_out);
                if (URB_DONE == urb_status) {
                    bot_stall_error_count = 0U;
                    usbh_msc_botxfer_param.bot_state_bkp = USBH_MSC_SENT_CBW; 

                    /* if the CBW packet is sent successful, then change the state */
                    xfer_direction = (usbh_msc_cbw_data.field.cbw_flags & USB_DIR_MASK);

                    if (0U != usbh_msc_cbw_data.field.cbw_transfer_length) {
                        remaining_data_length = usbh_msc_cbw_data.field.cbw_transfer_length;
                        datapointer = usbh_msc_botxfer_param.p_rx_tx_buff;
                        datapointer_prev = datapointer;

                        /* if there is data transfer stage */
                        if (USB_DIR_IN == xfer_direction) {
                            /* data direction is IN */
                            usbh_msc_botxfer_param.bot_state = USBH_MSC_BOT_DATAIN_STATE;
                        } else {
                            /* data direction is OUT */
                            usbh_msc_botxfer_param.bot_state = USBH_MSC_BOT_DATAOUT_STATE;
                        }
                    } else {
                        /* if there is no data transfer stage */
                        usbh_msc_botxfer_param.bot_state = USBH_MSC_RECEIVE_CSW_STATE;
                    }
                } else if (URB_NOTREADY == urb_status) {
                    usbh_msc_botxfer_param.bot_state  = usbh_msc_botxfer_param.bot_state_bkp;    
                } else if (URB_STALL == urb_status) {
                    error_direction = USBH_MSC_DIR_OUT;
                    usbh_msc_botxfer_param.bot_state = USBH_MSC_BOT_ERROR_OUT;
                }
                break;

            case USBH_MSC_BOT_DATAIN_STATE:
                urb_status = hcd_urb_state_get(pudev, msc_machine.hc_num_in);

                /* bot data in stage */
                if ((URB_DONE == urb_status) ||(USBH_MSC_BOT_DATAIN_STATE != usbh_msc_botxfer_param.bot_state_bkp)) {
                    bot_stall_error_count = 0U;
                    usbh_msc_botxfer_param.bot_state_bkp = USBH_MSC_BOT_DATAIN_STATE;    

                    if (remaining_data_length > msc_machine.msc_bulk_in_ep_size) {
                        usbh_xfer(pudev,
                                  datapointer, 
                                  msc_machine.hc_num_in, 
                                  msc_machine.msc_bulk_in_ep_size);

                        remaining_data_length -= msc_machine.msc_bulk_in_ep_size;
                        datapointer = datapointer + msc_machine.msc_bulk_in_ep_size;
                    } else if (0U == remaining_data_length) {
                        /* if value was 0, and successful transfer, then change the state */
                        usbh_msc_botxfer_param.bot_state = USBH_MSC_RECEIVE_CSW_STATE;
                    } else {
                        usbh_xfer(pudev,
                                  datapointer, 
                                  msc_machine.hc_num_in, 
                                  remaining_data_length);

                        remaining_data_length = 0U; /* reset this value and keep in same state */
                    }
                } else if (URB_STALL == urb_status) {
                    /* this is data stage stall condition */

                    error_direction = USBH_MSC_DIR_IN;
                    usbh_msc_botxfer_param.bot_state  = USBH_MSC_BOT_ERROR_IN;

                    /* refer to usb mass-storage class : bot (www.usb.org) 
                    6.7.2 host expects to receive data from the device
                    3. on a stall condition receiving data, then:
                    the host shall accept the data received.
                    the host shall clear the bulk-in pipe.
                    4. the host shall attempt to receive a csw.

                    usbh_msc_botxferparam.botstatebkp is used to switch to the original 
                    state after the clearfeature command is issued.
                    */
                    usbh_msc_botxfer_param.bot_state_bkp = USBH_MSC_RECEIVE_CSW_STATE;
                }
                break;

            case USBH_MSC_BOT_DATAOUT_STATE:
                /* bot data out stage */
                urb_status = hcd_urb_state_get(pudev, msc_machine.hc_num_out);
                if (URB_DONE == urb_status) {
                    bot_stall_error_count = 0U;
                    usbh_msc_botxfer_param.bot_state_bkp = USBH_MSC_BOT_DATAOUT_STATE;

                    if (remaining_data_length > msc_machine.msc_bulk_out_ep_size) {
                        usbh_xfer (pudev,
                                   datapointer, 
                                   msc_machine.hc_num_out, 
                                   msc_machine.msc_bulk_out_ep_size);

                        datapointer_prev = datapointer;
                        datapointer = datapointer + msc_machine.msc_bulk_out_ep_size;

                        remaining_data_length = remaining_data_length - msc_machine.msc_bulk_out_ep_size;
                    } else if (0U == remaining_data_length) {
                        /* if value was 0, and successful transfer, then change the state */
                        usbh_msc_botxfer_param.bot_state = USBH_MSC_RECEIVE_CSW_STATE;
                    } else {
                        usbh_xfer (pudev,
                                   datapointer, 
                                   msc_machine.hc_num_out, 
                                   remaining_data_length);

                        remaining_data_length = 0U; /* reset this value and keep in same state */   

                        datapointer_prev = datapointer;
                        datapointer = datapointer + msc_machine.msc_bulk_out_ep_size;
                    }
                } else if(URB_NOTREADY == urb_status) {
                    if (datapointer != datapointer_prev) {
                        usbh_xfer (pudev,
                                   (datapointer - msc_machine.msc_bulk_out_ep_size), 
                                   msc_machine.hc_num_out, 
                                   msc_machine.msc_bulk_out_ep_size);
                    } else {
                        usbh_xfer (pudev,
                                   datapointer,
                                   msc_machine.hc_num_out, 
                                   msc_machine.msc_bulk_out_ep_size);
                    }
                } else if (URB_STALL == urb_status) {
                    error_direction = USBH_MSC_DIR_OUT;
                    usbh_msc_botxfer_param.bot_state  = USBH_MSC_BOT_ERROR_OUT;

                    /* refer to usb mass-storage class : bot (www.usb.org) 
                    6.7.3 ho - host expects to send data to the device
                    3. on a stall condition sending data, then:
                    " the host shall clear the bulk-out pipe.
                    4. the host shall attempt to receive a csw.

                    the above statement will do the clear the bulk-out pipe.
                    the below statement will help in getting the csw.  

                    usbh_msc_botxferparam.botstatebkp is used to switch to the original 
                    state after the clearfeature command is issued.
                    */

                    usbh_msc_botxfer_param.bot_state_bkp = USBH_MSC_RECEIVE_CSW_STATE;
                }
                break;

            case USBH_MSC_RECEIVE_CSW_STATE:
                /* bot csw stage */     
                /* note: we cannot reset the botstallerrorcount here as it may come from the clearfeature from previous command */
                usbh_msc_botxfer_param.bot_state_bkp = USBH_MSC_RECEIVE_CSW_STATE;
                usbh_msc_botxfer_param.p_rx_tx_buff = usbh_msc_csw_data.csw_array;
                usbh_msc_botxfer_param.data_length = USBH_MSC_CSW_MAX_LENGTH;

                for (index = USBH_MSC_CSW_LENGTH; index != 0U; index--) {
                    usbh_msc_csw_data.csw_array[index] = 0U;
                }

                usbh_msc_csw_data.csw_array[0] = 0U;

                usbh_xfer (pudev,
                           usbh_msc_botxfer_param.p_rx_tx_buff, 
                           msc_machine.hc_num_in, 
                           USBH_MSC_CSW_MAX_LENGTH);

                usbh_msc_botxfer_param.bot_state = USBH_MSC_DECODE_CSW;
                break;

            case USBH_MSC_DECODE_CSW:
                urb_status = hcd_urb_state_get(pudev, msc_machine.hc_num_in);

                /* decode csw */
                if (URB_DONE == urb_status) {
                    bot_stall_error_count = 0U;
                    usbh_msc_botxfer_param.bot_state_bkp = USBH_MSC_RECEIVE_CSW_STATE;
                    usbh_msc_botxfer_param.msc_state = usbh_msc_botxfer_param.msc_state_current ;
                    usbh_msc_botxfer_param.bot_xfer_status = usbh_msc_decode_csw(pudev , puhost);
                } else if(URB_STALL == urb_status) {
                    error_direction = USBH_MSC_DIR_IN;
                    usbh_msc_botxfer_param.bot_state  = USBH_MSC_BOT_ERROR_IN;
                }
                break;

            case USBH_MSC_BOT_ERROR_IN: 
                status = usbh_msc_bot_abort(pudev, puhost,pustate, USBH_MSC_DIR_IN);
                if (USBH_OK == status) {
                    /* check if the error was due in both the directions */
                    if (USBH_MSC_BOTH_DIR == error_direction) {
                        /* if both directions are needed, switch to out direction */
                        usbh_msc_botxfer_param.bot_state = USBH_MSC_BOT_ERROR_OUT;
                    } else {
                        /* switch back to the original state, in many cases this will be USBH_MSC_RECEIVE_CSW_STATE state */
                        usbh_msc_botxfer_param.bot_state = usbh_msc_botxfer_param.bot_state_bkp;
                    }
                } else if (USBH_UNRECOVERED_ERROR == status) {
                    /* this means that there is a stall error limit, do reset recovery */
                    usbh_msc_botxfer_param.bot_xfer_status = USBH_MSC_PHASE_ERROR;
                }
                break;

            case USBH_MSC_BOT_ERROR_OUT: 
                status = usbh_msc_bot_abort(pudev, puhost,pustate, USBH_MSC_DIR_OUT);
                if (USBH_OK == status) {
                    /* switch back to the original state */
                    usbh_msc_botxfer_param.bot_state = usbh_msc_botxfer_param.bot_state_bkp;        
                } else if (USBH_UNRECOVERED_ERROR == status) {
                    /* this means that there is a stall error limit, do reset recovery */
                    usbh_msc_botxfer_param.bot_xfer_status = USBH_MSC_PHASE_ERROR;
                }
                break;

            default:
                break;
        }
    }
}

/*!
    \brief      manages the different Error handling for STALL
    \param[in]  pudev: pointer to selected usb otg device
    \param[in]  puhost: pointer to usb host
    \param[in]  pustate: pointer to usb state
    \param[in]  direction: USB derection
    \param[out] none
    \retval     usbh status
*/
usbh_status_enum usbh_msc_bot_abort(usb_core_handle_struct *pudev, 
                                    usbh_host_struct *puhost,
                                    usbh_state_handle_struct* pustate,
                                    uint8_t direction)
{
    usbh_status_enum status = USBH_BUSY;

    switch (direction) {
        case USBH_MSC_DIR_IN :
            if (CTRL_IDLE == puhost->usbh_backup_state.ctrl_backup_state) {
                usbh_clear_feature(pudev,
                                puhost,
                                msc_machine.msc_bulk_in_ep,
                                msc_machine.hc_num_in);
            }
            status = ctrl_state_polling_fun(pudev, puhost, pustate);
            break;

        case USBH_MSC_DIR_OUT :
          
            if (CTRL_IDLE == puhost->usbh_backup_state.ctrl_backup_state) {
                usbh_clear_feature(pudev,
                                puhost,
                                msc_machine.msc_bulk_out_ep,
                                msc_machine.hc_num_out);
            }
            status = ctrl_state_polling_fun(pudev, puhost, pustate);

            break;

        default:
            break;
    }

    bot_stall_error_count++; /* check continous number of times, stall has occured */ 
    if (bot_stall_error_count > MAX_BULK_STALL_COUNT_LIMIT ) {
        status = USBH_UNRECOVERED_ERROR;
    }

    return status;
}

/*!
    \brief      decode the CSW received by the device and updates the same to upper layer.
    \param[in]  pudev: pointer to selected usb otg device
    \param[in]  puhost: pointer to usb host
    \param[out] none
    \retval     on success USBH_MSC_OK, on failure USBH_MSC_FAIL
*/
uint8_t usbh_msc_decode_csw(usb_core_handle_struct *pudev, usbh_host_struct *puhost)
{
    uint8_t status;
    uint32_t data_xfer_count = 0U;
    status = USBH_MSC_FAIL;

    if (hcd_is_device_connected(pudev)) {
        /*checking if the transfer length is diffrent than 13*/
        data_xfer_count = hcd_xfer_count_get(pudev, msc_machine.hc_num_in); 

        if (USBH_MSC_CSW_LENGTH != data_xfer_count) {
            /*(4) Hi > Dn (host expects to receive data from the device,
            device intends to transfer no data)
            (5) Hi > Di (host expects to receive data from the device,
            device intends to send data to the host)
            (9) Ho > Dn (host expects to send data to the device,
            device intends to transfer no data)
            (11) Ho > Do  (host expects to send data to the device,
            device intends to receive data from the host)*/

            status = USBH_MSC_PHASE_ERROR;
        } else {
            /* CSW length is correct */

            /* check validity of the CSW signature and CSWStatus */
            if (USBH_MSC_BOT_CSW_SIGNATURE == usbh_msc_csw_data.field.csw_signature) {
                /* check condition 1. dCSWSignature is equal to 53425355h */
                if (usbh_msc_csw_data.field.csw_tag == usbh_msc_cbw_data.field.cbw_tag) {
                    /* check condition 3. dCSWTag matches the dCBWTag from the corresponding CBW */
                    if (USBH_MSC_OK == usbh_msc_csw_data.field.csw_status) {
                        /* refer to USB Mass-Storage Class : BOT (www.usb.org) 
                        Hn host expects no data transfers
                        Hi host expects to receive data from the device
                        Ho host expects to send data to the device

                        Dn device intends to transfer no data
                        Di device intends to send data to the host
                        Do device intends to receive data from the host

                        section 6.7 
                        (1) Hn = Dn (Host expects no data transfers,
                        device intends to transfer no data)
                        (6) Hi = Di (Host expects to receive data from the device,
                        device intends to send data to the host)
                        (12) Ho = Do (Host expects to send data to the device, 
                        device intends to receive data from the host)
                        */

                        status = USBH_MSC_OK;
                    } else if (USBH_MSC_FAIL == usbh_msc_csw_data.field.csw_status) {
                        status = USBH_MSC_FAIL;
                    } else if (USBH_MSC_PHASE_ERROR == usbh_msc_csw_data.field.csw_status) {
                        /* refer to USB Mass-Storage Class : BOT (www.usb.org) 
                        section 6.7 
                        (2) Hn < Di ( Host expects no data transfers, 
                        device intends to send data to the host)
                        (3) Hn < Do ( Host expects no data transfers, 
                        device intends to receive data from the host)
                        (7) Hi < Di ( Host expects to receive data from the device, 
                        device intends to send data to the host)
                        (8) Hi <> Do ( Host expects to receive data from the device, 
                        device intends to receive data from the host)
                        (10) Ho <> Di (Host expects to send data to the device,
                        Di device intends to send data to the host)
                        (13) Ho < Do (Host expects to send data to the device, 
                        device intends to receive data from the host)
                        */

                        status = USBH_MSC_PHASE_ERROR;
                    }
                } /* CSW tag matching is checked  */
            /* CSW signature correct checking */
            } else {
                /* if the CSW signature is not valid, we sall return the phase error to
                    upper layers for reset recovery */

                status = USBH_MSC_PHASE_ERROR;
            }
        } /* CSW length check */
    }

    usbh_msc_botxfer_param.bot_xfer_status = status;

    return status;
}

