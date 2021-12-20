/*!
    \file  usbh_msc_scsi.c 
    \brief this file implements the SCSI commands

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
#include "usb_std.h"

msc_param_struct usbh_msc_param;

uint8_t usbh_data_in_buffer[512];
uint8_t usbh_data_out_buffer[512];

/*!
    \brief      send'test unit ready' command to the device
    \param[in]  pudev: pointer to usb device
    \param[out] none
    \retval     host operation status
*/
uint8_t usbh_msc_test_unit_ready (usb_core_handle_struct *pudev)
{
    uint8_t index;
    usbh_msc_status_enum status = USBH_MSC_BUSY;

    if (hcd_is_device_connected(pudev)) {
        switch (usbh_msc_botxfer_param.cmd_state_machine) {
            case CMD_SEND_STATE:  
                /* prepare the CBW and relevent field */
                usbh_msc_cbw_data.field.cbw_transfer_length = 0U;       /* no data transfer */
                usbh_msc_cbw_data.field.cbw_flags = USB_DIR_OUT;
                usbh_msc_cbw_data.field.cbw_length = CBW_LENGTH_TEST_UNIT_READY;
                usbh_msc_botxfer_param.p_rx_tx_buff = usbh_msc_csw_data.csw_array;
                usbh_msc_botxfer_param.data_length = USBH_MSC_CSW_MAX_LENGTH;
                usbh_msc_botxfer_param.msc_state_current = USBH_MSC_TEST_UNIT_READY;

                for (index = CBW_CB_LENGTH; index != 0U; index--) {
                    usbh_msc_cbw_data.field.cbwcb[index] = 0x00U;
                }

                usbh_msc_cbw_data.field.cbwcb[0]  = OPCODE_TEST_UNIT_READY; 
                usbh_msc_botxfer_param.bot_state = USBH_MSC_SEND_CBW;

                /* start the transfer, then let the state machine magage the other transactions */
                usbh_msc_botxfer_param.msc_state = USBH_MSC_BOT_USB_TRANSFERS;
                usbh_msc_botxfer_param.bot_xfer_status = USBH_MSC_BUSY;
                usbh_msc_botxfer_param.cmd_state_machine = CMD_WAIT_STATUS;

                status = USBH_MSC_BUSY; 
                break;

            case CMD_WAIT_STATUS: 
                if (USBH_MSC_OK == usbh_msc_botxfer_param.bot_xfer_status) {
                    /* commands successfully sent and response received */
                    usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
                    status = USBH_MSC_OK;
                } else if (USBH_MSC_FAIL == usbh_msc_botxfer_param.bot_xfer_status) {
                    /* failure mode */
                    usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
                    status = USBH_MSC_FAIL;
                } else if (USBH_MSC_PHASE_ERROR == usbh_msc_botxfer_param.bot_xfer_status) {
                    /* failure mode */
                    usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
                    status = USBH_MSC_PHASE_ERROR;    
                }
                break;

        default:
                break;
        }
    }

    return status;
}

/*!
    \brief      send the read capacity command to the device
    \param[in]  pudev: pointer to usb device
    \param[out] none
    \retval     host operation status
*/
uint8_t usbh_msc_read_capacity10(usb_core_handle_struct *pudev)
{
    uint8_t index;
    usbh_msc_status_enum status = USBH_MSC_BUSY;

    if (hcd_is_device_connected(pudev)) {
        switch (usbh_msc_botxfer_param.cmd_state_machine) {
            case CMD_SEND_STATE:
                /*prepare the CBW and relevent field*/
                usbh_msc_cbw_data.field.cbw_transfer_length = XFER_LEN_READ_CAPACITY10;
                usbh_msc_cbw_data.field.cbw_flags = USB_DIR_IN;
                usbh_msc_cbw_data.field.cbw_length = CBW_LENGTH;

                usbh_msc_botxfer_param.p_rx_tx_buff = usbh_data_in_buffer;
                usbh_msc_botxfer_param.msc_state_current = USBH_MSC_READ_CAPACITY10;

                for (index = CBW_CB_LENGTH; index != 0U; index--) {
                    usbh_msc_cbw_data.field.cbwcb[index] = 0x00U;
                }

                usbh_msc_cbw_data.field.cbwcb[0] = OPCODE_READ_CAPACITY10; 
                usbh_msc_botxfer_param.bot_state = USBH_MSC_SEND_CBW;

                /* start the transfer, then let the state machine manage the other transactions */
                usbh_msc_botxfer_param.msc_state = USBH_MSC_BOT_USB_TRANSFERS;
                usbh_msc_botxfer_param.bot_xfer_status = USBH_MSC_BUSY;
                usbh_msc_botxfer_param.cmd_state_machine = CMD_WAIT_STATUS;

                status = USBH_MSC_BUSY;
                break;

            case CMD_WAIT_STATUS:
                if (USBH_MSC_OK == usbh_msc_botxfer_param.bot_xfer_status) {
                    /*assign the capacity*/
                    (((uint8_t*)&usbh_msc_param.msc_capacity )[3]) = usbh_data_in_buffer[0];
                    (((uint8_t*)&usbh_msc_param.msc_capacity )[2]) = usbh_data_in_buffer[1];
                    (((uint8_t*)&usbh_msc_param.msc_capacity )[1]) = usbh_data_in_buffer[2];
                    (((uint8_t*)&usbh_msc_param.msc_capacity )[0]) = usbh_data_in_buffer[3];

                    /*assign the page length*/
                    (((uint8_t*)&usbh_msc_param.msc_page_length )[1]) = usbh_data_in_buffer[6];
                    (((uint8_t*)&usbh_msc_param.msc_page_length )[0]) = usbh_data_in_buffer[7];

                    /* commands successfully sent and response received  */       
                    usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
                    status = USBH_MSC_OK;      
                } else if (USBH_MSC_FAIL == usbh_msc_botxfer_param.bot_xfer_status) {
                    /* failure mode */
                    usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
                    status = USBH_MSC_FAIL;
                } else if (USBH_MSC_PHASE_ERROR == usbh_msc_botxfer_param.bot_xfer_status) {
                    /* failure mode */
                    usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
                    status = USBH_MSC_PHASE_ERROR;
                } else {
                    /* wait for the commands to get completed */
                    /* no change in state machine */
                }
                break;

            default:
                break;
        }
    }

    return status;
}

/*!
    \brief      send the mode sense6 command to the device
    \param[in]  pudev: pointer to usb device
    \param[out] none
    \retval     host operation status
*/
uint8_t usbh_msc_mode_sense6(usb_core_handle_struct *pudev)
{
    uint8_t index;
    usbh_msc_status_enum status = USBH_MSC_BUSY;

    if (hcd_is_device_connected(pudev)) {
        switch (usbh_msc_botxfer_param.cmd_state_machine) {
            case CMD_SEND_STATE:
                /*prepare the CBW and relevent field*/
                usbh_msc_cbw_data.field.cbw_transfer_length = XFER_LEN_MODE_SENSE6;
                usbh_msc_cbw_data.field.cbw_flags = USB_DIR_IN;
                usbh_msc_cbw_data.field.cbw_length = CBW_LENGTH;

                usbh_msc_botxfer_param.p_rx_tx_buff = usbh_data_in_buffer;
                usbh_msc_botxfer_param.msc_state_current = USBH_MSC_MODE_SENSE6;

                for (index = CBW_CB_LENGTH; index != 0; index--) {
                    usbh_msc_cbw_data.field.cbwcb[index] = 0x00U;
                }

                usbh_msc_cbw_data.field.cbwcb[0] = OPCODE_MODE_SENSE6; 
                usbh_msc_cbw_data.field.cbwcb[2] = MODE_SENSE_PAGE_CONTROL_FIELD | \
                                                    MODE_SENSE_PAGE_CODE;
                usbh_msc_cbw_data.field.cbwcb[4]  = XFER_LEN_MODE_SENSE6;
                usbh_msc_botxfer_param.bot_state = USBH_MSC_SEND_CBW;

                /* start the transfer, then let the state machine manage the other transactions */
                usbh_msc_botxfer_param.msc_state = USBH_MSC_BOT_USB_TRANSFERS;
                usbh_msc_botxfer_param.bot_xfer_status = USBH_MSC_BUSY;
                usbh_msc_botxfer_param.cmd_state_machine = CMD_WAIT_STATUS;

                status = USBH_MSC_BUSY;
                break;

            case CMD_WAIT_STATUS:
                if (USBH_MSC_OK == usbh_msc_botxfer_param.bot_xfer_status) {
                    /* assign the write protect status */
                    /* if writeprotect = 0, writing is allowed 
                       if writeprotect != 0, disk is write protected */
                    if (usbh_data_in_buffer[2] & MASK_MODE_SENSE_WRITE_PROTECT) {
                        usbh_msc_param.msc_write_protect = DISK_WRITE_PROTECTED;
                    } else {
                        usbh_msc_param.msc_write_protect = 0U;
                    }

                    /* commands successfully sent and response received */
                    usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
                    status = USBH_MSC_OK;      
                } else if (USBH_MSC_FAIL == usbh_msc_botxfer_param.bot_xfer_status) {
                    /* failure mode */
                    usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
                    status = USBH_MSC_FAIL;
                } else if (USBH_MSC_PHASE_ERROR == usbh_msc_botxfer_param.bot_xfer_status) {
                    /* failure mode */
                    usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
                    status = USBH_MSC_PHASE_ERROR;    
                } else {
                    /* wait for the commands to get completed */
                    /* no change in state machine */
                }
                break;

            default:
                break;
        }
    }

    return status;
}

/*!
    \brief      send the request sense command to the device
    \param[in]  pudev: pointer to usb device
    \param[out] none
    \retval     host operation status
*/
uint8_t usbh_msc_request_sense(usb_core_handle_struct *pudev)
{
    usbh_msc_status_enum status = USBH_MSC_BUSY;
    uint8_t index;

    if (hcd_is_device_connected(pudev)) {
        switch (usbh_msc_botxfer_param.cmd_state_machine) {
            case CMD_SEND_STATE:

            /*prepare the CBW and relevent field*/
            usbh_msc_cbw_data.field.cbw_transfer_length = ALLOCATION_LENGTH_REQUEST_SENSE;
            usbh_msc_cbw_data.field.cbw_flags = USB_DIR_IN;
            usbh_msc_cbw_data.field.cbw_length = CBW_LENGTH;

            usbh_msc_botxfer_param.p_rx_tx_buff = usbh_data_in_buffer;
            usbh_msc_botxfer_param.msc_state_bkp = usbh_msc_botxfer_param.msc_state_current;
            usbh_msc_botxfer_param.msc_state_current = USBH_MSC_REQUEST_SENSE;

            for (index = CBW_CB_LENGTH; index != 0; index--) {
                usbh_msc_cbw_data.field.cbwcb[index] = 0x00U;
            }

            usbh_msc_cbw_data.field.cbwcb[0] = OPCODE_REQUEST_SENSE; 
            usbh_msc_cbw_data.field.cbwcb[1] = DESC_REQUEST_SENSE;
            usbh_msc_cbw_data.field.cbwcb[4] = ALLOCATION_LENGTH_REQUEST_SENSE;

            usbh_msc_botxfer_param.bot_state = USBH_MSC_SEND_CBW;

            /* start the transfer, then let the state machine magage the other transactions */
            usbh_msc_botxfer_param.msc_state = USBH_MSC_BOT_USB_TRANSFERS;
            usbh_msc_botxfer_param.bot_xfer_status = USBH_MSC_BUSY;
            usbh_msc_botxfer_param.cmd_state_machine = CMD_WAIT_STATUS;

            status = USBH_MSC_BUSY;
            break;

        case CMD_WAIT_STATUS:
            if (USBH_MSC_OK == usbh_msc_botxfer_param.bot_xfer_status) {
                /* get sense data*/
                (((uint8_t*)&usbh_msc_param.msc_sense_key )[3]) = usbh_data_in_buffer[0];
                (((uint8_t*)&usbh_msc_param.msc_sense_key )[2]) = usbh_data_in_buffer[1];
                (((uint8_t*)&usbh_msc_param.msc_sense_key )[1]) = usbh_data_in_buffer[2];
                (((uint8_t*)&usbh_msc_param.msc_sense_key )[0]) = usbh_data_in_buffer[3];

                /* commands successfully sent and response received  */
                usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
                status = USBH_MSC_OK;      
            } else if (USBH_MSC_FAIL == usbh_msc_botxfer_param.bot_xfer_status) {
                /* failure mode */
                usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
                status = USBH_MSC_FAIL;
            } else if (USBH_MSC_PHASE_ERROR == usbh_msc_botxfer_param.bot_xfer_status) {
                /* failure mode */
                usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
                status = USBH_MSC_PHASE_ERROR;    
            } else {
                /* wait for the commands to get completed */
                /* no change in state machine */
            }
            break;

        default:
            break;
        }
    }

    return status;
}

/*!
    \brief      send the write command to the device
    \param[in]  pudev: pointer to usb device
    \param[in]  data_buffer: data buffer contains the data to write
    \param[in]  address: address to which the data will be written
    \param[in]  nb_of_bytes: number of bytes to be written
    \param[out] none
    \retval     host operation status
*/
uint8_t usbh_msc_write10(usb_core_handle_struct *pudev, 
                         uint8_t *data_buffer,
                         uint32_t address,
                         uint32_t nb_of_bytes)
{
    uint8_t index;
    usbh_msc_status_enum status = USBH_MSC_BUSY;
    uint16_t nb_of_pages;

    if (hcd_is_device_connected(pudev)) {
        switch (usbh_msc_botxfer_param.cmd_state_machine) {
            case CMD_SEND_STATE:   
                usbh_msc_cbw_data.field.cbw_transfer_length = nb_of_bytes;
                usbh_msc_cbw_data.field.cbw_flags = USB_DIR_OUT;
                usbh_msc_cbw_data.field.cbw_length = CBW_LENGTH;
                usbh_msc_botxfer_param.p_rx_tx_buff = data_buffer;

                for (index = CBW_CB_LENGTH; index != 0; index--) {
                    usbh_msc_cbw_data.field.cbwcb[index] = 0x00U;
                }

                usbh_msc_cbw_data.field.cbwcb[0]  = OPCODE_WRITE10; 

                /*logical block address*/
                usbh_msc_cbw_data.field.cbwcb[2]  = (((uint8_t*)&address)[3]);
                usbh_msc_cbw_data.field.cbwcb[3]  = (((uint8_t*)&address)[2]);
                usbh_msc_cbw_data.field.cbwcb[4]  = (((uint8_t*)&address)[1]);
                usbh_msc_cbw_data.field.cbwcb[5]  = (((uint8_t*)&address)[0]);

                /*USBH_MSC_PAGE_LENGTH = 512*/
                nb_of_pages = nb_of_bytes/ USBH_MSC_PAGE_LENGTH; 

                /*tranfer length */
                usbh_msc_cbw_data.field.cbwcb[7]  = (((uint8_t *)&nb_of_pages)[1]);
                usbh_msc_cbw_data.field.cbwcb[8]  = (((uint8_t *)&nb_of_pages)[0]);

                usbh_msc_botxfer_param.bot_state = USBH_MSC_SEND_CBW;

                /* start the transfer, then let the state machine magage the other transactions */
                usbh_msc_botxfer_param.msc_state = USBH_MSC_BOT_USB_TRANSFERS;
                usbh_msc_botxfer_param.bot_xfer_status = USBH_MSC_BUSY;
                usbh_msc_botxfer_param.cmd_state_machine = CMD_WAIT_STATUS;

                status = USBH_MSC_BUSY;
                break;

            case CMD_WAIT_STATUS:
                if (USBH_MSC_OK == usbh_msc_botxfer_param.bot_xfer_status) {
                    /* commands successfully sent and response received */
                    usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
                    status = USBH_MSC_OK;
                } else if (USBH_MSC_FAIL == usbh_msc_botxfer_param.bot_xfer_status) {
                    /* failure mode */
                    usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
                } else if (USBH_MSC_PHASE_ERROR == usbh_msc_botxfer_param.bot_xfer_status) {
                    /* failure mode */
                    usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
                    status = USBH_MSC_PHASE_ERROR;    
                }
                break;

            default:
                break;
        }
    }

    return status;
}

/*!
    \brief      send the read command to the device
    \param[in]  data_buffer: data buffer will contain the data to be read
    \param[in]  address: address from which the data will be read
    \param[in]  nb_of_bytes: number of bytes to be read
    \param[out] none
    \retval     host operation status
*/
uint8_t usbh_msc_read10(usb_core_handle_struct *pudev,
                        uint8_t *data_buffer,
                        uint32_t address,
                        uint32_t nb_of_bytes)
{
    uint8_t index;
    static usbh_msc_status_enum status = USBH_MSC_BUSY;
    uint16_t nb_of_pages;
    status = USBH_MSC_BUSY;

    if (hcd_is_device_connected(pudev)) {
        switch (usbh_msc_botxfer_param.cmd_state_machine) {
            case CMD_SEND_STATE:
                /*prepare the CBW and relevent field*/
                usbh_msc_cbw_data.field.cbw_transfer_length = nb_of_bytes;
                usbh_msc_cbw_data.field.cbw_flags = USB_DIR_IN;
                usbh_msc_cbw_data.field.cbw_length = CBW_LENGTH;

                usbh_msc_botxfer_param.p_rx_tx_buff = data_buffer;

                for (index = CBW_CB_LENGTH; index != 0; index--) {
                  usbh_msc_cbw_data.field.cbwcb[index] = 0x00U;
                }

                usbh_msc_cbw_data.field.cbwcb[0]  = OPCODE_READ10; 

                /*logical block address*/
                usbh_msc_cbw_data.field.cbwcb[2]  = (((uint8_t*)&address)[3]);
                usbh_msc_cbw_data.field.cbwcb[3]  = (((uint8_t*)&address)[2]);
                usbh_msc_cbw_data.field.cbwcb[4]  = (((uint8_t*)&address)[1]);
                usbh_msc_cbw_data.field.cbwcb[5]  = (((uint8_t*)&address)[0]);

                /*USBH_MSC_PAGE_LENGTH = 512*/
                nb_of_pages = nb_of_bytes/ USBH_MSC_PAGE_LENGTH;

                /*tranfer length */
                usbh_msc_cbw_data.field.cbwcb[7]  = (((uint8_t *)&nb_of_pages)[1]) ; 
                usbh_msc_cbw_data.field.cbwcb[8]  = (((uint8_t *)&nb_of_pages)[0]) ; 

                usbh_msc_botxfer_param.bot_state = USBH_MSC_SEND_CBW;

                /* start the transfer, then let the state machine magage the other transactions */
                usbh_msc_botxfer_param.msc_state = USBH_MSC_BOT_USB_TRANSFERS;
                usbh_msc_botxfer_param.bot_xfer_status = USBH_MSC_BUSY;
                usbh_msc_botxfer_param.cmd_state_machine = CMD_WAIT_STATUS;

                status = USBH_MSC_BUSY;
                break;

            case CMD_WAIT_STATUS:
                if ((USBH_MSC_OK == usbh_msc_botxfer_param.bot_xfer_status) && \
                    (hcd_is_device_connected(pudev))) {
                    /* commands successfully sent and response received  */
                    usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
                    status = USBH_MSC_OK;
                } else if ((USBH_MSC_FAIL == usbh_msc_botxfer_param.bot_xfer_status) && \
                        (hcd_is_device_connected(pudev))) {
                    /* failure mode */
                    usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
                } else if (USBH_MSC_PHASE_ERROR == usbh_msc_botxfer_param.bot_xfer_status) {
                    /* failure mode */
                    usbh_msc_botxfer_param.cmd_state_machine = CMD_SEND_STATE;
                    status = USBH_MSC_PHASE_ERROR;    
                } else {
                    /* wait for the commands to get completed */
                    /* no change in state machine */
                }
                break;

            default:
                break;
        }
    }

    return status;
}

