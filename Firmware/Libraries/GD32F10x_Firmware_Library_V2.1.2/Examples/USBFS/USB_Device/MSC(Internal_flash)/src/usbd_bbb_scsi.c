/*!
    \file  usbd_bbb_scsi.c
    \brief USB BBB and SCSI protocol driver functions

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

#include "usbd_bbb_scsi.h"
#include "usbd_flash_access.h"

/* USB mass storage disk inquiry data */
uint8_t disk_inquiry_data[DISK_INQUIRY_DATA_LENGTH]=
{
    0x00,                                   /* bit0-bit4: peripheral device type */
    0x80,                                   /* bit7: rmb */
    0x00,                                   /* bit6-bit7: iso version, bit3-bit5:ecma version, bit0-bit2:ansi version (00h) */
    0x01,                                   /* bit0-bit3: response data format */
    0x1f,                                   /* additional length (31) */
    0x00,                                   /* reserved */
    0x00,                                   /* reserved */
    0x00,                                   /* reserved */
    'G', 'D', '3', '2', ' ', ' ', ' ', ' ', /* vendor information */
  
    'I', 'n', 't', 'e', 'r', 'n', 'a', 'l', /* product identification */
    '-', 'F', 'L', 'A', 'S', 'H', ' ', ' ',
  
    '1', '.', '0' ,'0',                     /* product revision level */
};

/* USB mass storage format capacities data */
uint8_t format_capacities_data[FORMAT_CAPACITIES_DATA_LENGTH] =
{
    0x00, 0x00, 0x00,                         /* reserved */
    0x08,                                     /* capacity list length */
    (uint8_t)((ISFLASH_BLOCK_NUM - 1) >> 24),   /* number of blocks (MSB) */
    (uint8_t)((ISFLASH_BLOCK_NUM - 1) >> 16),   /* number of blocks (MSB) */
    (uint8_t)((ISFLASH_BLOCK_NUM - 1) >> 8),    /* number of blocks (MSB) */
    (uint8_t)(ISFLASH_BLOCK_NUM - 1),           /* number of blocks (MSB) */
    0x02,                                     /* bit0 - bit1:descriptor code */
    (uint8_t)((ISFLASH_BLOCK_SIZE) >> 16),      /* block length (MSB) */
    (uint8_t)((ISFLASH_BLOCK_SIZE) >> 8),       /* block length (MSB) */
    (uint8_t)(ISFLASH_BLOCK_SIZE)               /* block length (MSB) */
};

/* USB mass storage read capacities data */
uint8_t read_capacities_data[READ_CAPACITIES_DATA_LENGTH] = 
{
    (uint8_t)((ISFLASH_BLOCK_NUM - 1) >> 24),   /* last logical block address (MSB) */
    (uint8_t)((ISFLASH_BLOCK_NUM - 1) >> 16),   /* last logical block address (MSB) */
    (uint8_t)((ISFLASH_BLOCK_NUM - 1) >> 8),    /* last logical block address (MSB) */
    (uint8_t)(ISFLASH_BLOCK_NUM - 1),           /* last logical block address (MSB) */

    (uint8_t)((ISFLASH_BLOCK_SIZE) >> 24),      /* block length in bytes (MSB) */
    (uint8_t)((ISFLASH_BLOCK_SIZE) >> 16),      /* block length in bytes (MSB) */
    (uint8_t)((ISFLASH_BLOCK_SIZE) >> 8),       /* block length in bytes (MSB) */
    (uint8_t)(ISFLASH_BLOCK_SIZE)               /* block length in bytes (MSB) */
};

/* USB mass storage request sense data */
uint8_t request_sense_data[REQUEST_SENSE_DATA_LENGTH] = 
{
    0x70,                                   /* bit0-bit6: error code (70h), bit7:valid */
    0x00,                                   /* reserved */
    0x05,                                   /* bit0-bit3:sense key */
    0x00, 0x00, 0x00, 0x00,                 /* information (MSB) */
    0x0A,                                   /* additional sense length (10) */
    0x00, 0x00, 0x00, 0x00,                 /* reserved */
    0x20,                                   /* additional sense code (mandatory) */
    0x00,                                   /* additional sense code qualifier (mandatory) */
    0x00, 0x00, 0x00, 0x00                  /* reserved */
};

/* USB mass storage sense 6  data */
uint8_t  sense6_data[SENSE6_DATA_LENGTH] = 
{
    0x03,                                   /* mode data length */
    0x00,                                   /* medium type */
    0x00,                                   /* device specific parameter, bit7: 0(write and read), 1(read only) */
    0x00,                                   /* block descriptor length */
    0x00,                                   /* reserved */
    0x00,                                   /* reserved */
    0x00,                                   /* reserved */
    0x00                                    /* reserved */
};

/* the array to store the SCSI command */
uint8_t scsi_cmd[SCSI_CMD_LENGTH];

/* the global data to store the transport stage in bbb protocol */
uint8_t g_bbb_transport_stage;

/* the data that need to return the host when the command is not valid */
uint8_t error_data = 0;

/* the array to store the datas from or to host in read10 or write10 command */
uint8_t bbb_data[MSC_MEDIA_PACKET_SIZE];

bbb_cbw_struct bbb_cbw;
bbb_csw_struct bbb_csw;

uint8_t* p_bbb_return_data;
uint32_t g_return_data_length;

uint32_t block_len;
uint32_t block_addr;

uint32_t disk_pop = 0;


/*!
    \brief      parse the SCSI commands from the CBW
    \param[in]  none
    \param[out] none
    \retval     none
*/
void scsi_command_parse(void)
{
    uint8_t i;

    for (i = 0; i < SCSI_CMD_LENGTH; i++) {
        scsi_cmd[i] = bbb_cbw.CBWCB[i];
    }
}

/*!
    \brief      set the CSW struct
    \param[in]  csw_tag: signature that helps identify this data packet as a csw
    \param[in]  csw_data_residue: the device shall set this field to the value received in the dcbwtag of the associated bbb_cbw
    \param[in]  csw_status: for data-out: the difference between the receive datas and dCBWDataTransferLength
                            for data-in: the difference between the transmit datas and dCBWDataTransferLength 
    \param[out] none
    \retval     none
*/
void bbb_set_csw(uint32_t csw_tag, uint32_t csw_data_residue, uint8_t csw_status)
{
    bbb_csw.dCSWSignature = BBB_CSW_SIGNATURE;
    bbb_csw.dCSWTag  = csw_tag;
    bbb_csw.dCSWDataResidue = csw_data_residue;
    bbb_csw.bCSWStatus = csw_status;
}

/*!
    \brief      report array to host
    \param[in]  pudev: pointer to device instance
    \param[in]  p_data: pointer to the send array
    \param[in]  report_length: the report length
    \param[out] none
    \retval     none
*/
void bbb_report_array_to_host(void *pudev, uint8_t *p_data, uint32_t report_length)
{
    uint32_t csw_tag;
    uint32_t csw_data_residue;
    uint8_t csw_status;
  
    /* send the array in DATA_IN_STAGE */
    if (COMMAND_STAGE == g_bbb_transport_stage) {
        g_bbb_transport_stage = DATA_IN_STAGE;
        p_bbb_return_data = p_data;
        g_return_data_length = bbb_cbw.dCBWDataTransferLength;

        if (g_return_data_length > report_length) {
            g_return_data_length = report_length;
        }

        usbd_ep_tx (pudev, MSC_IN_EP, (uint8_t *)p_bbb_return_data, g_return_data_length);
    /* send the CSW in STATUS_STAGE */
    } else if (DATA_IN_STAGE == g_bbb_transport_stage) {
        csw_tag = bbb_cbw.dCBWTag;
        csw_data_residue = bbb_cbw.dCBWDataTransferLength - g_return_data_length;
        csw_status = 0;
        bbb_set_csw(csw_tag, csw_data_residue, csw_status);
        p_bbb_return_data = (uint8_t *)&bbb_csw;
        g_return_data_length = BBB_CSW_LENGTH;
        g_bbb_transport_stage = STATUS_STAGE;
        usbd_ep_tx (pudev, MSC_IN_EP, (uint8_t *)p_bbb_return_data, g_return_data_length);
    /* set the transport stage to COMMAND_STAGE, and set the msc out endpoint to receive the bbb_cbw */
    } else if (STATUS_STAGE == g_bbb_transport_stage) {
        g_bbb_transport_stage = COMMAND_STAGE;
        usbd_ep_rx (pudev, MSC_OUT_EP, (uint8_t *)&bbb_cbw, BBB_CBW_LENGTH);

        return;
    }
}

/*!
    \brief      process the scsi command
    \param[in]  pudev: pointer to device instance
    \param[out] none
    \retval     none
*/
void process_scsi_command(void *pudev)
{
    uint8_t csw_status;
    uint32_t csw_tag;
    uint32_t csw_data_residue;
    static uint32_t len = 0;

    switch (scsi_cmd[0]) {
        /* process the INQUIRY command */
        case INQUIRY:
            bbb_report_array_to_host(pudev, disk_inquiry_data, DISK_INQUIRY_DATA_LENGTH);
            break;

        /* process the READ_FORMAT_CAPACITIES command */
        case READ_FORMAT_CAPACITIES:
            bbb_report_array_to_host (pudev, 
                                      format_capacities_data, 
                                      FORMAT_CAPACITIES_DATA_LENGTH);
            break;

        /* process the READ_CAPACITY command */
        case READ_CAPACITY:
            bbb_report_array_to_host(pudev, read_capacities_data, READ_CAPACITIES_DATA_LENGTH);
            break;

        /* process the READ_10 command */
        case READ_10:
            /* read the frist block */
            if (COMMAND_STAGE == g_bbb_transport_stage) {
                g_bbb_transport_stage = DATA_IN_STAGE;

                block_addr = (scsi_cmd[2] << 24) | \
                             (scsi_cmd[3] << 16) | \
                             (scsi_cmd[4] << 8) | scsi_cmd[5];
                block_len = (scsi_cmd[7] << 8) | scsi_cmd[8];

                block_addr *= ISFLASH_BLOCK_SIZE;
                block_len *= ISFLASH_BLOCK_SIZE;

                len = USB_MIN(block_len, MSC_MEDIA_PACKET_SIZE);
                flash_multi_blocks_read (bbb_data, 
                                        block_addr, 
                                        ISFLASH_BLOCK_SIZE, 
                                        len / ISFLASH_BLOCK_SIZE);
                usbd_ep_tx (pudev, MSC_IN_EP, bbb_data, len);

                block_addr += len;
                block_len -= len;
            } else if (DATA_IN_STAGE == g_bbb_transport_stage) {
                /* end of the read, send bbb_csw to host */
                if (0 == block_len) {
                    g_bbb_transport_stage = STATUS_STAGE;
                    csw_tag = bbb_cbw.dCBWTag;
                    csw_data_residue = bbb_cbw.dCBWDataTransferLength - ((scsi_cmd[7] << 8) | scsi_cmd[8]) * ISFLASH_BLOCK_SIZE;
                    csw_status = 0;
                    len = 0;
                    bbb_set_csw(csw_tag, csw_data_residue, csw_status);
                    usbd_ep_tx (pudev, MSC_IN_EP, (uint8_t *)&bbb_csw, BBB_CSW_LENGTH);
                /* read the nest block */
                } else {
                    len = USB_MIN(block_len, MSC_MEDIA_PACKET_SIZE);
                    flash_multi_blocks_read (bbb_data, 
                                            block_addr, 
                                            ISFLASH_BLOCK_SIZE, 
                                            len / ISFLASH_BLOCK_SIZE);
                    usbd_ep_tx (pudev, MSC_IN_EP, bbb_data, len);
                    block_addr += len;
                    block_len -= len;
                }
            /* enter the COMMAND_STAGE */
            } else if (STATUS_STAGE == g_bbb_transport_stage) {
                g_bbb_transport_stage = COMMAND_STAGE;
                usbd_ep_rx (pudev, MSC_OUT_EP, (uint8_t *)&bbb_cbw, BBB_CBW_LENGTH);
            }
            break;

        /* process the WRITE_10 command */
        case WRITE_10:
            /* write the first block */
            if (COMMAND_STAGE == g_bbb_transport_stage) {
                g_bbb_transport_stage = DATA_OUT_STAGE;

                block_addr = (scsi_cmd[2] << 24) | \
                             (scsi_cmd[3] << 16) | \
                             (scsi_cmd[4] << 8) | \
                              scsi_cmd[5];
                block_len = (scsi_cmd[7] << 8) | scsi_cmd[8];

                block_addr *= ISFLASH_BLOCK_SIZE;
                block_len *= ISFLASH_BLOCK_SIZE;

                len = USB_MIN(block_len, MSC_MEDIA_PACKET_SIZE);
                usbd_ep_rx (pudev, MSC_OUT_EP, bbb_data, len);
            } else if (DATA_OUT_STAGE == g_bbb_transport_stage) {
                flash_multi_blocks_write (bbb_data, 
                                         block_addr, 
                                         ISFLASH_BLOCK_SIZE, 
                                         len / ISFLASH_BLOCK_SIZE);

                block_addr += len;
                block_len -= len;

                /* end of the write, send bbb_csw to host */
                if (0 == block_len) {
                    g_bbb_transport_stage = STATUS_STAGE;
                    csw_tag = bbb_cbw.dCBWTag;
                    csw_data_residue = bbb_cbw.dCBWDataTransferLength - ((scsi_cmd[7] << 8) | scsi_cmd[8]) * ISFLASH_BLOCK_SIZE;
                    csw_status = 0;
                    len = 0;
                    bbb_set_csw(csw_tag, csw_data_residue, csw_status);
                    usbd_ep_tx (pudev, MSC_IN_EP, (uint8_t *)&bbb_csw, BBB_CSW_LENGTH);
                /* receive the next block datas */
                } else {
                    len = USB_MIN(block_len, MSC_MEDIA_PACKET_SIZE);
                    usbd_ep_rx (pudev, MSC_OUT_EP, bbb_data, len);
                }
            /* enter the COMMAND_STAGE */
            } else if (STATUS_STAGE == g_bbb_transport_stage) {
                g_bbb_transport_stage = COMMAND_STAGE;
                usbd_ep_rx (pudev, MSC_OUT_EP, (uint8_t *)&bbb_cbw, BBB_CBW_LENGTH);
            }
            break;

        /* process the REQUEST_SENSE command */
        case REQUEST_SENSE:
            bbb_report_array_to_host(pudev, request_sense_data, REQUEST_SENSE_DATA_LENGTH);
            break;

        /* process the TEST_UNIT_READY command */
        case TEST_UNIT_READY:
            if (COMMAND_STAGE == g_bbb_transport_stage) {
                g_bbb_transport_stage = STATUS_STAGE;
                bbb_set_csw(bbb_cbw.dCBWTag, 0, 0);
                usbd_ep_tx (pudev, MSC_IN_EP, (uint8_t *)&bbb_csw, BBB_CSW_LENGTH);
            } else {
                g_bbb_transport_stage = COMMAND_STAGE;
                usbd_ep_rx (pudev, MSC_OUT_EP, (uint8_t *)&bbb_cbw, BBB_CBW_LENGTH);
            }
            if(disk_pop == 1)
            USB_SOFT_DISCONNECT_ENABLE();
            break;

        case SCSI_START_STOP_UNIT:
             disk_pop = 1;
        case SCSI_ALLOW_MEDIUM_REMOVAL:
            if (COMMAND_STAGE == g_bbb_transport_stage) {
                g_bbb_transport_stage = STATUS_STAGE;
                bbb_set_csw(bbb_cbw.dCBWTag, 0, 0);
                usbd_ep_tx (pudev, MSC_IN_EP, (uint8_t *)&bbb_csw, BBB_CSW_LENGTH);
            } else {
                g_bbb_transport_stage = COMMAND_STAGE;
                usbd_ep_rx (pudev, MSC_OUT_EP, (uint8_t *)&bbb_cbw, BBB_CBW_LENGTH);
            }
            break;

        /* process the SCSI_MODE_SENSE6 command */
        case SCSI_MODE_SENSE6:
            bbb_report_array_to_host(pudev, sense6_data, SENSE6_DATA_LENGTH);
            break;

        /* default process: the SCSI command is not valid */
        default:
            /* if the data stage is DATA_IN, return the error_data to host in DATA_IN_STAGE */
            if (0x80 & bbb_cbw.bmCBWFlags) {
                if (COMMAND_STAGE == g_bbb_transport_stage) {
                    g_bbb_transport_stage = DATA_IN_STAGE;
                    g_return_data_length = 1;
                    usbd_ep_tx (pudev, MSC_IN_EP, &error_data, 1);
                } else if (DATA_IN_STAGE == g_bbb_transport_stage) {
                    g_bbb_transport_stage = STATUS_STAGE;
                    bbb_set_csw(bbb_cbw.dCBWTag , bbb_cbw.dCBWDataTransferLength - 1, 1);
                    usbd_ep_tx (pudev, MSC_IN_EP, (uint8_t *)&bbb_csw, BBB_CSW_LENGTH);
                } else if (STATUS_STAGE == g_bbb_transport_stage) {
                    g_bbb_transport_stage = COMMAND_STAGE;
                    usbd_ep_rx (pudev, MSC_OUT_EP, (uint8_t *)&bbb_cbw, BBB_CBW_LENGTH);
                }

            /* if the data stage is DATA_OUT, return the bbb_csw to host immediately */
            } else {
                if (COMMAND_STAGE == g_bbb_transport_stage) {
                    g_return_data_length = 0;
                    bbb_set_csw(bbb_cbw.dCBWTag, bbb_cbw.dCBWDataTransferLength, 1);
                    usbd_ep_tx (pudev, MSC_IN_EP, (uint8_t *)&bbb_csw, BBB_CSW_LENGTH);
                    g_bbb_transport_stage = STATUS_STAGE;
                } else if (STATUS_STAGE == g_bbb_transport_stage) {
                    g_bbb_transport_stage = COMMAND_STAGE;
                    usbd_ep_rx (pudev, MSC_OUT_EP, (uint8_t *)&bbb_cbw, BBB_CBW_LENGTH);
                }
            }
            break;
    }
}
