/*!
    \file  usbh_msc_fatfs.c 
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

#include "usb_conf.h"
#include "diskio.h"
#include "usbh_msc_core.h"

static volatile DSTATUS stat = STA_NOINIT; /* disk status */

extern usb_core_handle_struct          usb_core_dev;
extern usbh_host_struct                usb_host;
extern usbh_state_handle_struct        usbh_state_core;

/*!
    \brief      initialize disk drive
    \param[in]  drv: physical drive number (0)
    \param[out] none
    \retval     status
*/
DSTATUS disk_initialize (BYTE drv)
{
    if (hcd_is_device_connected(&usb_core_dev)) {
        stat &= ~STA_NOINIT;
    }

    return stat;
}

/*!
    \brief      get disk status
    \param[in]  drv: physical drive number (0)
    \param[out] none
    \retval     status
*/
DSTATUS disk_status (BYTE drv)
{
    /* supports only single drive */
    if (drv) return STA_NOINIT; 

    return stat;
}

/*!
    \brief      read sector(s)
    \param[in]  drv: physical drive number (0)
    \param[in]  buff: pointer to the data buffer to store read data
    \param[in]  sector: start sector number (LBA)
    \param[in]  count: sector count (1..255)
    \param[out] none
    \retval     status
*/
DRESULT disk_read (BYTE drv, 
                   BYTE *buff, 
                   DWORD sector, 
                   BYTE count)
{
    BYTE status = USBH_MSC_OK;

    if (drv || !count) return RES_PARERR;
    if (stat & STA_NOINIT) return RES_NOTRDY;

    if (hcd_is_device_connected(&usb_core_dev)) {
        do {
            status = usbh_msc_read10(&usb_core_dev, buff,sector, 512 * count);
            usbh_msc_handle_botxfer(&usb_core_dev, &usb_host,  &usbh_state_core);

            if (!hcd_is_device_connected(&usb_core_dev)) {
                return RES_ERROR;
            }
        }
        while(USBH_MSC_BUSY == status);
    }

    if (USBH_MSC_OK == status) return RES_OK;

    return RES_ERROR;
}

#if _READONLY == 0

/*!
    \brief      write sector(s)
    \param[in]  drv: physical drive number (0)
    \param[in]  buff: pointer to the data buffer to be written
    \param[in]  sector: start sector number (LBA)
    \param[in]  count: sector count (1..255)
    \param[out] none
    \retval     status
*/
DRESULT disk_write (BYTE drv, 
                    const BYTE *buff, 
                    DWORD sector, 
                    BYTE count)
{
    BYTE status = USBH_MSC_OK;
    if (drv || !count) return RES_PARERR;
    if (stat & STA_NOINIT) return RES_NOTRDY;
    if (stat & STA_PROTECT) return RES_WRPRT;

    if (hcd_is_device_connected(&usb_core_dev)) {
        do {
            status = usbh_msc_write10(&usb_core_dev,(BYTE*)buff,sector,512 * count);
            usbh_msc_handle_botxfer(&usb_core_dev, &usb_host,  &usbh_state_core);

            if (!hcd_is_device_connected(&usb_core_dev)) {
                return RES_ERROR;
            }
        } while(USBH_MSC_BUSY == status);
    }

    if (USBH_MSC_OK == status) return RES_OK;

    return RES_ERROR;
}

#endif /* _READONLY == 0 */

/*-----------------------------------------------------------------------*/
/* miscellaneous functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL != 0

/*!
    \brief      I/O control function
    \param[in]  drv: physical drive number (0)
    \param[in]  ctrl: control code
    \param[in]  buff: buffer to send/receive control data
    \param[out] none
    \retval     status
*/
DRESULT disk_ioctl (BYTE drv, 
                    BYTE ctrl, 
                    void *buff)
{
    DRESULT res = RES_OK;

    if (drv) return RES_PARERR;

    res = RES_ERROR;

    if (stat & STA_NOINIT) return RES_NOTRDY;

    switch (ctrl) {
        case CTRL_SYNC:         /* make sure that no pending write process */
            res = RES_OK;
            break;

        case GET_SECTOR_COUNT:  /* get number of sectors on the disk (DWORD) */
            *(DWORD*)buff = (DWORD) usbh_msc_param.msc_capacity;
            res = RES_OK;
            break;

        case GET_SECTOR_SIZE:  /* get R/W sector size (WORD) */
            *(WORD*)buff = 512U;
            res = RES_OK;
            break;

        case GET_BLOCK_SIZE:   /* get erase block size in unit of sector (DWORD) */
            *(DWORD*)buff = 512U;
            break;

        default:
            res = RES_PARERR;
            break;
    }

    return res;
}
#endif /* _USE_IOCTL != 0 */

