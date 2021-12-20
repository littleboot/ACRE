/*!
    \file  main.c
    \brief NAND test demo

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

#include "gd32f10x.h"
#include <stdio.h>
#include "gd32f10x_eval.h"
#include "exmc_nandflash.h"

#define BUFFER_SIZE                 (0x100U)
#define NAND_HY_MAKERID             (0xADU)
#define NAND_HY_DEVICEID            (0xF1U)

nand_id_struct nand_id;
uint8_t txbuffer[BUFFER_SIZE], rxbuffer[BUFFER_SIZE];
__IO uint32_t writereadstatus = 0, status= 0;
uint32_t j = 0;
uint32_t k = 0;
uint32_t writereadaddr ;
uint16_t zone, block, page, pageoffset;

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* LED initialize */
    gd_eval_led_init(LED2);
    gd_eval_led_init(LED5);
    /* config the USART */
    gd_eval_com_init(EVAL_COM0);
    /* config the EXMC access mode */
    exmc_nandflash_init();
    /* read NAND ID */
    nand_read_id(&nand_id);

    printf("read NAND ID");
    /* print NAND ID */
    printf("\r\nNand flash ID:0x%X 0x%X 0x%X 0x%X\r\n",nand_id.maker_id,nand_id.device_id,
                                               nand_id.third_id,nand_id.fourth_id);

    if((NAND_HY_MAKERID == nand_id.maker_id) && (NAND_HY_DEVICEID == nand_id.device_id))
    {
        /* set the read and write the address */
        zone = 0;
        block = 10;
        page = 0;
        pageoffset = 1100;
        writereadaddr = ((zone * NAND_ZONE_SIZE + block) * NAND_BLOCK_SIZE + page) 
                        * NAND_PAGE_SIZE + pageoffset;

        /* whether address cross-border */
        if((writereadaddr + BUFFER_SIZE ) > NAND_MAX_ADDRESS)
        {
            printf("\r\naddress cross-border");

            /* failure, light on LED5 */
            gd_eval_led_on(LED5);
            while(1);
        }

        /* fill writebuffer with 0x00.. */
        fill_buffer_nand(txbuffer, BUFFER_SIZE , 0x00);

        /* write data to nand flash */
        status = nand_write(writereadaddr, txbuffer, BUFFER_SIZE);
        if(NAND_OK == status)
        {
            printf("\r\nwrite data successfully!");
        }
        else
        {
            printf("\r\nwrite data failure!");

            /* failure, light on LED5 */
            gd_eval_led_on(LED5);
            while(1);
        }

        /* read data from nand flash */
        status = nand_read(writereadaddr, rxbuffer, BUFFER_SIZE);
        if(NAND_OK == status)
        {
            printf("\r\nread data successfully!");
        }
        else
        {
            printf("\r\nread data failure!\n\r");

            /* failure, light on LED5 */
            gd_eval_led_on(LED5);
            while(1);
        }

        /* Read and write data comparison for equality */
        writereadstatus = 0;
        for(j = 0; j < BUFFER_SIZE; j++)
        {
            if(txbuffer[j] != rxbuffer[j])
            {
                writereadstatus++;
                break;
            }
        }

        printf("\r\nthe result to access the nand flash:");
        if (writereadstatus == 0)
        { 
            printf("\r\naccess NAND flash successfully!");

            gd_eval_led_on(LED2);
        }
        else
        { 
            printf("\r\naccess NAND flash failure!");
            
            /* failure, light on LED5 */
            gd_eval_led_on(LED5);
            while(1);
        }
        printf("\r\nprintf data to be read:\r\n");
        for(k = 0; k < BUFFER_SIZE; k ++)
        {
            printf("0x%02X ",rxbuffer[k]);
        }
    }
    else
    {
        printf("\n\rread NAND ID failure!\n\r");
        
        /* failure, light on LED5 */
        gd_eval_led_on(LED5);
        while(1);
    }

    while (1);
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM0, (uint8_t) ch);
    while (RESET == usart_flag_get(EVAL_COM0,USART_FLAG_TBE));
    return ch;
}
