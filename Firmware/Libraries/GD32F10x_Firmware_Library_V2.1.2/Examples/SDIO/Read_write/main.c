/*!
    \file  main.c
    \brief SD card read and write demo 

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
#include "gd32f10x_eval.h"
#include "sdcard.h"
#include <stdio.h>

//#define DATA_PRINT                                        /* uncomment the macro to print out the data */

sd_card_info_struct sd_cardinfo;                            /* information of SD card */
uint32_t buf_write[512];                                    /* store the data written to the card */
uint32_t buf_read[512];                                     /* store the data read from the card */

void nvic_config(void);
sd_error_enum sd_io_init(void);
void card_info_get(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    sd_error_enum sd_error;
    uint16_t i = 5;
#ifdef DATA_PRINT
    uint8_t *pdata;
#endif /* DATA_PRINT */
    
    /* configure the NVIC, USART and LED */
    nvic_config();
    gd_eval_com_init(EVAL_COM0);
    gd_eval_led_init(LED2);
    gd_eval_led_init(LED3);
    gd_eval_led_init(LED4);
    gd_eval_led_init(LED5);
    
    /* turn off all the LEDs */
    gd_eval_led_off(LED2);
    gd_eval_led_off(LED3);
    gd_eval_led_off(LED4);
    gd_eval_led_off(LED5);
    
    /* initialize the card */
    do{
        sd_error = sd_io_init();
    }while((SD_OK != sd_error) && (--i));
    
    if(i){
        printf("\r\n Card init success!\r\n");
    }else{
        printf("\r\n Card init failed!\r\n");
        /* turn on LED2, LED4 and turn off LED3, LED5 */
        gd_eval_led_on(LED2);
        gd_eval_led_on(LED4);
        gd_eval_led_off(LED3);
        gd_eval_led_off(LED5);
        while (1){
        }
    }
    
    /* get the information of the card and print it out by USART */
    card_info_get();
    
    /* init the write buffer */
    for(i=0; i<512; i++){
        buf_write[i] = i;
    }
    
    printf("\r\n\r\n Card test:");
    
    /* single block operation test */
    sd_error = sd_block_write(buf_write, 100*512, 512);
    if(SD_OK != sd_error){
        printf("\r\n Block write fail!");
        /* turn on LED2, LED4 and turn off LED3, LED5 */
        gd_eval_led_on(LED2);
        gd_eval_led_on(LED4);
        gd_eval_led_off(LED3);
        gd_eval_led_off(LED5);
        while (1){
        }
    }else{
        printf("\r\n Block write success!");
    }
    sd_error = sd_block_read(buf_read, 100*512, 512);
    if(SD_OK != sd_error){
        printf("\r\n Block read fail!");
        /* turn on LED2, LED4 and turn off LED3, LED5 */
        gd_eval_led_on(LED2);
        gd_eval_led_on(LED4);
        gd_eval_led_off(LED3);
        gd_eval_led_off(LED5);
        while (1){
        }
    }else{
        printf("\r\n Block read success!");
#ifdef DATA_PRINT
        pdata = (uint8_t *)buf_read;
        /* print data by USART */
        printf("\r\n");
        for(i = 0; i < 128; i++){
            printf(" %3d %3d %3d %3d ", *pdata, *(pdata+1), *(pdata+2), *(pdata+3));
            pdata += 4;
            if(0 == (i + 1) % 4){
                printf("\r\n");
            }
        }
#endif /* DATA_PRINT */
    }
    
    /* lock and unlock operation test */
    if(SD_CCC_LOCK_CARD & sd_cardinfo.card_csd.ccc){
        /* lock the card */
        sd_error = sd_lock_unlock(SD_LOCK);
        if(SD_OK != sd_error){
            printf("\r\n Lock failed!");
            /* turn on LED2, LED4 and turn off LED3, LED5 */
            gd_eval_led_on(LED2);
            gd_eval_led_on(LED4);
            gd_eval_led_off(LED3);
            gd_eval_led_off(LED5);
            while (1){
            }
        }else{
            printf("\r\n The card is locked!");
        }
        sd_error = sd_erase(100*512, 101*512);
        if(SD_OK != sd_error){
            printf("\r\n Erase failed!");
        }else{
            printf("\r\n Erase success!");
        }
        
        /* unlock the card */
        sd_error = sd_lock_unlock(SD_UNLOCK);
        if(SD_OK != sd_error){
            printf("\r\n Unlock failed!");
            /* turn on LED2, LED4 and turn off LED3, LED5 */
            gd_eval_led_on(LED2);
            gd_eval_led_on(LED4);
            gd_eval_led_off(LED3);
            gd_eval_led_off(LED5);
            while (1){
            }
        }else{
            printf("\r\n The card is unlocked!");
        }
        sd_error = sd_erase(100*512, 101*512);
        if(SD_OK != sd_error){
            printf("\r\n Erase failed!");
        }else{
            printf("\r\n Erase success!");
        }
        
        sd_error = sd_block_read(buf_read, 100*512, 512);
        if(SD_OK != sd_error){
            printf("\r\n Block read fail!");
            /* turn on LED2, LED4 and turn off LED3, LED5 */
            gd_eval_led_on(LED2);
            gd_eval_led_on(LED4);
            gd_eval_led_off(LED3);
            gd_eval_led_off(LED5);
            while (1){
            }
        }else{
            printf("\r\n Block read success!");
#ifdef DATA_PRINT
        pdata = (uint8_t *)buf_read;
        /* print data by USART */
        printf("\r\n");
        for(i = 0; i < 128; i++){
            printf(" %3d %3d %3d %3d ", *pdata, *(pdata+1), *(pdata+2), *(pdata+3));
            pdata += 4;
            if(0 == (i + 1) % 4){
                printf("\r\n");
            }
        }
#endif /* DATA_PRINT */
        }
    }
    
    /* multiple blocks operation test */
    sd_error = sd_multiblocks_write(buf_write, 200*512, 512, 3);
    if(SD_OK != sd_error){
        printf("\r\n Multiple block write fail!");
        /* turn on LED2, LED4 and turn off LED3, LED5 */
        gd_eval_led_on(LED2);
        gd_eval_led_on(LED4);
        gd_eval_led_off(LED3);
        gd_eval_led_off(LED5);
        while (1){
        }
    }else{
        printf("\r\n Multiple block write success!");
    }
    sd_error = sd_multiblocks_read(buf_read, 200*512, 512, 3);
    if(SD_OK != sd_error){
        printf("\r\n Multiple block read fail!");
        /* turn on LED2, LED4 and turn off LED3, LED5 */
        gd_eval_led_on(LED2);
        gd_eval_led_on(LED4);
        gd_eval_led_off(LED3);
        gd_eval_led_off(LED5);
        while (1){
        }
    }else{
        printf("\r\n Multiple block read success!");
#ifdef DATA_PRINT
        pdata = (uint8_t *)buf_read;
        /* print data by USART */
        printf("\r\n");
        for(i = 0; i < 512; i++){
            printf(" %3d %3d %3d %3d ", *pdata, *(pdata+1), *(pdata+2), *(pdata+3));
            pdata += 4;
            if(0 == (i + 1) % 4){
                printf("\r\n");
            }
        }
#endif /* DATA_PRINT */
    }
    
    /* turn on all the LEDs */
    gd_eval_led_on(LED2);
    gd_eval_led_on(LED3);
    gd_eval_led_on(LED4);
    gd_eval_led_on(LED5);
    while (1){
    }
}

/*!
    \brief      configure the NVIC
    \param[in]  none
    \param[out] none
    \retval     none
*/
void nvic_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(SDIO_IRQn, 0, 0);
}

/*!
    \brief      initialize the card, get the card information, set the bus mode and transfer mode
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_io_init(void)
{
    sd_error_enum status = SD_OK;
    uint32_t cardstate = 0;
    status = sd_init();
    if(SD_OK == status){
        status = sd_card_information_get(&sd_cardinfo);
    }
    if(SD_OK == status){
        status = sd_card_select_deselect(sd_cardinfo.card_rca);
    }
    status = sd_cardstatus_get(&cardstate);
    if(cardstate & 0x02000000){
        printf("\r\n the card is locked!");
        while (1){
        }
    }
    if ((SD_OK == status) && (!(cardstate & 0x02000000)))
    {
        /* set bus mode */
        status = sd_bus_mode_config(SDIO_BUSMODE_4BIT);
//        status = sd_bus_mode_config( SDIO_BUSMODE_1BIT );
    }
    if (SD_OK == status)
    {
        /* set data transfer mode */
//        status = sd_transfer_mode_config( SD_DMA_MODE );
        status = sd_transfer_mode_config( SD_POLLING_MODE );
    }
    return status;
}

/*!
    \brief      get the card information and print it out by USRAT
    \param[in]  none
    \param[out] none
    \retval     none
*/
void card_info_get(void)
{
    uint8_t sd_spec, sd_spec3, sd_spec4, sd_security;
    uint32_t block_count, block_size;
    uint16_t temp_ccc;
    printf("\r\n Card information:");
    sd_spec = (sd_scr[1] & 0x0F000000) >> 24;
    sd_spec3 = (sd_scr[1] & 0x00008000) >> 15;
    sd_spec4 = (sd_scr[1] & 0x00000400) >> 10;
    if(2 == sd_spec)
    {
        if(1 == sd_spec3)
        {
            if(1 == sd_spec4) 
            {
                printf("\r\n## Card version 4.xx ##");
            }
            else 
            {
                printf("\r\n## Card version 3.0x ##");
            }
        }
        else 
        {
            printf("\r\n## Card version 2.00 ##");
        }
    }
    else if(1 == sd_spec) 
    {
        printf("\r\n## Card version 1.10 ##");
    }
    else if(0 == sd_spec) 
    {
        printf("\r\n## Card version 1.0x ##");
    }
    
    sd_security = (sd_scr[1] & 0x00700000) >> 20;
    if(2 == sd_security) 
    {
        printf("\r\n## SDSC card ##");
    }
    else if(3 == sd_security) 
    {   
        printf("\r\n## SDHC card ##");
    }
    else if(4 == sd_security) 
    {
        printf("\r\n## SDXC card ##");
    }
    
    block_count = (sd_cardinfo.card_csd.c_size + 1)*1024;
    block_size = 512;
    printf("\r\n## Device size is %dKB ##", sd_card_capacity_get());
    printf("\r\n## Block size is %dB ##", block_size);
    printf("\r\n## Block count is %d ##", block_count);
    
    if(sd_cardinfo.card_csd.read_bl_partial){
        printf("\r\n## Partial blocks for read allowed ##" );
    }
    if(sd_cardinfo.card_csd.write_bl_partial){
        printf("\r\n## Partial blocks for write allowed ##" );
    }
    temp_ccc = sd_cardinfo.card_csd.ccc;
    printf("\r\n## CardCommandClasses is: %x ##", temp_ccc);
    if((SD_CCC_BLOCK_READ & temp_ccc) && (SD_CCC_BLOCK_WRITE & temp_ccc)){
        printf("\r\n## Block operation supported ##");
    }
    if(SD_CCC_ERASE & temp_ccc){
        printf("\r\n## Erase supported ##");
    }
    if(SD_CCC_WRITE_PROTECTION & temp_ccc){
        printf("\r\n## Write protection supported ##");
    }
    if(SD_CCC_LOCK_CARD & temp_ccc){
        printf("\r\n## Lock unlock supported ##");
    }
    if(SD_CCC_APPLICATION_SPECIFIC & temp_ccc){
        printf("\r\n## Application specific supported ##");
    }
    if(SD_CCC_IO_MODE & temp_ccc){
        printf("\r\n## I/O mode supported ##");
    }
    if(SD_CCC_SWITCH & temp_ccc){
        printf("\r\n## Switch function supported ##");
    }
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM0, (uint8_t)ch);
    while (RESET == usart_flag_get(EVAL_COM0, USART_FLAG_TBE));
    return ch;
}
