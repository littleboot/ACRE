/*!
    \file  main.c
    \brief ADC0 EXTI trigger regular and inserted channel
    
    \version 2017-12-26, V1.0.0, firmware for GD32F10x
*/

/*
    Copyright (c) 2017, GigaDevice Semiconductor Inc.

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
#include "systick.h"
#include <stdio.h>
#include "gd32f10x_eval.h"

uint16_t adc_value[4];
uint16_t inserted_data[4];

void rcu_config(void);
void gpio_config(void);
void nvic_config(void);
void dma_config(void);
void adc_config(void);
void exti_config(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

int main(void)
{
    /* system clocks configuration */
    rcu_config();
    /* GPIO configuration */
    gpio_config();
    /* configure the EXTI peripheral */
    exti_config();
    /* systick configuration */
    systick_config();
    /* NVIC configuration */
    nvic_config();
    /* configure COM port */
    gd_eval_com_init(EVAL_COM1);
    /* DMA configuration */
    dma_config();
    /* ADC configuration */
    adc_config();
    
    while(1){
        delay_1ms(1000);
        printf("\r\n ADC0 regular channel 0 data = %d \r\n",adc_value[0]);
        printf("\r\n ADC0 regular channel 1 data = %d \r\n",adc_value[1]);
        printf("\r\n ADC0 regular channel 2 data = %d \r\n",adc_value[2]);
        printf("\r\n ADC0 regular channel 3 data = %d \r\n",adc_value[3]);
        printf("\r\n ADC0 inserted channel 0 data = %d \r\n",inserted_data[0]);
        printf("\r\n ADC0 inserted channel 1 data = %d \r\n",inserted_data[1]);
        printf("\r\n ADC0 inserted channel 2 data = %d \r\n",inserted_data[2]);
        printf("\r\n ADC0 inserted channel 3 data = %d \r\n",inserted_data[3]);
        printf("\r\n ***********************************\r\n");
    }
}

/*!
    \brief      configure the different system clocks
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_config(void)
{
    /* enable GPIOA clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* enable ADC0 clock */
    rcu_periph_clock_enable(RCU_ADC0);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);
    /* enable DMA0 clock */
    rcu_periph_clock_enable(RCU_DMA0);
    /* enable alternate function clock */
    rcu_periph_clock_enable(RCU_AF);
}

/*!
    \brief      configure the GPIO peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void)
{
    /* config the GPIO as analog mode, for ADC */
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
    /* config the GPIO as floating input mode, for EXTI */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_15);
}

/**
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
  */
void nvic_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(ADC0_1_IRQn, 1, 1);
}

/*!
    \brief      configure the DMA peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void dma_config(void)
{
    /* ADC_DMA_channel configuration */
    dma_parameter_struct dma_data_parameter;
    
    /* ADC DMA_channel configuration */
    dma_deinit(DMA0, DMA_CH0);
    
    /* initialize DMA single data mode */
    dma_data_parameter.periph_addr  = (uint32_t)(&ADC_RDATA(ADC0));
    dma_data_parameter.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr  = (uint32_t)(&adc_value);
    dma_data_parameter.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;  
    dma_data_parameter.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number       = 4;
    dma_data_parameter.priority     = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH0, &dma_data_parameter);

    /* enable DMA circulation mode */
    dma_circulation_enable(DMA0, DMA_CH0);
  
    /* enable DMA channel */
    dma_channel_enable(DMA0, DMA_CH0);
}

/*!
    \brief      configure the ADC peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_config(void)
{
    /* reset ADC */
    adc_deinit(ADC0);
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE);
    /* ADC continous function enable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    
    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 4);
    /* ADC inserted channel config */
    adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_0, ADC_SAMPLETIME_55POINT5);
    adc_inserted_channel_config(ADC0, 1, ADC_CHANNEL_1, ADC_SAMPLETIME_55POINT5);
    adc_inserted_channel_config(ADC0, 2, ADC_CHANNEL_2, ADC_SAMPLETIME_55POINT5);
    adc_inserted_channel_config(ADC0, 3, ADC_CHANNEL_3, ADC_SAMPLETIME_55POINT5);
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_EXTI_15); 
    /* ADC external trigger enable */
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);
    /* clear the ADC flag */
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOC);
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
    /* enable ADC interrupt */
    adc_interrupt_enable(ADC0, ADC_INT_EOIC);
    
    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 4);
    /* ADC regular channel config */
    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_4, ADC_SAMPLETIME_55POINT5);
    adc_regular_channel_config(ADC0, 1, ADC_CHANNEL_5, ADC_SAMPLETIME_55POINT5);
    adc_regular_channel_config(ADC0, 2, ADC_CHANNEL_6, ADC_SAMPLETIME_55POINT5);
    adc_regular_channel_config(ADC0, 3, ADC_CHANNEL_7, ADC_SAMPLETIME_55POINT5);
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_EXTI_11);
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);
    /* ADC DMA function enable */
    adc_dma_mode_enable(ADC0);
    
    /* enable ADC interface */
    adc_enable(ADC0);
    delay_1ms(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
}

/*!
    \brief      configure the EXTI peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void exti_config(void)
{
    /* connect EXTI line to GPIO pin for inserted group */
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_15);
    /* configure EXTI line for inserted group */
    exti_init(EXTI_15, EXTI_EVENT, EXTI_TRIG_RISING);
    
    /* connect EXTI line to GPIO pin for inserted group */
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_11);
    /* configure EXTI line for inserted group */
    exti_init(EXTI_11, EXTI_EVENT, EXTI_TRIG_RISING);
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM1, (uint8_t)ch);
    while(RESET == usart_flag_get(EVAL_COM1, USART_FLAG_TBE));
    return ch;
}
