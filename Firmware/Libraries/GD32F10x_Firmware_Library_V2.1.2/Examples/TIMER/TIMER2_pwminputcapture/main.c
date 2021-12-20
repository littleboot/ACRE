/*!
    \file  main.c
    \brief TIMER2 PWM input capture demo

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

void gpio_configuration(void);
void timer_configuration(void);
void nvic_configuration(void);
int fputc(int ch, FILE *f);

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM1, (uint8_t) ch);
    while (usart_flag_get(EVAL_COM1,USART_FLAG_TBE) == RESET);
    return ch;
}

/**
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
  */
void gpio_configuration(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);

    /*configure PA6 (TIMER2 CH0) as alternate function*/
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
}

/**
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
  */
void nvic_configuration(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER2_IRQn, 1, 1);
}

/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void timer_configuration(void)
{
 /* TIMER2 configuration: PWM input mode ------------------------
     the external signal is connected to TIMER2 CH0 pin(PA6)
     the rising edge is used as active edge
     the TIMER2 CH0CV is used to compute the frequency value 
     the TIMER2 CH1CV is used to compute the duty cycle value
  ------------------------------------------------------------ */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER2);

    timer_deinit(TIMER2);

    /* TIMER2 configuration */
    timer_initpara.prescaler         = 107;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 65535;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2,&timer_initpara);

    /* TIMER2 configuration */
    /* TIMER2 CH0 PWM input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x0;
    timer_input_pwm_capture_config(TIMER2,TIMER_CH_0,&timer_icinitpara);

    /* slave mode selection: TIMER2 */
    timer_input_trigger_source_select(TIMER2,TIMER_SMCFG_TRGSEL_CI0FE0);
    timer_slave_mode_select(TIMER2,TIMER_SLAVE_MODE_RESTART);

    /* select the master slave mode */
    timer_master_slave_mode_config(TIMER2,TIMER_MASTER_SLAVE_MODE_ENABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER2);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER2,TIMER_INT_CH0);
    /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER2,TIMER_INT_CH0);

    /* TIMER2 counter enable */
    timer_enable(TIMER2);
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    gpio_configuration(); 
    gd_eval_com_init(EVAL_COM1);
    nvic_configuration();
    timer_configuration();

    while (1);
}
