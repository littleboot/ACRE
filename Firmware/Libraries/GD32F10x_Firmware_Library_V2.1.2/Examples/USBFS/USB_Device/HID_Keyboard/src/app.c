/*!
    \file  main.c
    \brief USB main routine for HID device(USB keyboard)

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

#include  "hid_core.h"
#include  "usb_delay.h"

typedef enum
{
    CHAR_A = 1,
    CHAR_B,
    CHAR_C
}key_char_enum;

uint8_t key_buffer[HID_IN_PACKET] = {0};

__IO uint8_t prev_transfer_complete = 1U;

uint8_t timer_prescaler = 0;
uint32_t usbfs_prescaler = 0;

usb_core_handle_struct usbhs_core_dev = 
{
    .dev = 
    {
        .dev_desc = (uint8_t *)&device_descripter,
        .config_desc = (uint8_t *)&configuration_descriptor,
        .strings = usbd_strings,
        .class_init = usbd_hid_init,
        .class_deinit = usbd_hid_deinit,
        .class_req_handler = usbd_hid_classreq_handle,
        .class_data_handler = usbd_hid_data_handler,
    },

    .udelay = delay_us,
    .mdelay = delay_ms
};

void usb_clock_config(void);
void usb_gpio_config(void);
void usb_interrupt_config(void);

void key_config(void);
uint8_t key_state(void);

/*!
    \brief      main routine will construct a USB keyboard
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* configure USB clock */
    usb_clock_config();

    /* timer nvic initialization */
    timer_nvic_init();

    /* configure key */
    key_config();

    /* USB device stack configure */
    usbd_init(&usbhs_core_dev, USB_FS_CORE_ID);

    /* USB interrupt configure */
    usb_interrupt_config();

    /* check if USB device is enumerated successfully */
    while (usbhs_core_dev.dev.status != USB_STATUS_CONFIGURED) {
    }

    while (1) {
        if (prev_transfer_complete) {
            switch (key_state()) {
            case CHAR_A:
                key_buffer[2] = 0x04U;
                break;
            case CHAR_B:
                key_buffer[2] = 0x05U;
                break;
            case CHAR_C:
                key_buffer[2] = 0x06U;
                break;
            default:
                break;
            }

            if (key_buffer[2] != 0U) {
                usbd_hid_report_send (&usbhs_core_dev, key_buffer, 8U);
            }
        }
    }
}

/*!
    \brief      configure USB clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_clock_config(void)
{
    uint32_t system_clock = rcu_clock_freq_get(CK_SYS);
  
    if (system_clock == 48000000) {
        usbfs_prescaler = RCU_CKUSB_CKPLL_DIV1;
        timer_prescaler = 3;
    } else if (system_clock == 72000000) {
        usbfs_prescaler = RCU_CKUSB_CKPLL_DIV1_5;
        timer_prescaler = 5;
    } else if (system_clock == 96000000) {
        usbfs_prescaler = RCU_CKUSB_CKPLL_DIV2;
        timer_prescaler = 7;
    } else if (system_clock == 120000000) {
        usbfs_prescaler = RCU_CKUSB_CKPLL_DIV2_5;
        timer_prescaler = 9;
    }  else {
        /*  reserved  */
    }

    rcu_usb_clock_config(usbfs_prescaler);
    rcu_periph_clock_enable(RCU_USBFS);
}

/*!
    \brief      configure USB interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_interrupt_config(void)
{
    nvic_irq_enable((uint8_t)USBFS_IRQn, 3U, 0U);

    /* enable the power module clock */
    rcu_periph_clock_enable(RCU_PMU);

    /* USB wakeup EXTI line configuration */
    exti_interrupt_flag_clear(EXTI_18);
    exti_init(EXTI_18, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_enable(EXTI_18);

    nvic_irq_enable((uint8_t)USBFS_WKUP_IRQn, 0U, 0U);
}

/*!
    \brief      configure key
    \param[in]  none
    \param[out] none
    \retval     none
*/
void key_config(void)
{
    /* configure the wakeup key in exti mode to remote wakeup */
    gd_eval_key_init(KEY_WAKEUP, KEY_MODE_EXTI);

    gd_eval_key_init(KEY_TAMPER, KEY_MODE_GPIO);
    gd_eval_key_init(KEY_USER, KEY_MODE_GPIO);

    exti_interrupt_flag_clear(WAKEUP_KEY_EXTI_LINE);
}

/*!
    \brief      to get usb keyboard state
    \param[in]  none
    \param[out] none
    \retval     the char
*/
uint8_t key_state (void)
{
    /* have pressed tamper key */
    if (!gd_eval_key_state_get(KEY_TAMPER)) {
        delay_ms(100U);

        if (!gd_eval_key_state_get(KEY_TAMPER)) {
            return CHAR_A;
        }
    }

    /* have pressed wakeup key */
    if (!gd_eval_key_state_get(KEY_WAKEUP)) {
        delay_ms(100U);

        if (!gd_eval_key_state_get(KEY_WAKEUP)) {
            return CHAR_B;
        }
    }

    /* have pressed user key */
    if (!gd_eval_key_state_get(KEY_USER)) {
        delay_ms(100U);

        if (!gd_eval_key_state_get(KEY_USER)) {
            return CHAR_C;
        }
    }

    /* no pressed any key */
    return 0U;
}
