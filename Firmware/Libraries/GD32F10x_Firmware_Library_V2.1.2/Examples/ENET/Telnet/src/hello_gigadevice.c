/*!
    \file    hello_gigadevice.c
    \brief   TCP server demo program 

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

#include "hello_gigadevice.h"  
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>
#include "gd32f10x.h"

#define GREETING         "\n\r======= HelloGigaDevice =======\
                          \n\r== GD32 ==\
                          \n\r== Telnet SUCCESS==\
                          \n\rHello. What is your name?\r\n"
#define HELLO            "\n\rGigaDevice¡¾23¡¿PORT Hello "
#define MAX_NAME_SIZE    32

extern const uint8_t gd32_str[];
struct name 
{
    int length;
    char bytes[MAX_NAME_SIZE];
};

static err_t hello_gigadevice_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
static err_t hello_gigadevice_accept(void *arg, struct tcp_pcb *pcb, err_t err);
static void hello_gigadevice_conn_err(void *arg, err_t err);

/*!
    \brief      called when a data is received on the telnet connection 
    \param[in]  arg: the user argument
    \param[in]  pcb: the tcp_pcb that has received the data
    \param[in]  p: the packet buffer
    \param[in]  err: the error value linked with the received data
    \param[out] none
    \retval     err_t: error value
*/ 
static err_t hello_gigadevice_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    struct pbuf *q;
    struct name *name = (struct name *)arg;
    int done;
    char *c;
    int i;

    /* we perform here any necessary processing on the pbuf */
    if (p != NULL){    
        /* we call this function to tell the LwIp that we have processed the data */
        /* this lets the stack advertise a larger window, so more data can be received*/
        tcp_recved(pcb, p->tot_len);

        /* check the name if NULL, no data passed, return with illegal argument error */
        if(!name){ 
            pbuf_free(p);
            return ERR_ARG;
        }

        done = 0;
        for(q=p; q != NULL; q = q->next){ 
            c = q->payload;
            for(i=0; i<q->len && !done; i++){ 
                done = ((c[i] == '\r') || (c[i] == '\n'));
                if(name->length < MAX_NAME_SIZE){ 
                    name->bytes[name->length++] = c[i];
                }
            } 
        }
        
        if(done){ 
            if(name->bytes[name->length-2] != '\r' || name->bytes[name->length-1] != '\n'){ 
                if((name->bytes[name->length-1] == '\r' || name->bytes[name->length-1] == '\n') && (name->length+1 <= MAX_NAME_SIZE)){ 
                    name->length += 1;
                }else if(name->length+2 <= MAX_NAME_SIZE){ 
                    name->length += 2;
                }else{  
                    name->length = MAX_NAME_SIZE;
                }

                name->bytes[name->length-2] = '\r';
                name->bytes[name->length-1] = '\n';
            }
            
            tcp_write(pcb, HELLO, strlen(HELLO), 1);
            tcp_write(pcb, name->bytes, name->length, TCP_WRITE_FLAG_COPY);
            printf("\n\rGigaDevice\n\rTelnet %s %s", HELLO, name->bytes);
            name->length = 0;
        }
        /* end of processing, we free the pbuf */
        pbuf_free(p);
        
    }else if (err == ERR_OK){  
        /* when the pbuf is NULL and the err is ERR_OK, the remote end is closing the connection. */
        /* we free the allocated memory and we close the connection */
        mem_free(name);
        return tcp_close(pcb);
    }
    return ERR_OK;
}

/*!
    \brief      this function when the Telnet connection is established 
    \param[in]  arg: user supplied argument
    \param[in]  pcb: the tcp_pcb which accepted the connection
    \param[in]  err: error value
    \param[out] none
    \retval     err_t: error value
*/
static err_t hello_gigadevice_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{     
    u32_t ipaddress;
    u8_t iptxt[50];
    volatile u8_t iptab[4];
  
    ipaddress = pcb->remote_ip.addr;
    printf("\n\rTelnet hello_gigadevice_accept:%d.%d.%d.%d  %s", 
          (u8_t)(ipaddress),(u8_t)(ipaddress >> 8),(u8_t)(ipaddress >> 16),(u8_t)(ipaddress >> 24),GREETING);
   
    /* read its IP address */
    iptab[0] = (u8_t)(ipaddress >> 24);
    iptab[1] = (u8_t)(ipaddress >> 16);
    iptab[2] = (u8_t)(ipaddress >> 8);
    iptab[3] = (u8_t)(ipaddress);

    sprintf((char*)iptxt, "Telnet:%d.%d.%d.%d   ", iptab[3], iptab[2], iptab[1], iptab[0]);
    printf("%s\r\n",iptxt);
    /* tell LwIP to associate this structure with this connection. */
    tcp_arg(pcb, mem_calloc(sizeof(struct name), 1));
  
    /* configure LwIP to use our call back functions. */
    tcp_err(pcb, hello_gigadevice_conn_err);
    tcp_recv(pcb, hello_gigadevice_recv);
    
    /* send out the first message */  
    tcp_write(pcb, iptxt, strlen((char *)iptxt), 1); 
    sprintf((char*)iptxt, "Your telnet computer's IP is: %d.%d.%d.%d\n", iptab[3], iptab[2], iptab[1], iptab[0]);
    printf("%s\r\n",iptxt);
    tcp_write(pcb, gd32_str, strlen((char *)gd32_str), 1); 
    tcp_write(pcb, GREETING, strlen(GREETING), 1); 
  
    return ERR_OK;
}

/*!
    \brief      initialize the hello application
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hello_gigadevice_init(void)
{
    struct tcp_pcb *pcb;
  
    /* create a new TCP control block  */
    pcb = tcp_new();

    /* assign to the new pcb a local IP address and a port number */
    /* using IP_ADDR_ANY allow the pcb to be used by any local interface */
    tcp_bind(pcb, IP_ADDR_ANY, 23);       

    /* set the connection to the LISTEN state */
    pcb = tcp_listen(pcb);

    /* Specify the function to be called when a connection is established */    
    tcp_accept(pcb, hello_gigadevice_accept);   
}

/*!
    \brief      this function is called when an error occurs on the connection
    \param[in]  arg: user supplied argument
    \param[in]  err: error value
    \param[out] none
    \retval     none
*/
static void hello_gigadevice_conn_err(void *arg, err_t err)
{
    struct name *name;
    name = (struct name *)arg;

    mem_free(name);
}
