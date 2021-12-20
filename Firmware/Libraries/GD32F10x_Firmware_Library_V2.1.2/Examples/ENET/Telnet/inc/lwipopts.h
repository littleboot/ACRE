/*!
    \file    lwipopts.h
    \brief   LwIP options configuration 

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

#ifndef LWIPOPTS_H
#define LWIPOPTS_H


#define SYS_LIGHTWEIGHT_PROT    0                        /* SYS_LIGHTWEIGHT_PROT==1: if you want inter-task protection 
                                                            for certain critical regions during buffer allocation,
                                                            deallocation and memory allocation and deallocation */                                                            

#define NO_SYS                  1                        /* NO_SYS==1: provides VERY minimal functionality. 
                                                            Otherwise, use lwIP facilities */

/*  memory options  */
#define MEM_ALIGNMENT           4                        /* should be set to the alignment of the CPU for which lwIP
                                                            is compiled. 4 byte alignment -> define MEM_ALIGNMENT 
                                                            to 4, 2 byte alignment -> define MEM_ALIGNMENT to 2 */

#define MEM_SIZE                (20*1024)                /* the size of the heap memory, if the application will 
                                                            send a lot of data that needs to be copied, this should
                                                            be set high */

#define MEMP_NUM_PBUF           10                       /* the number of memp struct pbufs. If the application
                                                            sends a lot of data out of ROM (or other static memory),
                                                            this should be set high */

#define MEMP_NUM_UDP_PCB        6                        /* the number of UDP protocol control blocks, one
                                                            per active UDP "connection" */

#define MEMP_NUM_TCP_PCB        10                       /* the number of simulatenously active TCP connections */

#define MEMP_NUM_TCP_PCB_LISTEN 6                        /* the number of listening TCP connections */

#define MEMP_NUM_TCP_SEG        12                       /* the number of simultaneously queued TCP segments */

#define MEMP_NUM_SYS_TIMEOUT    10                        /* the number of simulateously active timeouts */


/* Pbuf options */
#define PBUF_POOL_SIZE          10                       /* the number of buffers in the pbuf pool */
#define PBUF_POOL_BUFSIZE       1500                     /* the size of each pbuf in the pbuf pool */

/* TCP options */
#define LWIP_TCP                1
#define TCP_TTL                 255

#define TCP_QUEUE_OOSEQ         0                        /* controls if TCP should queue segments that arrive out of
                                                            order, Define to 0 if your device is low on memory. */

#define TCP_MSS                 (1500 - 40)              /* TCP Maximum segment size, 
                                                            TCP_MSS = (Ethernet MTU - IP header size - TCP header size) */

#define TCP_SND_BUF             (2*TCP_MSS)              /* TCP sender buffer space (bytes) */

#define TCP_SND_QUEUELEN        ((6* TCP_SND_BUF)/TCP_MSS)   /* TCP sender buffer space (pbufs), this must be at least
                                                            as much as (2 * TCP_SND_BUF/TCP_MSS) for things to work */

#define TCP_WND                 (2*TCP_MSS)              /* TCP receive window */
                                                   

/* ICMP options */
#define LWIP_ICMP               1


/* DHCP options */
#define LWIP_DHCP               1                        /* define to 1 if you want DHCP configuration of interfaces,
                                                            DHCP is not implemented in lwIP 0.5.1, however, so
                                                            turning this on does currently not work. */

/* UDP options */
#define LWIP_UDP                1
#define UDP_TTL                 255


/* statistics options */
#define LWIP_STATS              0
#define LWIP_PROVIDE_ERRNO      1

/* checksum options */
#define CHECKSUM_BY_HARDWARE                             /* computing and verifying the IP, UDP, TCP and ICMP
                                                            checksums by hardware */

/* sequential layer options */
#define LWIP_NETCONN            0                        /* set to 1 to enable netconn API (require to use api_lib.c) */

/* socket options */
#define LWIP_SOCKET             0                        /* set to 1 to enable socket API (require to use sockets.c) */

/* Lwip debug options */
//#define LWIP_DEBUG            1



#ifdef CHECKSUM_BY_HARDWARE
    /* CHECKSUM_GEN_IP==0: generate checksums by hardware for outgoing IP packets.*/
    #define CHECKSUM_GEN_IP                 0
    /* CHECKSUM_GEN_UDP==0: generate checksums by hardware for outgoing UDP packets.*/
    #define CHECKSUM_GEN_UDP                0
    /* CHECKSUM_GEN_TCP==0: generate checksums by hardware for outgoing TCP packets.*/
    #define CHECKSUM_GEN_TCP                0 
    /* CHECKSUM_CHECK_IP==0: check checksums by hardware for incoming IP packets.*/
    #define CHECKSUM_CHECK_IP               0
    /* CHECKSUM_CHECK_UDP==0: check checksums by hardware for incoming UDP packets.*/
    #define CHECKSUM_CHECK_UDP              0
    /* CHECKSUM_CHECK_TCP==0: check checksums by hardware for incoming TCP packets.*/
    #define CHECKSUM_CHECK_TCP              0
    #define CHECKSUM_GEN_ICMP               0
#else
    /* CHECKSUM_GEN_IP==1: generate checksums in software for outgoing IP packets.*/
    #define CHECKSUM_GEN_IP                 1
    /* CHECKSUM_GEN_UDP==1: generate checksums in software for outgoing UDP packets.*/
    #define CHECKSUM_GEN_UDP                1
    /* CHECKSUM_GEN_TCP==1: generate checksums in software for outgoing TCP packets.*/
    #define CHECKSUM_GEN_TCP                1
    /* CHECKSUM_CHECK_IP==1: check checksums in software for incoming IP packets.*/
    #define CHECKSUM_CHECK_IP               1
    /* CHECKSUM_CHECK_UDP==1: check checksums in software for incoming UDP packets.*/
    #define CHECKSUM_CHECK_UDP              1
    /* CHECKSUM_CHECK_TCP==1: check checksums in software for incoming TCP packets.*/
    #define CHECKSUM_CHECK_TCP              1
    #define CHECKSUM_GEN_ICMP               1
#endif

#endif /* LWIPOPTS_H */
