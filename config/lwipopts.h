#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

/* Common lwIP configuration for EchoGuard Pico W */

#define NO_SYS                      0
#define SYS_LIGHTWEIGHT_PROT        1

#define MEM_ALIGNMENT               4
#define MEM_SIZE                    4000

#define MEMP_NUM_PBUF               16
#define MEMP_NUM_UDP_PCB            6
#define MEMP_NUM_TCP_PCB            5
#define MEMP_NUM_TCP_PCB_LISTEN     1
#define MEMP_NUM_SYS_TIMEOUT        10

#define PBUF_POOL_SIZE              16
#define PBUF_POOL_BUFSIZE           256

#define LWIP_DHCP                   0
#define LWIP_LINK_POLL_INTERVAL     1
#define ETH_PAD_SIZE                2

#define LWIP_TCP                    0
#define LWIP_UDP                    0
#define LWIP_DNS                    0
#define LWIP_SOCKET                 0

#define TCPIP_THREAD_NAME            "LWIP"
#define TCPIP_THREAD_STACKSIZE       1024
#define TCPIP_THREAD_PRIO            3

#define DEFAULT_THREAD_STACKSIZE     1024
#define DEFAULT_THREAD_PRIO          3
#define DEFAULT_RAW_RECVMBOX_SIZE    4
#define DEFAULT_UDP_RECVMBOX_SIZE    4
#define DEFAULT_TCP_RECVMBOX_SIZE    4
#define DEFAULT_ACCEPTMBOX_SIZE      4

#endif /* _LWIPOPTS_H */
