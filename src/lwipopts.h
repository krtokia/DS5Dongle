#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// Minimal lwIP for one purpose: receive small UDP trigger datagrams, plus the
// DHCP exchange needed to get an address. Nothing is sent, nothing is streamed.
//
// Every byte here is a byte of heap this firmware does not get. The Bluetooth
// audio path allocates heavily and the stock build already runs close to the
// limit, so the pools are sized for the traffic that actually arrives rather
// than for a general-purpose stack.

#define NO_SYS                      1
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0

#define MEM_LIBC_MALLOC             0
#define MEM_ALIGNMENT               4
// Only has to hold the odd outgoing DHCP message.
#define MEM_SIZE                    1600

#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1
#define LWIP_RAW                    0
#define LWIP_UDP                    1
#define LWIP_TCP                    0
#define LWIP_DHCP                   1
#define LWIP_DNS                    0
#define LWIP_IPV4                   1
#define LWIP_IPV6                   0
#define LWIP_IGMP                   0
#define LWIP_STATS                  0
#define LWIP_STATS_DISPLAY          0
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_NETIF_STATUS_CALLBACK  0
#define LWIP_NETIF_LINK_CALLBACK    0

// Received frames are chained out of the pool, so the buffers can be far
// smaller than an MTU as long as there are enough of them for one frame plus
// a little slack.
#define PBUF_POOL_SIZE              6
#define PBUF_POOL_BUFSIZE           512

#define MEMP_NUM_PBUF               6
#define MEMP_NUM_UDP_PCB            3
#define MEMP_NUM_ARP_QUEUE          2
#define MEMP_NUM_SYS_TIMEOUT        8
#define MEMP_NUM_NETBUF             0
#define MEMP_NUM_NETCONN            0

#define DHCP_DOES_ARP_CHECK         0
#define LWIP_DHCP_DOES_ACD_CHECK    0

#define LWIP_CHKSUM_ALGORITHM       3

#ifndef NDEBUG
#define LWIP_DEBUG                  0
#endif

#endif // _LWIPOPTS_H
