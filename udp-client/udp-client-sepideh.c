/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "sys/ctimer.h"
#ifdef WITH_COMPOWER
#include "powertrace.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include "apps/battery-control/battery_simulated.h"
//#include "apps/battery-collect/collect-common.h"
//#include "apps/battery-collect-view/battery_collect_view.h"
//#include "apps/sepideh/sepideh.h"


#include "apps/serial-shell/serial-shell.h"
#include "apps/shell/shell-rime.h"

#include "collect-common.h"
#include "collect-view.h"

#include "dev/serial-line.h"
#include "net/ipv6/uip-ds6-route.h"

#if TSCH_TIME_SYNCH
#include "net/rpl/rpl-private.h"
#include "net/mac/4emac/4emac-private.h"
#include "net/mac/4emac/4emac-buf.h"
#endif /* TSCH_TIME_SYNCH */

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#ifndef PERIOD
#define PERIOD  10
#endif

#define SEND_INTERVAL     (PERIOD * CLOCK_SECOND) >> 0
#define SEND_TIME         (random_rand() % (SEND_INTERVAL))
#define MAX_PAYLOAD_LEN   64

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process , &collect_common_process);
/*---------------------------------------------------------------------------*/
static int seq_id;
static int reply;

#if 1
/*---------------------------------------------------------------------------*/
void
collect_common_set_sink(void)
{
  /* A udp client can never become sink */
}

/*---------------------------------------------------------------------------*/
void
collect_common_net_print(void)
{
  rpl_dag_t *dag;
  uip_ds6_route_t *r;

  /* Let's suppose we have only one instance */
  dag = rpl_get_any_dag();
  if(dag->preferred_parent != NULL) {
    PRINTF("Preferred parent: ");
    PRINT6ADDR(rpl_get_parent_ipaddr(dag->preferred_parent));
    PRINTF("\n");
  }
  for(r = uip_ds6_route_head();
      r != NULL;
      r = uip_ds6_route_next(r)) {
    PRINT6ADDR(&r->ipaddr);
  }
  PRINTF("---\n");
}

/*---------------------------------------------------------------------------*/
void
collect_common_net_init(void)
{
  uart1_set_input(serial_line_input_byte);
  serial_line_init();
}

/*---------------------------------------------------------------------------*/
void
collect_common_send(void)
{
  static uint8_t seqno;
  struct {
    uint8_t seqno;
    uint8_t for_alignment;
    struct collect_view_data_msg msg;
  } msg;
  /* struct collect_neighbor *n; */
  uint16_t parent_etx;
  uint16_t rtmetric;
  uint16_t num_neighbors;
  uint16_t beacon_interval;
  rpl_parent_t *preferred_parent;
  linkaddr_t parent;
  rpl_dag_t *dag;

  if(client_conn == NULL) {
    /* Not setup yet */
    return;
  }
  memset(&msg, 0, sizeof(msg));
  seqno++;
  if(seqno == 0) {
    /* Wrap to 128 to identify restarts */
    seqno = 128;
  }
  msg.seqno = seqno;

  linkaddr_copy(&parent, &linkaddr_null);
  parent_etx = 0;

  /* Let's suppose we have only one instance */
  dag = rpl_get_any_dag();
  if(dag != NULL) {
    preferred_parent = dag->preferred_parent;
    if(preferred_parent != NULL) {
      uip_ds6_nbr_t *nbr;
      nbr = uip_ds6_nbr_lookup(rpl_get_parent_ipaddr(preferred_parent));
      if(nbr != NULL) {
        /* Use parts of the IPv6 address as the parent address, in reversed byte order. */
        parent.u8[LINKADDR_SIZE - 1] = nbr->ipaddr.u8[sizeof(uip_ipaddr_t) - 2];
        parent.u8[LINKADDR_SIZE - 2] = nbr->ipaddr.u8[sizeof(uip_ipaddr_t) - 1];
        parent_etx = rpl_get_parent_rank((uip_lladdr_t *) uip_ds6_nbr_get_ll(nbr)) / 2;
      }
    }
    rtmetric = dag->rank;
    beacon_interval = (uint16_t) ((2L << dag->instance->dio_intcurrent) / 1000);
    num_neighbors = uip_ds6_nbr_num();
  } else {
    rtmetric = 0;
    beacon_interval = 0;
    num_neighbors = 0;
  }

  /* num_neighbors = collect_neighbor_list_num(&tc.neighbor_list); */
  collect_view_construct_message(&msg.msg, &parent,
                                 parent_etx, rtmetric,
                                 num_neighbors, beacon_interval);

  uip_udp_packet_sendto(client_conn, &msg, sizeof(msg),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
}
#endif

/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  char *str;

#if 0
  if(uip_newdata()) {
    str = uip_appdata;
    str[uip_datalen()] = '\0';
    reply++;
    printf("DATA recv '%s' (s:%d, r:%d)\n", str, seq_id, reply);
  }
  #endif
}
/*---------------------------------------------------------------------------*/
static void
send_packet(void *ptr)
{
  char buf[MAX_PAYLOAD_LEN], buf_len = 0;
#if 0
#ifdef SERVER_REPLY
  uint8_t num_used = 0;
  uip_ds6_nbr_t *nbr;

  nbr = nbr_table_head(ds6_neighbors);
  while(nbr != NULL) {
    nbr = nbr_table_next(ds6_neighbors, nbr);
    num_used++;
  }

  if(seq_id > 0) {
    ANNOTATE("#A r=%d/%d,color=%s,n=%d %d\n", reply, seq_id,
             reply == seq_id ? "GREEN" : "RED", uip_ds6_route_num_routes(), num_used);
  }
#endif /* SERVER_REPLY */

  seq_id++;
  PRINTF("DATA send to %d 'Hello %d'\n",
         server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1], seq_id);

  sprintf(buf, "Hello %d from the client", seq_id);
  buf_len += strlen((char *)buf);

  uip_udp_packet_sendto(client_conn, buf, buf_len, &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
#endif
}
#if 0
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
	uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
#endif
/*---------------------------------------------------------------------------*/
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

/* The choice of server address determines its 6LoWPAN header compression.
 * (Our address will be compressed Mode 3 since it is derived from our
 * link-local address)
 * Obviously the choice made here must also be selected in udp-server.c.
 *
 * For correct Wireshark decoding using a sniffer, add the /64 prefix to the
 * 6LowPAN protocol preferences,
 * e.g. set Context 0 to fd00::. At present Wireshark copies Context/128 and
 * then overwrites it.
 * (Setting Context 0 to fd00::1111:2222:3333:4444 will report a 16 bit
 * compressed address of fd00::1111:22ff:fe33:xxxx)
 *
 * Note the IPCMV6 checksum verification depends on the correct uncompressed
 * addresses.
 */
 
#if 1
/* Mode 1 - 64 bits inline */
   uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 5);
#elif 0
/* Mode 2 - 16 bits inline */
  uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0x00ff, 0xfe00, 1);
#else
/* Mode 3 - derived from server link-local (MAC) address */
  uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0x0250, 0xc2ff, 0xfea8, 0xcd1a); //redbee-econotag
#endif
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer periodic;
#if !TSCH_TIME_SYNCH
  static struct ctimer backoff_timer;
#endif
#if WITH_COMPOWER
  static int print = 0;
#endif

  PROCESS_BEGIN();

  //battery_simulated_init();
  //battery_collect_view_init();
  //collect_process_start();
  
  PROCESS_PAUSE();

  set_global_address();

  PRINTF("UDP client process started\n");

  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL); 
  if(client_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT)); 
#if 0
  PRINTF("Created a connection with the server ");
  PRINT6ADDR(&client_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
	UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));
#endif
#if WITH_COMPOWER
  powertrace_sniff(POWERTRACE_ON);
#endif

#if TSCH_TIME_SYNCH
  foure_timesynch_init(0);
#endif /* TIMESYNCH_CONF_ENABLED */

  //collect_process_start();
  
  //example_init();

  etimer_set(&periodic, SEND_INTERVAL);
  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
    #if 0
    if(ev == serial_line_event_message && data != NULL) {
      char *str;
      str = data;
      if(str[0] == 'r') {
        uip_ds6_route_t *r;
        uip_ipaddr_t *nexthop;
        uip_ds6_defrt_t *defrt;
        uip_ipaddr_t *ipaddr;
        defrt = NULL;
        if((ipaddr = uip_ds6_defrt_choose()) != NULL) {
          defrt = uip_ds6_defrt_lookup(ipaddr);
        }
        if(defrt != NULL) {
          PRINTF("DefRT: :: -> %02d", defrt->ipaddr.u8[15]);
          PRINTF(" lt:%lu inf:%d\n", stimer_remaining(&defrt->lifetime),
                 defrt->isinfinite);
        } else {
          PRINTF("DefRT: :: -> NULL\n");
        }

        for(r = uip_ds6_route_head();
            r != NULL;
            r = uip_ds6_route_next(r)) {
          nexthop = uip_ds6_route_nexthop(r);
          PRINTF("Route: %02d -> %02d", r->ipaddr.u8[15], nexthop->u8[15]);
          /* PRINT6ADDR(&r->ipaddr); */
          /* PRINTF(" -> "); */
          /* PRINT6ADDR(nexthop); */
          PRINTF(" lt:%lu\n", r->state.lifetime);

        }
      }else if(str[0] == 'p'){
        int peroid = atoi(&str[1]);
        if(peroid > (CLOCK_SECOND >> 5)){ /* 31.25 ms */
          PRINTF("Period %u\n", peroid);
          etimer_set(&periodic, peroid);
        }
      }
    }
    #endif

    #if 1
    if(etimer_expired(&periodic)) {
      etimer_reset(&periodic);
#if TSCH_TIME_SYNCH
      if(foure_control.synched && foure_control.authenticated && default_instance != NULL) {
        send_packet(NULL);
      } else
        PRINTF("Default instance is null\n");
#else
      //ctimer_set(&backoff_timer, SEND_TIME, send_packet, NULL);
#endif

    }
    #endif
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#ifdef UIP_CALLBACK_FORWARDING
#define UIP_IP_BUF      ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_EXT_BUF ((struct uip_udp_hdr *)&uip_buf[UIP_LLH_LEN + UIP_IPH_LEN + uip_ext_len])
#define PAYLOAD_START   (UIP_LLH_LEN + UIP_IPH_LEN + uip_ext_len + UIP_UDPH_LEN)

void
uip_add_route(void)
{
  if(uip_buf[PAYLOAD_START] == '#' && uip_buf[PAYLOAD_START + 1] == '!'){
    if(uip_buf[uip_len - 3] == 0 && uip_buf[uip_len - 2] == 0){
      uint16_t rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
      uint16_t asn = ((uint32_t)packetbuf_attr(PACKETBUF_ATTR_ASN_LSB_2_3) & 0xffff) | ((uint32_t)packetbuf_attr(PACKETBUF_ATTR_ASN_LSB_0_1) << 16);
      memcpy(&uip_buf[uip_len - 5], &rssi, sizeof(uint16_t));
      memcpy(&uip_buf[uip_len - 3], &asn, sizeof(uint16_t));
    }
    uip_buf[uip_len++] = uip_lladdr.addr[7];

    UIP_IP_BUF->len[0] = ((uip_len - UIP_IPH_LEN) >> 8);
    UIP_IP_BUF->len[1] = ((uip_len - UIP_IPH_LEN) & 0xff);

    UIP_UDP_EXT_BUF->udplen = UIP_HTONS(uip_len - UIP_IPH_LEN - uip_ext_len);
    UIP_UDP_EXT_BUF->udpchksum = 0;
#if UIP_UDP_CHECKSUMS
    /* Calculate New UDP checksum. */
    UIP_UDP_EXT_BUF->udpchksum = ~(uip_udpchksum());
    if(UIP_UDP_EXT_BUF->udpchksum == 0) {
      UIP_UDP_EXT_BUF->udpchksum = 0xffff;
    }
#endif /* UIP_UDP_CHECKSUMS */
  }
}
#endif /* UIP_CALLBACK_FORWARDING */
/*---------------------------------------------------------------------------*/
