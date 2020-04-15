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
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "net/rpl/rpl.h"
//#include "apps/battery-control/battery_simulated.h"
//#include "apps/battery-collect-view/battery_collect_view.h"


#include "net/netstack.h"
#if PLATFORM_HAS_BUTTON
#include "dev/button-sensor.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "apps/serial-shell/serial-shell.h"
#include "apps/shell/shell-rime.h"
#include "apps/shell/shell-collect-view.h"
#include "dev/serial-line.h"

#include "collect-common.h"
#include "collect-view.h"

#if TSCH_TIME_SYNCH
#include "net/rpl/rpl-private.h"
#include "net/mac/4emac/4emac-private.h"
static uint8_t master_synched = 1;
#endif /* TSCH_TIME_SYNCH */

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

#define SERVER_REPLY 1

static struct uip_udp_conn *server_conn;

PROCESS(udp_server_process, "UDP server process");
AUTOSTART_PROCESSES(&udp_server_process, &collect_common_process);

#if 1
/*---------------------------------------------------------------------------*/
void
collect_common_set_sink(void)
{
}
/*---------------------------------------------------------------------------*/
void
collect_common_net_print(void)
{
  PRINTF("I am sink!\n");
}
/*---------------------------------------------------------------------------*/
void
collect_common_send(void)
{
  /* Server never sends */
}
/*---------------------------------------------------------------------------*/
void
collect_common_net_init(void)
{

  uart1_set_input(serial_line_input_byte);
  serial_line_init();

  PRINTF("I am sink!\n");
}
#endif

/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  #if 0
  char *appdata;

  if(uip_newdata()) {
    appdata = (char *)uip_appdata;
    appdata[uip_datalen()] = 0;
    PRINTF("DATA recv from ");
    PRINTF("%d",
           UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1]);
    PRINTF(" '%s'\n", appdata);
    //PRINTF("RSSI : %d\n", packetbuf_attr(PACKETBUF_ATTR_RSSI));
#if SERVER_REPLY
    PRINTF("DATA sending reply\n");
    uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
    uip_udp_packet_send(server_conn, "Reply", sizeof("Reply"));
    uip_create_unspecified(&server_conn->ripaddr);
#endif
  }
  #endif

  #if 1
  uint8_t *appdata;
  linkaddr_t sender;
  uint8_t seqno;
  uint8_t hops;

  if(uip_newdata()) {
    appdata = (uint8_t *)uip_appdata;
    
  
    appdata[uip_datalen()] = 0;
    //PRINTF("DATA recv from ");
    //PRINTF("%d",
    //       UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1]);
    //PRINTF(" '%s'\n", appdata);
    
    
    sender.u8[0] = UIP_IP_BUF->srcipaddr.u8[15];
    sender.u8[1] = UIP_IP_BUF->srcipaddr.u8[14];
    seqno = *appdata;
    hops = uip_ds6_if.cur_hop_limit - UIP_IP_BUF->ttl + 1;
    collect_common_recv(&sender, seqno, hops,
                        appdata + 2, uip_datalen() - 2);
  }
  #endif
}
#if 0
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Server IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(state == ADDR_TENTATIVE || state == ADDR_PREFERRED) {
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
PROCESS_THREAD(udp_server_process, ev, data)
{
  uip_ipaddr_t ipaddr;
  struct uip_ds6_addr *root_if;

  PROCESS_BEGIN();

  PROCESS_PAUSE();
  
  //battery_simulated_init();
  //battery_collect_view_init();
  //collect_process_start();

#if PLATFORM_HAS_BUTTON
  SENSORS_ACTIVATE(button_sensor);
#endif

  PRINTF("UDP server started\n");

#if UIP_CONF_ROUTER
/* The choice of server address determines its 6LoWPAN header compression.
 * Obviously the choice made here must also be selected in udp-client.c.
 *
 * For correct Wireshark decoding using a sniffer, add the /64 prefix to the
 * 6LowPAN protocol preferences,
 * e.g. set Context 0 to fd00::. At present Wireshark copies Context/128 and
 * then overwrites it.
 * (Setting Context 0 to fd00::1111:2222:3333:4444 will report a 16 bit
 * compressed address of fd00::1111:22ff:fe33:xxxx)
 * Note Wireshark's IPCMV6 checksum verification depends on the correct
 * uncompressed addresses.
 */
 
#if 1
/* Mode 1 - 64 bits inline */
   uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 5);
#elif 0
/* Mode 2 - 16 bits inline */
  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0x00ff, 0xfe00, 1);
#else
/* Mode 3 - derived from link local (MAC) address */
  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
#endif

  uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);
  root_if = uip_ds6_addr_lookup(&ipaddr);
  if(root_if != NULL) {
    rpl_dag_t *dag;
    dag = rpl_set_root(RPL_DEFAULT_INSTANCE,(uip_ip6addr_t *)&ipaddr);
    uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
    rpl_set_prefix(dag, &ipaddr, 64);
    PRINTF("%d %d ", ipaddr.u8[14],ipaddr.u8[15]);
    PRINTF("created a new RPL dag\n");
  } else {
    PRINTF("failed to create a new RPL DAG\n");
  }
#endif /* UIP_CONF_ROUTER */

#if TSCH_TIME_SYNCH
  foure_timesynch_init(master_synched); /* Inıtialize after rpl set root */
#endif /* TIMESYNCH_CONF_ENABLED */
  
  server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
  if(server_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));
#if 0
  PRINTF("Created a server connection with remote address ");
  PRINT6ADDR(&server_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n", UIP_HTONS(server_conn->lport),
         UIP_HTONS(server_conn->rport));
#endif

  //collect_process_start();

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
#if PLATFORM_HAS_BUTTON
    else if(ev == sensors_event && data == &button_sensor) {
      PRINTF("Initiaing global repair\n");
      rpl_repair_root(RPL_DEFAULT_INSTANCE);
    }
#endif
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
