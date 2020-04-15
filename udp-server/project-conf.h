/*
 * Copyright (c) 2015, Swedish Institute of Computer Science.
 * All rights reserved.
 *
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
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

#ifdef APP_CONF_H
#include APP_CONF_H
#endif /* APP_CONF_H */

#define IPV6_RPL_UDP 1

#ifndef WITH_NON_STORING
#define WITH_NON_STORING 0 /* Set this to run with non-storing mode */
#endif /* WITH_NON_STORING */

/* Define as minutes */
#define RPL_CONF_DEFAULT_LIFETIME_UNIT   60

/* 10 minutes lifetime of routes */
#define RPL_CONF_DEFAULT_LIFETIME        10

#define RPL_CONF_DEFAULT_ROUTE_INFINITE_LIFETIME 1

#if WITH_NON_STORING
#undef RPL_NS_CONF_LINK_NUM
#define RPL_NS_CONF_LINK_NUM 40 /* Number of links maintained at the root. Can be set to 0 at non-root nodes. */
#undef UIP_CONF_MAX_ROUTES
#define UIP_CONF_MAX_ROUTES 0 /* No need for routes */
#undef RPL_CONF_MOP
#define RPL_CONF_MOP RPL_MOP_NON_STORING /* Mode of operation*/
#endif /* WITH_NON_STORING */

#define FOURE_DEBUG DEBUG_PRINT

/* Select CC2538 devices
#define CC2538_DEV_CC2538SF53   CC2538_DEV_DEF(0, 512, 32, 1, 1)
#define CC2538_DEV_CC2538SF23   CC2538_DEV_DEF(1, 256, 32, 1, 1)
#define CC2538_DEV_CC2538NF53   CC2538_DEV_DEF(2, 512, 32, 1, 0)
#define CC2538_DEV_CC2538NF23   CC2538_DEV_DEF(3, 256, 32, 1, 0)
#define CC2538_DEV_CC2538NF11   CC2538_DEV_DEF(4, 128, 16, 1, 0)
*/
#define CC2538_DEV_CONF         CC2538_DEV_CC2538NF53

#define UART0_CONF_BAUD_RATE    921600

#define TSCH_TIME_SYNCH         1

#define FRAME802154_CONF_VERSION    FRAME802154_IEEE802154E_2012

#if TSCH_TIME_SYNCH
//#define FOURE_CONF_DEFAULT_HOPPING_SEQUENCE (uint8_t[]){ 11, 26 }

#define LLSEC802154_CONF_ENABLED 0

#if LLSEC802154_CONF_ENABLED
#define SLOT_CONF_FRAME_SIZE 11
#define FOURE_CONF_DEFAULT_TIMESLOT_LENGTH 45000

#define LLSEC802154_CONF_USES_EXPLICIT_KEYS 1
#define LLSEC802154_CONF_USES_FRAME_COUNTER 0
#define LLSEC802154_CONF_USES_AUX_HEADER    1
#endif /* LLSEC802154_CONF_ENABLED */

#ifndef UIP_CONF_MAX_ROUTES
#define UIP_CONF_MAX_ROUTES          20
#endif

#ifndef NBR_TABLE_CONF_MAX_NEIGHBORS
#define NBR_TABLE_CONF_MAX_NEIGHBORS 8
#endif

#define FOURE_CONF_HARD_SLOTS    { {0, 0, SLOT_TYPE_TIMEKEEP}, {1, 0, SLOT_TYPE_TIMEKEEP}, {2, 0, SLOT_TYPE_SHARED}, {3, 0, SLOT_TYPE_SHARED}, {4, 0, SLOT_TYPE_SHARED}  }

#define FOURE_CONF_MAX_CONTENT 10
#define FOURE_CONF_MAX_SLOT    30
#define FOURE_CONF_MAX_SLOTS_PER_DESTINATION 10

#define SIXTOP_SERIAL_COMMANDS_ENABLE 1
#endif /* TSCH_TIME_SYNCH */
#endif /* PROJECT_CONF_H_ */
