/***************************************************************************
*                      ZBOSS ZigBee Pro 2007 stack                         *
*                                                                          *
*          Copyright (c) 2012 DSR Corporation Denver CO, USA.              *
*                       http://www.dsr-wireless.com                        *
*                                                                          *
*                            All rights reserved.                          *
*          Copyright (c) 2011 ClarIDy Solutions, Inc., Taipei, Taiwan.     *
*                       http://www.claridy.com/                            *
*                                                                          *
*          Copyright (c) 2011 Uniband Electronic Corporation (UBEC),       *
*                             Hsinchu, Taiwan.                             *
*                       http://www.ubec.com.tw/                            *
*                                                                          *
*          Copyright (c) 2011 DSR Corporation Denver CO, USA.              *
*                       http://www.dsr-wireless.com                        *
*                                                                          *
*                            All rights reserved.                          *
*                                                                          *
*                                                                          *
* ZigBee Pro 2007 stack, also known as ZBOSS (R) ZB stack is available     *
* under either the terms of the Commercial License or the GNU General      *
* Public License version 2.0.  As a recipient of ZigBee Pro 2007 stack, you*
* may choose which license to receive this code under (except as noted in  *
* per-module LICENSE files).                                               *
*                                                                          *
* ZBOSS is a registered trademark of DSR Corporation AKA Data Storage      *
* Research LLC.                                                            *
*                                                                          *
* GNU General Public License Usage                                         *
* This file may be used under the terms of the GNU General Public License  *
* version 2.0 as published by the Free Software Foundation and appearing   *
* in the file LICENSE.GPL included in the packaging of this file.  Please  *
* review the following information to ensure the GNU General Public        *
* License version 2.0 requirements will be met:                            *
* http://www.gnu.org/licenses/old-licenses/gpl-2.0.html.                   *
*                                                                          *
* Commercial Usage                                                         *
* Licensees holding valid ClarIDy/UBEC/DSR Commercial licenses may use     *
* this file in accordance with the ClarIDy/UBEC/DSR Commercial License     *
* Agreement provided with the Software or, alternatively, in accordance    *
* with the terms contained in a written agreement between you and          *
* ClarIDy/UBEC/DSR.                                                        *
*                                                                          *
****************************************************************************
PURPOSE: to port ZBOSS v1 onto Firefly3. 
*/
#include "zb_common.h"
#include "zb_rf231_soc.h"

#include "zb_scheduler.h"
#include "zb_nwk.h"
#include "zb_mac.h"
#include "mac_internal.h"
#include "zb_mac_transport.h"
#include "zb_secur.h"

/* NanoRK includes */
#include <include.h>
#include <basic_rf.h>
#include <nrk.h>
#include <ulib.h>
#include <stdlib.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <nrk.h>
#include <nrk_events.h>
#include <nrk_timer.h>
#include <nrk_error.h>
#include <nrk_reserve.h>
#include <nrk_cfg.h>

/* definitions for zb_nw_task */
#ifndef ZB_STACKSIZE
#define ZB_STACKSIZE 128
#endif
//TODO wsn gr12 need to check the period 
#define ZB_DEFAULT_CHECK_RATE_MS  100
#define ZB_TASK_PRIORITY 20
#define ZB_MIN_CHECK_RATE_MS 20
#define ZB_DEFAULT_CHECK_RATE_MS 1000
#define RF_IO_BUF_SIZE 148

static nrk_task_type zb_task;
static NRK_STK zb_task_stack[ZB_STACKSIZE];
static uint32_t rx_failure_cnt;

static uint8_t tx_data_ready;
static uint8_t zb_running;
static uint8_t pkt_got_ack; // wsn gr12 needed??
static uint8_t g_chan;
static uint8_t is_enabled;
static nrk_time_t dummy_t;
static nrk_time_t _zb_check_period;

RF_RX_INFO zb_rfRxInfo;
RF_TX_INFO zb_rfTxInfo;

uint8_t rx_buf_empty;
uint8_t rx_buf[RF_IO_BUF_SIZE];
uint8_t tx_buf[RF_IO_BUF_SIZE];

int8_t zb_nrk_rf_init()
{
    tx_data_ready = 0;
    // Set the one main rx buffer
    zb_rfRxInfo.pPayload = rx_buf;
    zb_rfRxInfo.max_length = RF_IO_BUF_SIZE;
    printf("zb_nrk_rf_init: max_length:%d RF_IO_BUF_SIZE: %d \r\n", zb_rfRxInfo.max_length, RF_IO_BUF_SIZE);

    // Setup the cc2420 chip
    rf_power_up ();
    rf_init (&zb_rfRxInfo, ZB_MAC_START_CHANNEL_NUMBER, 0xFFFF, 0x00000);
    g_chan = ZB_MAC_START_CHANNEL_NUMBER;
    rx_buf_empty = 1;

    rf_set_cca_thresh (0x0);
    zb_running = 1;
    is_enabled = 1;
 
    _zb_check_period.secs = 0;
    //_zb_check_period.nano_secs = ZB_DEFAULT_CHECK_RATE_MS * NANOS_PER_MS;
    _zb_check_period.nano_secs = 1000;

    rf_auto_ack_disable();

    nrk_int_enable();

    return NRK_OK;
}

void zb_task_config ()
{
  nrk_task_set_entry_function (&zb_task, zb_nw_task);
  nrk_task_set_stk (&zb_task, zb_task_stack, ZB_STACKSIZE);
  zb_task.prio = ZB_TASK_PRIORITY;
  zb_task.FirstActivation = TRUE;
  zb_task.Type = BASIC_TASK;
  zb_task.SchType = PREEMPTIVE;
  zb_task.period.secs = 0;
  //zb_task.period.nano_secs = ZB_MIN_CHECK_RATE_MS * NANOS_PER_MS;
  zb_task.period.nano_secs = 1000;
  zb_task.cpu_reserve.secs = 0;       // zb reserve , 0 to disable
  zb_task.cpu_reserve.nano_secs = 0;
  zb_task.offset.secs = 0;
  zb_task.offset.nano_secs = 0;
#ifdef DEBUG
  printf ("zb activate\r\n");
#endif

  nrk_activate_task (&zb_task);
}

// Assuming that CCA returned 1 and a packet is on its way
// Receive the packet or timeout and error
int8_t _zb_rx ()
{
  int8_t n;
  uint8_t cnt;

  rf_rx_on ();
  cnt = 0;
//printf( "calling rx\r\n" );
// TODO: do we need to wait?!!! wsn gr12
  dummy_t.secs = 0;
  dummy_t.nano_secs = 5 * NANOS_PER_MS;
  nrk_wait (dummy_t);

  n = zb_rf_rx_packet_nonblock ();

  if (n != NRK_OK) {
    if (rx_failure_cnt < 65535)
      rx_failure_cnt++;
    rf_rx_off ();
    return 0;
  }

  /*
   * wsn gr12 TODO FIXME Do we really need rx_buf_task 
   * This is a flag used in bmac where the user buffer is
   * set to the rfRxInfo.pPayload structure and buf is 
   * initially set to be free. Here it is set not to be
   * free as a packet has been filled in
   * Check src/net/bmac/rf231_soc/bmac.c:bmac_rx_pkt_set_buffer
   * Ans: We may not need it and add to RX queue of ZB instead 
   */ 
  rx_buf_empty = 0;
#ifdef DEBUG
  printf ("ZB: SNR= %d [", zb_rfRxInfo.rssi);
  for (uint8_t i = 0; i < zb_rfRxInfo.length; i++)
    printf ("%c", zb_rfRxInfo.pPayload[i]);
  printf ("]\r\n");
#endif
  rf_rx_off ();
  return 1;
}

int8_t _zb_channel_check()
{
    int8_t val = 0;
    rf_rx_on();
    val += rf_cca_check();
    val += rf_cca_check();
    val += rf_cca_check();
    if (val > 1)
        val = 1;
    rf_rx_off();
    return val;
}

void zb_nw_task ()
{
    /*
     * Zigbee feature for pending data polling
     * TODO: need to figure this out for nanork
     if (ZB_MAC_GET_INDIRECT_DATA_REQUEST())
     {
     if (ZB_CHECK_PENDING())
     {
     ZB_MAC_SET_PENDING_DATA();
     } else
     {
     ZB_MAC_CLEAR_PENDING_DATA();
     }
     }
     */
    int8_t v, i;
    int8_t e;

    while (1) {
        rf_power_up ();
        v = 1;

        if (rx_buf_empty == 1)
            v = _zb_channel_check ();
        else
            ZB_PUT_RX_QUEUE();
        // zb_channel check turns on radio, don't turn off if
        // data is coming.

        if (v == 0) {
            if (_zb_rx () == 1) {
                ZB_PUT_RX_QUEUE();
            }
        }

//TODO wsn gr12
//should we check and transmit data here itself or do we 
        //if (tx_data_ready == 1) {
          //  _zb_tx ();
        //}
        rf_rx_off ();
        rf_power_down ();

        nrk_wait (_zb_check_period);
    }
}

void zb_transceiver_set_channel(zb_uint8_t channel_number)
{
  /* 5 is a frequency step */
  MAC_CTX().current_channel = channel_number;
  rf_power_up ();
  g_chan = channel_number;
  rf_set_rx(&zb_rfRxInfo, channel_number);
}

void zb_transceiver_set_coord_ext_addr(zb_ieee_addr_t coord_addr_long)
{
}

void zb_transceiver_set_coord_short_addr(zb_uint16_t coord_addr_short)
{
}


void zb_mac_short_write_reg(zb_uint8_t short_addr, zb_uint8_t byte_value) 
{
}

void zb_mac_long_write_reg(zb_uint16_t long_addr, zb_uint8_t byte_value) 
{
}

void zb_mac_short_read_reg(zb_uint8_t short_addr) 
{
}

void zb_transceiver_update_long_mac()
{
/* TODO wsn gr 12: Reference Atmega128RFA1 datasheet , needs verification*/
  IEEE_ADDR_0 = ZB_PIB_EXTENDED_ADDRESS()[0];
  IEEE_ADDR_1 = ZB_PIB_EXTENDED_ADDRESS()[1];
  IEEE_ADDR_2 = ZB_PIB_EXTENDED_ADDRESS()[2];
  IEEE_ADDR_3 = ZB_PIB_EXTENDED_ADDRESS()[3];
  IEEE_ADDR_4 = ZB_PIB_EXTENDED_ADDRESS()[4];
  IEEE_ADDR_5 = ZB_PIB_EXTENDED_ADDRESS()[5];
  IEEE_ADDR_6 = ZB_PIB_EXTENDED_ADDRESS()[6];
  IEEE_ADDR_7 = ZB_PIB_EXTENDED_ADDRESS()[7];
}

void zb_transceiver_get_rssi(zb_uint8_t *rssi_value)
{
   *rssi_value = zb_rfRxInfo.rssi;
}

zb_ret_t zb_transceiver_send_fifo_packet(zb_uint8_t header_length, 
                                         zb_buf_t *buf, zb_uint8_t need_tx) 
{
  zb_uint8_t *fc = ZB_BUF_BEGIN(buf);
  zb_uint8_t frame_len = ZB_BUF_LEN(buf);
  zb_uint8_t retval = 10;

  /*TRACE_MSG(TRACE_MAC1, ">> zb_transceiver_send_fifo_packet, %d, buf %p, state %hd", (FMT__D_P,
                        (zb_uint16_t)header_length, buf));*/
  TRACE_MSG(TRACE_MAC1, ">> zb_transceiver_send_fifo_packet, header len %d, frame_len %d", (FMT__D_D,
                        (zb_uint16_t)header_length, frame_len));

  /* TODO: if acknowledgement is required for normal fifo, set ackreq
   * bit (SREG0x1B[2]) */
  /* we need to determine if our frame broadcast or not */

  /* Don't want to parse entire mhr here. All we need is frame control and
   * destination address. Destination address has fixed position in mhr.
   * Fields layout is fc (2b), seq number (1b), dest panid (2b), dest address (2b).
   */
  zb_uint8_t need_ack = (!((ZB_FCF_GET_FRAME_TYPE(fc) == MAC_FRAME_BEACON
                            || (ZB_FCF_GET_DST_ADDRESSING_MODE(fc) == ZB_ADDR_16BIT_DEV_OR_BROADCAST
                                && fc[5] == 0xff && fc[6] == 0xff)))
                         && ZB_FCF_GET_ACK_REQUEST_BIT(fc));


  if( (retval = zb_rf_tx_packet(buf, frame_len)) != NRK_OK)
      TRACE_MSG(TRACE_MAC1, "--- RF_TX ERROR ---", (FMT__0));

  //printf("send_fifo_packet:: retval %d\r\n", retval);

    /* The same bit is used to start normal and beacon fifio.
       If not joined yet (pac_pan_id is not set), do not request acks.
    */
    TRACE_MSG(TRACE_MAC2, "Need ACK: %hd", (FMT__H, need_ack));
    //TODO wsn gr12 We transmit using rf_tx_packet. we need not have this
    //ZB_START_NORMAL_FIFO_TX(ZB_MAC_PIB_MAX_FRAME_RETRIES, need_ack);
    if (need_ack)
    {
      ZB_MAC_START_ACK_WAITING();
    }
    else
    {
      ZB_MAC_SET_ACK_OK();
    }
  TRACE_MSG(TRACE_MAC1, "<< zb_transceiver_send_fifo_packet", (FMT__0));
  return RET_OK;
}
