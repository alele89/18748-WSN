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
#define ZB_STACKSIZE 256
#endif
//TODO wsn gr12 need to check the period 
#define ZB_DEFAULT_CHECK_RATE_MS  100
#define ZB_TASK_PRIORITY 20
#define ZB_MIN_CHECK_RATE_MS 16
#define RF_IO_BUF_SIZE 148

static nrk_task_type zb_task;
static NRK_STK zb_task_stack[ZB_STACKSIZE];
static uint32_t rx_failure_cnt;

volatile static uint8_t tx_data_ready;
static uint8_t zb_running;
static uint8_t pkt_got_ack; // wsn gr12 needed??
static uint8_t g_chan;
static uint8_t is_enabled;
static nrk_time_t dummy_t;
static nrk_time_t _zb_check_period;

RF_RX_INFO zb_rfRxInfo;
RF_TX_INFO zb_rfTxInfo;

static uint8_t cca_active;
uint8_t rx_buf_empty;
uint8_t rx_buf[RF_IO_BUF_SIZE];
uint8_t tx_buf[RF_IO_BUF_SIZE];

int8_t zb_nrk_rf_init()
{
    tx_data_ready = 0;
    rx_buf_empty = 1;
    cca_active = true;
    rx_failure_cnt = 0;

    // Set the one main rx buffer
    zb_rfRxInfo.pPayload = rx_buf;
    zb_rfRxInfo.max_length = RF_IO_BUF_SIZE;

    _zb_check_period.secs = 0;
    _zb_check_period.nano_secs = ZB_DEFAULT_CHECK_RATE_MS * NANOS_PER_MS;
    //_zb_check_period.nano_secs = 100000;
    zb_rx_pkt_signal = nrk_signal_create ();
    if (zb_rx_pkt_signal == NRK_ERROR) {
        nrk_kprintf (PSTR ("ZB ERROR: creating rx signal failed\r\n"));
        nrk_kernel_error_add (NRK_SIGNAL_CREATE_ERROR, nrk_cur_task_TCB->task_ID);
        return NRK_ERROR;
    }
    zb_tx_pkt_done_signal = nrk_signal_create ();
    if (zb_tx_pkt_done_signal == NRK_ERROR) {
        nrk_kprintf (PSTR ("ZB ERROR: creating tx signal failed\r\n"));
        nrk_kernel_error_add (NRK_SIGNAL_CREATE_ERROR, nrk_cur_task_TCB->task_ID);
        return NRK_ERROR;
    }

    // Setup the cc2420 chip
    rf_power_up ();
    rf_init (&zb_rfRxInfo, ZB_MAC_START_CHANNEL_NUMBER, 0xFFFF, 0x00000);
    g_chan = ZB_MAC_START_CHANNEL_NUMBER;

    rf_set_cca_thresh (0x0);
    zb_running = 1;
    is_enabled = 1;
    
    //rf_auto_ack_disable();

    //nrk_int_enable();

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
  zb_task.period.nano_secs = ZB_MIN_CHECK_RATE_MS * NANOS_PER_MS;
  //zb_task.period.nano_secs = 100000;
  zb_task.cpu_reserve.secs = 0;       // zb reserve , 0 to disable
  zb_task.cpu_reserve.nano_secs = 0;
  zb_task.offset.secs = 0;
  zb_task.offset.nano_secs = 0;
#ifdef DEBUG
  printf ("zb activate\r\n");
#endif

  nrk_activate_task (&zb_task);
}

int8_t _zb_channel_check ()
{
  int8_t val = 0;

  rf_rx_on ();
  val += rf_cca_check ();
  val += rf_cca_check ();
  val += rf_cca_check ();
  if (val > 1)
    val = 1;
  rf_rx_off ();
  return val;
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

/*while ( rf_rx_packet_nonblock() != NRK_OK )
	{
	cnt++;
	nrk_wait(_bmac_check_period);
	if(cnt>2) { 
			#ifdef DEBUG
			printf( "rx timeout 1 %d\r\n",cnt );
			#endif
			if(rx_failure_cnt<65535) rx_failure_cnt++;
			rf_rx_off();
			return 0;
			} 
	}
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

uint8_t _b_pow (uint8_t in)
{
  uint8_t i;
  uint8_t result;
  if (in <= 1)
    return 1;
  if (in > 7)
    in = 6;                     // cap it at 128 
  result = 1;
  for (i = 0; i < in; i++)
    result = result * 2;
  return result;
}


int8_t _zb_tx ()
{
  uint8_t v, backoff, backoff_count;
  uint16_t b;

#ifdef DEBUG
  nrk_kprintf (PSTR ("_zb_tx()\r\n"));
#endif
  if (cca_active) {

// Add random time here to stop nodes from synchronizing with eachother
    b = _nrk_time_to_ticks (&_zb_check_period);
    b = b / ((rand () % 10) + 1);
//printf( "waiting %d\r\n",b );
    nrk_wait_until_ticks (b);
//nrk_wait_ticks(b);

    backoff_count = 1;
    do {
#ifdef BMAC_MOD_CCA
	v=_zb_rx();
	v=!v;
        if (v == 1) { 
		break; 
	}
        //nrk_event_signal (zb_rx_pkt_signal);
#else
      v = _zb_channel_check ();
      if (v == 1)
        break;
#endif
      // Channel is busy
      backoff = rand () % (_b_pow (backoff_count));
#ifdef DEBUG
      printf ("backoff %d\r\n", backoff);
#endif
//      printf( "backoff %d\r\n",backoff );
      nrk_wait_until_next_n_periods (backoff);
      backoff_count++;
      if (backoff_count > 6)
        backoff_count = 6;      // cap it at 64    
      b = _nrk_time_to_ticks (&_zb_check_period);
      b = b / ((rand () % 10) + 1);
//      printf( "waiting %d\r\n",b );
      nrk_wait_until_ticks (b);
//      nrk_wait_ticks(b);

    }
    while (v == 0);
  }

  // send extended preamble
  zb_rfTxInfo.cca = 0;
  zb_rfTxInfo.ackRequest = 0;

  uint16_t ms = _zb_check_period.secs * 1000;
  ms += _zb_check_period.nano_secs / 1000000;
  //printf( "CR ms: %u\n",ms );
  //target_t.nano_secs+=20*NANOS_PER_MS;
  rf_rx_on ();
  pkt_got_ack = zb_rf_tx_packet (&zb_rfTxInfo, ms);

  if (!pkt_got_ack)
      TRACE_MSG(TRACE_MAC1, "--- RF_TX ERROR ---", (FMT__0));
  // send packet
  // pkt_got_ack=rf_tx_packet (&bmac_rfTxInfo);
  rf_rx_off ();                 // Just in case auto-ack left radio on
  tx_data_ready = 0;
  //nrk_event_signal (zb_tx_pkt_done_signal);
  return NRK_OK;
}

void zb_nw_task ()
{
  int8_t v, i;
  int8_t e;
  uint8_t backoff;
  nrk_sig_mask_t event;

  backoff = 0;
  while (1) {
#ifdef NRK_SW_WDT
#ifdef BMAC_SW_WDT_ID
    nrk_sw_wdt_update (BMAC_SW_WDT_ID);
#endif
#endif
    rf_power_up ();
    if (is_enabled) {
      v = 1;

#ifdef BMAC_MOD_CCA
      if (rx_buf_empty == 1)
      {
          if (_zb_rx () == 1) e = nrk_event_signal (zb_rx_pkt_signal);
      }
      else
          e = nrk_event_signal (zb_rx_pkt_signal);
#else
      if (rx_buf_empty == 1)
        v = _zb_channel_check ();
      // If the buffer is full, signal the receiving task again.
      else
      {
        ZB_PUT_RX_QUEUE();
        e = nrk_event_signal (zb_rx_pkt_signal);
      }
      // zb_channel check turns on radio, don't turn off if
      // data is coming.

      if (v == 0) {
        if (_zb_rx () == 1) {
          ZB_PUT_RX_QUEUE();
          e = nrk_event_signal (zb_rx_pkt_signal);
          //if(e==NRK_ERROR) {
          //      nrk_kprintf( PSTR("bmac rx pkt signal failed\r\n"));
          //      printf( "errno: %u \r\n",nrk_errno_get() );
          //}
        }
        //else nrk_kprintf( PSTR("Pkt failed, buf could be corrupt\r\n" ));

      }

#endif
      if (tx_data_ready == 1) {
        _zb_tx ();
      }
      rf_rx_off ();
      rf_power_down ();

      //do {
      nrk_wait (_zb_check_period);
      //      if(rx_buf_empty_bmac!=1)  nrk_event_signal (bmac_rx_pkt_signal);
      //} while(rx_buf_empty_bmac!=1);
    }
    else {
#if 0
      event = 0;
      do {
        v = nrk_signal_register (bmac_enable_signal);
        event = nrk_event_wait (SIG (bmac_enable_signal));
      }
      while ((event & SIG (bmac_enable_signal)) == 0);
#endif
    }
    //nrk_wait_until_next_period();
  }
}


#if 0
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
//TODO wsn gr12
//no need rf_rx_off -> rx_ready = 0 done in zb_rf_rx_nonblock NOT TRUE!
//no need rf_power_down as it is causing errors in transmit and really,
//we are not looking to save power now
        rf_rx_off ();
        //rf_power_down ();
        //TODO wsn gr 12 changing this to nrk_wait_until_next_period
        //nrk_wait (_zb_check_period);
        nrk_wait_until_next_period();
    }
}
#endif

void zb_transceiver_set_channel(zb_uint8_t channel_number)
{
  /* 5 is a frequency step */
  MAC_CTX().current_channel = channel_number;
  rf_power_up ();
  g_chan = channel_number;
  //rf_init (&zb_rfRxInfo, channel_number, 0xFFFF, 0x00000);
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
  uint32_t mask;

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

  uint8_t v = nrk_signal_register (zb_tx_pkt_done_signal);
  if (v == NRK_ERROR)
    nrk_kprintf(PSTR("Failed to register tx signal"));
  tx_data_ready = 1;
  zb_rfTxInfo.pPayload = fc;
  zb_rfTxInfo.length = frame_len;
  zb_rfTxInfo.ackRequest = need_ack;
#ifdef DEBUG
  nrk_kprintf (PSTR ("Waiting for tx done signal\r\n"));
#endif
/*
  mask = nrk_event_wait (SIG (zb_tx_pkt_done_signal));
  if (mask == 0)
    nrk_kprintf (PSTR ("ZB TX: Error calling event wait\r\n"));
  if ((mask & SIG (zb_tx_pkt_done_signal)) == 0)
    nrk_kprintf (PSTR ("ZB TX: Woke up on wrong signal\r\n"));
  if (pkt_got_ack)
  {
    return RET_OK;
  }
  else
  {
      TRACE_MSG(TRACE_MAC1, "--- RF_TX ERROR ---", (FMT__0));
  }
*/
    while (tx_data_ready);
    printf("Sync tx data done\r\n");
    if (need_ack)
      ZB_MAC_SET_ACK_OK();
    return RET_OK;
#if 0
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
#endif

  TRACE_MSG(TRACE_MAC1, "<< zb_transceiver_send_fifo_packet", (FMT__0));
  return RET_OK;
}
