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
PURPOSE: rf231_soc specific code
*/
#ifndef ZB_RF231SOC_H
#define ZB_RF231SOC_H 1

#include <basic_rf.h>

#define ZB_TASK_PRIORITY 20

/**
   Min channel # of RF231 SOC 
 */
#define ZB_TRANSCEIVER_START_CHANNEL_NUMBER 11
/**
   Max channel # of RF231 SOC
 */
#define ZB_TRANSCEIVER_MAX_CHANNEL_NUMBER   26

/**
   Send command/data/beacon to the transiver FIFO

   @param header_length - mhr length to write to UZ transiver
   @param buf - buffer to send
 */

#define ZB_TRANS_SEND_COMMAND(header_length, buf)          \
  zb_transceiver_send_fifo_packet((header_length), (buf), 1)

/**
   Receive packet from the transiver
 */
#define ZB_TRANS_RECV_PACKET(buf) zb_rf231soc_fifo_read(0, buf, 0)

/**
   Trigger TX in normal rf231_soc 

   @param need_ack - if 1, wait for ACK and retransmit automatically
 */
#define ZB_START_NORMAL_FIFO_TX(retries, need_ack)  \
/*
  (MAC_CTX().tx_cnt = 0,\
  (ZB_WRITE_SHORT_REG(ZB_SREG_TXNTRIG, ((retries << 5) & 0xE0) | ((need_ack)? 0x65 : 0x01))))
  Need to change this part to start transmission in rf231_soc TODO wsn gr12
*/


/**
   Header to be written into normal RF231 SOC fifo before packet
 */

typedef struct zb_ubec_fifo_header_s
{
  zb_uint8_t header_length;
  zb_uint8_t frame_length;
} 
zb_ubec_fifo_header_t;

/*
 * Change these macros to reflect firefly specific functionality
 */

#define ZB_UBEC_GET_RX_DATA_STATUS() (TRANS_CTX().int_status )  
#define ZB_UBEC_GET_TX_DATA_STATUS() (TRANS_CTX().int_status )  
#define ZB_UBEC_GET_WAKE_STATUS()    (TRANS_CTX().int_status )  
#define ZB_UBEC_CLEAR_RX_DATA_STATUS() (TRANS_CTX().int_status &= (~0x08))  

/**
   Transiver context specific for UZ2400
 */

typedef struct zb_transceiver_ctx_s
{
  zb_uint8_t int_status;
  zb_uint8_t tx_status;
  zb_uint8_t interrupt_flag;
}
zb_transceiver_ctx_t;

#define ZB_MAC_GET_BYTE_VALUE() ZG->mac.mac_ctx.rw_reg.value.byte_value
#define ZB_CLEAR_NORMAL_FIFO_BUSY() (TRANS_CTX().normal_fifo_busy = 0)
#define ZB_UBEC_GET_NORMAL_FIFO_TX_STATUS() (TRANS_CTX().int_status & 0x01) /* check bit 0 */
#define ZB_UBEC_CLEAR_NORMAL_FIFO_TX_STATUS() (TRANS_CTX().int_status &= 0xFE) /* clear bit 0 */

#define ZB_SET_TRANS_INT() (TRANS_CTX().interrupt_flag = 1)
#define ZB_CLEAR_TRANS_INT() (TRANS_CTX().interrupt_flag = 0)
#define ZB_GET_TRANS_INT() (TRANS_CTX().interrupt_flag)



/*
  clear ISRSTS bit 0  - TX Normal FIFO transmission interrupt bit
  clear TXSR bit 5 - CCAFAIL: Channel busy causes CSMA-CA fails
  clear TXSR bit 0 - Normal FIFO release status (1 - fail, retry count exceed)
*/
#define ZB_CLEAR_TX_STATUS() (TRANS_CTX().int_status &= 0xFE, TRANS_CTX().tx_status &= 0xDE)
/* check TXSR bit 0 - Normal FIFO release status (1 - fail, retry count exceed) */
#define ZB_IS_TX_CHANNEL_BUSY() (TRANS_CTX().tx_status & 0x20)
/* check TXSR bit 5 - CCAFAIL: Channel busy causes CSMA-CA fails */
#define ZB_IS_TX_RETRY_COUNT_EXCEEDED() (TRANS_CTX().tx_status & 0xDF)


/* set up/down pending bit */
#define ZB_SET_PENDING_BIT() 
#if 0
  /* TODO: wsn gr12 */
  ZB_READ_SHORT_REG(ZB_SREG_ACKTMOUT),                                  \
    ZB_WRITE_SHORT_REG(ZB_SREG_ACKTMOUT, ZB_MAC_GET_BYTE_VALUE()|0x80)
#endif

#define ZB_CLEAR_PENDING_BIT()
#if 0
  /* TODO: wsn gr12 */
ZB_READ_SHORT_REG(ZB_SREG_ACKTMOUT),\
ZB_WRITE_SHORT_REG(ZB_SREG_ACKTMOUT, ZB_MAC_GET_BYTE_VALUE()&0x7F)
#endif

#define ZB_TRANS_CHECK_CHANNEL_BUSY_ERROR() ZB_IS_TX_CHANNEL_BUSY() /* not 0 means channel busy error */
#define ZB_TRANS_CHECK_TX_RETRY_COUNT_EXCEEDED_ERROR() ZB_IS_TX_RETRY_COUNT_EXCEEDED() /* not 0 means cca fail error */
#define ZB_TRANS_CHECK_CHANNEL_ERROR() (ZB_IS_TX_CHANNEL_BUSY() || ZB_IS_TX_RETRY_COUNT_EXCEEDED())

/**
 * Initialization routine for zboss task for polling radio
 */
void zb_task_config();
void zb_nw_task();

/**
   Fill FIFO
 */

void zb_rf231soc_fifo_write(zb_uint16_t long_addr, zb_buf_t *buf) ;


/**
   Read from FIFO
 */
void zb_rf231soc_fifo_read(zb_uint8_t tx_fifo, zb_buf_t *buf, zb_uint8_t len) ;

/**
   Check that transiver is in the beacon mode
 */
zb_bool_t zb_check_beacon_mode_on();
#define ZB_CHECK_BEACON_MODE_ON() ZB_TRUE


zb_ret_t zb_transceiver_send_fifo_packet(zb_uint8_t header_length, 
                                         zb_buf_t *buf, zb_uint8_t need_tx) ;

/**
   Set channel os RF231 SOC transiver

   @param channel_number - channel number (absolute - means, channel 0 on UZ2400
                           is 11)
 */
void zb_transceiver_set_channel(zb_uint8_t channel_number);

/**
   Portable macro for zb_transceiver_set_channel()
 */
#define ZB_TRANSCEIVER_SET_CHANNEL(channel_number) zb_transceiver_set_channel(channel_number)

/**
   Get RSSI from RF231 SOC 

   @param rssi_value - (out) rssi value as UZ2400 returns it
 */
void zb_transceiver_get_rssi(zb_uint8_t *rssi_value);

/**
 * Update RF231 SOC PAN ID
 */
#define ZB_UPDATE_PAN_ID() \
  (ZB_TRANSCEIVER_SET_PAN_ID(MAC_PIB().mac_pan_id))

/**
   Assign short pan ID in RF231 SOC 

   @param pan_id - new pan id
 */

#define ZB_TRANSCEIVER_SET_PAN_ID(pan_id) rf_addr_decode_set_my_panid(pan_id)


/**
 * Assign short mac addr in RF231 SOC to value in PIB
 */
#define ZB_UPDATE_SHORT_ADDR() rf_addr_decode_set_my_mac(MAC_PIB().mac_short_address)
#define ZB_CLEAR_SHORT_ADDR() rf_addr_decode_clear_my_mac()


/**
   Assign ext (long) coordinator address in RF231 SOC 

   @param coord_addr_long - addres to remember
 */
void zb_transceiver_set_coord_ext_addr(zb_ieee_addr_t coord_addr_long);


/**
   Portable macro for zb_transceiver_set_coord_ext_addr()
*/
#define ZB_TRANSCEIVER_SET_COORD_EXT_ADDR(addr) zb_transceiver_set_coord_ext_addr((addr))

/**
   Assign short coordinator address in UZ2400

   @param coord_addr_short - addres to remember
 */
void zb_transceiver_set_coord_short_addr(zb_uint16_t coord_addr_short);

/**
   Portable macro for zb_transceiver_set_coord_short_addr()
 */
#define ZB_TRANSCEIVER_SET_COORD_SHORT_ADDR(addr) zb_transceiver_set_coord_short_addr((addr))



void zb_transceiver_update_long_mac();
#define ZB_UPDATE_LONGMAC() \
  zb_transceiver_update_long_mac()

#define ZB_TRANS_GO_IDLE()

/*fifo header */
#define ZB_FIFO_HEADER_SIZE 2

#define ZB_RX_QUEUE_CAP 4
/* main ring buffer, that contains whole packets itself */
ZB_RING_BUFFER_DECLARE(zb_rx_queue, zb_buf_t *, ZB_RX_QUEUE_CAP);
/* small ringbuf, containing registers, that read using direct memory access, instead of*/
/* appropriate function  */

#define ZB_PUT_RX_QUEUE() \
 if (ZB_RING_BUFFER_IS_FULL(&MAC_CTX().mac_rx_queue)) \
  { 							\
   MAC_CTX().recv_buf_full = 1; \
  } else                        \
  {    							\
  ZB_MEMCPY(ZB_BUF_BEGIN(MAC_CTX().mac_rx_queue.ring_buf[MAC_CTX().mac_rx_queue.write_i]),\
                (zb_uint8_t *)zb_rfRxInfo.pPayload,                 \
				zb_rfRxInfo.length) ; \
  MAC_CTX().mac_rx_queue.ring_buf[MAC_CTX().mac_rx_queue.write_i]->u.hdr.len =  zb_rfRxInfo.length ; \
  MAC_CTX().mac_rx_queue.written++;                                                     \
  MAC_CTX().mac_rx_queue.write_i = (MAC_CTX().mac_rx_queue.write_i + 1) % ZB_RX_QUEUE_CAP;           \
  rx_buf_empty = 1;   \
  }


#define ZB_GET_RX_QUEUE()       \
  {												\
	  zb_buf_t *mac_rx_queue_tmp;		 \
	  mac_rx_queue_tmp = MAC_CTX().recv_buf; \
	  MAC_CTX().recv_buf = MAC_CTX().mac_rx_queue.ring_buf[MAC_CTX().mac_rx_queue.read_i];\
	  MAC_CTX().mac_rx_queue.ring_buf[MAC_CTX().mac_rx_queue.read_i] = mac_rx_queue_tmp; \
  	  ZB_RING_BUFFER_FLUSH_GET(&MAC_CTX().mac_rx_queue);\
      if (MAC_CTX().recv_buf_full) \
	  {							   \
	  	MAC_CTX().recv_buf_full = 0;\
		ZB_PUT_RX_QUEUE();\
	  };						   \
  }

#endif /* ZB_RF231SOC_H */
