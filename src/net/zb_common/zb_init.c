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
PURPOSE: ZigBee stack initialization
*/


#include "zb_common.h"

#include "zb_bufpool.h"
#include "zb_ringbuffer.h"
#include "zb_scheduler.h"
#include "zb_mac_transport.h"
#include "zb_aps.h"

/*! \addtogroup ZB_BASE */
/*! @{ */

void zb_zdo_init() ;


/**
   Globals data structure implementation - let it be here.

   FIXME: maybe, put it into separate .c file?
 */
 zb_globals_t g_zb;
 zb_intr_globals_t g_izb;
 zb_64bit_addr_t g_zero_addr={0,0,0,0,0,0,0,0};


void zb_init() 
{
  ZB_MEMSET(&g_zb, 0, sizeof(zb_globals_t));
  ZB_MEMSET((void*)&g_izb, 0, sizeof(zb_intr_globals_t));
  /* some init of 8051 HW moved to zb_low_level_init() */
  ZB_START_DEVICE();

  TRACE_INIT("");
  /* special trick for ns build run on 8051 simulator: get node number from the
   * rx pipe name  */
  /* set defaults, then update it from nvram */
  zb_ib_set_defaults((char*)"");
  zb_ib_load();

  zb_sched_init();
  zb_init_buffers();

  zb_mac_init();

  zb_nwk_init();

#if defined ZB_NVRAM_WRITE_CFG && defined ZB_USE_NVRAM && defined C8051F120
/* Write config to nvram. Think there's no any reason to invoke this second time*/
/*
  zb_uint8_t aps_designated_coord
  zb_uint8_t aps_use_insecure_join
  zb_uint8_t aps_use_extended_pan_id
  zb_ieee_addr_t mac_extended_address
*/
  {
    zb_uint8_t addr[8]={0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x0B};
    zb_write_nvram_config(0, 1, 1, addr);
  }
#endif

#ifdef ZB_USE_NVRAM
/*  zb_config_from_nvram();
  zb_read_up_counter();
  zb_read_security_key();
  zb_read_formdesc_data();*/
#endif

  zb_aps_init();
  zb_zdo_init();
}


void zb_handle_parms_before_start()
{
   /* if pan_id isn't set, it should be 0xffff to receive all beacons w/o using promiscous mode */
   /* TODO: add random pan_id generator */
   /* we should not check nib_dev_type, because it's not set yet */
   /* if ((!MAC_PIB().mac_pan_id)&&(ZB_NIB_DEVICE_TYPE() != ZB_NWK_DEVICE_TYPE_COORDINATOR))*/
if ((!MAC_PIB().mac_pan_id)&&(!ZB_AIB().aps_designated_coordinator))
  MAC_PIB().mac_pan_id = 0xffff;
  ZB_UPDATE_LONGMAC();
  ZB_UPDATE_PAN_ID();
}

/*! @} */
