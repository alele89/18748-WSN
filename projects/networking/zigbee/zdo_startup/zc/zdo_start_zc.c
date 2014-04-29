/*************************************************************************
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
PURPOSE: Test for ZC application written using ZDO.
*/
/* NanoRK includes */
#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <hal.h>
#include <nrk_error.h>

#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <ff_basic_sensor.h>

#include <stdlib.h>


/* ZBOSS includes */
#include "zb_common.h"
#include "zb_scheduler.h"
#include "zb_bufpool.h"
#include "zb_nwk.h"
#include "zb_aps.h"
#include "zb_zdo.h"
#include "zb_types.h"
#include "zb_rf231_soc.h"

NRK_STK Stack1[NRK_APP_STACKSIZE];
nrk_task_type TaskOne;

void zc_task(void);

void nrk_create_taskset();
void nrk_register_drivers();

#define ZB_TEST_DUMMY_DATA_SIZE 1

zb_ieee_addr_t g_zc_addr = {0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa};

/*! \addtogroup ZB_TESTS */
/*! @{ */

#ifndef ZB_COORDINATOR_ROLE
#error Coordinator role is not compiled!
#endif

/*
   The test is: ZC starts PAN, ZR joins to it by association and send APS data packet, when ZC
   received packet, it sends packet to ZR, when ZR received packet, it sends
   packet to ZC etc.
   */

static void zc_send_data(zb_buf_t *buf, zb_uint16_t addr);

void data_indication(zb_uint8_t param) ;

int main ()
{
    nrk_setup_ports ();
    nrk_setup_uart (UART_BAUDRATE_115K2);

    nrk_init ();

    nrk_register_drivers();

    nrk_led_clr (0);
    nrk_led_clr (1);
    nrk_led_clr (2);
    nrk_led_clr (3);

    nrk_time_set (0, 0);

    printf("Hello World from ZBOSS ZC..\r\n");

    zb_task_config ();
    zb_nrk_rf_init();

    nrk_create_taskset ();
    nrk_start ();

    return 0;
}

void nrk_create_taskset()
{
	TaskOne.task =  zc_task;
	nrk_task_set_stk( &TaskOne, Stack1, NRK_APP_STACKSIZE);
	TaskOne.prio = 1;
	TaskOne.FirstActivation = TRUE;
	TaskOne.Type = BASIC_TASK;
	TaskOne.SchType = PREEMPTIVE;
    // wsn gr12 TODO: check the period. may need to change.
	TaskOne.period.secs = 1;
	TaskOne.period.nano_secs = 0;//0*NANOS_PER_MS;
	TaskOne.cpu_reserve.secs = 0;
	TaskOne.cpu_reserve.nano_secs = 0;//0*NANOS_PER_MS;
	TaskOne.offset.secs = 0;
	TaskOne.offset.nano_secs= 0;

	nrk_activate_task (&TaskOne);
}

void zc_task()
{
    /* 
     * Init device, load IB values from nvram or set it to default 
     * Resets g_zb and g_izb.
     * Initializes TRACE, sched, buffers, mac, nwk, aps, zdo
     * */
    zb_init();

    ZB_IEEE_ADDR_COPY(ZB_PIB_EXTENDED_ADDRESS(), &g_zc_addr);
    MAC_PIB().mac_pan_id = 0x1aaa;
    
    /* let's always be coordinator */
    ZB_AIB().aps_designated_coordinator = 1;
     
    if (zdo_dev_start() != RET_OK)
    {
        TRACE_MSG(TRACE_ERROR, "zdo_dev_start failed", (FMT__0));
    }
    else
        zdo_main_loop();
}

void zb_zdo_startup_complete(zb_uint8_t param) 
{
    zb_buf_t *buf = ZB_BUF_FROM_REF(param);
    TRACE_MSG(TRACE_APS1, ">>zb_zdo_startup_complete status %d", (FMT__D, (int)buf->u.hdr.status));
    if (buf->u.hdr.status == 0)
    {
        TRACE_MSG(TRACE_APS1, "Device STARTED OK", (FMT__0));
        zb_af_set_data_indication(data_indication);
    }
    else
    {
        TRACE_MSG(TRACE_APS1, "Device start FAILED status %d", (FMT__D, (int)buf->u.hdr.status));
    }
    zb_free_buf(buf);
}


/*
   Trivial test: dump all APS data received
 */
void data_indication(zb_uint8_t param) 
{
    zb_uint8_t *ptr;
    zb_buf_t *asdu = (zb_buf_t *)ZB_BUF_FROM_REF(param);
    zb_apsde_data_indication_t *ind = ZB_GET_BUF_PARAM(asdu, zb_apsde_data_indication_t);

    /* Remove APS header from the packet */
    ZB_APS_HDR_CUT_P(asdu, ptr);

    TRACE_MSG(TRACE_APS3, "apsde_data_indication: packet %p len %d handle 0x%x", (FMT__P_D_D,
                asdu, (int)ZB_BUF_LEN(asdu), asdu->u.hdr.status));

    /* send packet back to ZR */
    zc_send_data(asdu, ind->src_addr);
}

static void zc_send_data(zb_buf_t *buf, zb_uint16_t addr)
{
    zb_apsde_data_req_t *req;
    zb_uint8_t *ptr = NULL;
    zb_short_t i;

    ZB_BUF_INITIAL_ALLOC(buf, ZB_TEST_DUMMY_DATA_SIZE , ptr);
    req = ZB_GET_BUF_TAIL(buf, sizeof(zb_apsde_data_req_t));
    req->dst_addr.addr_short = addr; /* send to ZR */
    req->addr_mode = ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    req->tx_options = ZB_APSDE_TX_OPT_ACK_TX;
    req->radius = 1;
    req->profileid = 2;
    req->src_endpoint = 10;
    req->dst_endpoint = 10;
    buf->u.hdr.handle = 0x11;
    for (i = 0 ; i < ZB_TEST_DUMMY_DATA_SIZE ; i++)
    {
        ptr[i] = i + '0';
    }
    TRACE_MSG(TRACE_APS3, "Sending apsde_data.request", (FMT__0));
    ZB_SCHEDULE_CALLBACK(zb_apsde_data_request, ZB_REF_FROM_BUF(buf));
}

void nrk_register_drivers()
{
/*
 * TODO: wsn gr12 Need to fix this
    int8_t val;
    val = nrk_register_driver(&dev_manager_ff3_sensors, FIREFLY_3_SENSOR_BASIC);
    if (val == NRK_ERROR)
        nrk_kprintf(PSTR("Failed to load ADC driver\r\n"));
*/
}

/*! @} */
