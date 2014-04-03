/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2007, Real-Time and Multimedia Lab, Carnegie Mellon University
*  All rights reserved.
*
*  This is the Open Source Version of Nano-RK included as part of a Dual
*  Licensing Model. If you are unsure which license to use please refer to:
*  http://www.nanork.org/nano-RK/wiki/Licensing
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, version 2.0 of the License.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*******************************************************************************/

#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <hal.h>
#include <basic_rf.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>

void user_recv(void);
void my_callback(uint16_t global_slot );
void zb_interdemo_main_loop();


RF_TX_INFO rfTxInfo;
RF_RX_INFO rfRxInfo;
uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];
//------------------------------------------------------------------------------
//      void main (void)
//
//      DESCRIPTION:
//              Startup routine and main loop
//------------------------------------------------------------------------------
int main (void)
{
    nrk_setup_ports(); 
    nrk_setup_uart (UART_BAUDRATE_115K2);
    
    zb_init();
    zb_sched_main_loop();
}

void user_tx() 
{
    uint8_t cnt,i,length;

    printf( "Basic TX...\r\n" ); 
    nrk_led_set(0); 
    nrk_led_set(1); 
    nrk_led_clr(2); 
    nrk_led_clr(3); 
/*
	    while(1) {
		   
				for(i=0; i<40; i++ )
					halWait(10000);
		    nrk_led_toggle(1);

	    }

*/
  
    rfRxInfo.pPayload = rx_buf;
    rfRxInfo.max_length = RF_MAX_PAYLOAD_SIZE;
    rfRxInfo.ackRequest= 0;
		nrk_int_enable();
    rf_init (&rfRxInfo, 13, 0x2420, 0x1214);
    cnt=0;


    // Code to control the CC2591 
    DPDS1=0x3; 
    DDRG=0x1;
    PORTG=0x1;

    DDRE=0xE0;
    PORTE=0xE0;




//				nrk_led_set(GREEN_LED);
    		rfTxInfo.pPayload=tx_buf;
    		sprintf( tx_buf, "This is my string counter %d", cnt); 
    		rfTxInfo.length= strlen(tx_buf) + 1;
				rfTxInfo.destAddr = 0x1215;
				rfTxInfo.cca = 0;
				rfTxInfo.ackRequest = 1;
				
				printf( "Sending\r\n" );
				if(zb_rf_tx_packet(&rfTxInfo) != 1)
					printf("--- RF_TX ERROR ---\r\n");
				cnt++;
		
				for(i=0; i<80; i++ )
					halWait(10000);
//				nrk_led_clr(GREEN_LED);
				for(i=0; i<20; i++ )
					halWait(10000);


}

void user_rx()
{
    uint8_t cnt,i,length;
    uint8_t n = 0;
				nrk_led_clr(GREEN_LED);
        /*
				rf_init (&rfRxInfo, 13, 0x2420, 0x1215);
        rf_set_rx (&rfRxInfo, 13); 	
				*/

				rf_polling_rx_on();
        while ((n = rf_rx_check_sfd()) == 0)
						continue; 
				nrk_led_set(GREEN_LED);
 				if (n != 0) {
        		n = 0;
        		// Packet on its way
    				cnt=0;
        		while ((n = rf_polling_rx_packet ()) == 0) {
								if (cnt > 5) {
                		//printf( "PKT Timeout\r\n" );
										break;		// huge timeout as failsafe
								}
        				halWait(10000);
								cnt++;
						}
    		}

				//rf_rx_off();
    		if (n == 1) {
    				nrk_led_clr(RED_LED);
        		// CRC and checksum passed
						printf("packet received\r\n");
						printf("SEQNUM: %d  SRCADDR: 0x%x  SNR: %d Ack: %d \r\n[",
								rfRxInfo.seqNumber, rfRxInfo.srcAddr, rfRxInfo.rssi, rfRxInfo.ackRequest);
        	
						for(i=0; i<rfRxInfo.length; i++ )
								printf( "%c", rfRxInfo.pPayload[i]);
						printf( "]\r\n\r\n" );
    		} 
				else if(n != 0){ 
						printf( "CRC failed!\r\n" ); nrk_led_set(RED_LED); 
				}
        
}

void zb_interdemo_main_loop()
{
    user_tx();
    user_rx();
    
}
