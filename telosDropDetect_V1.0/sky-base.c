/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: sky-collect.c,v 1.13 2010/09/14 06:47:08 adamdunkels Exp $
 */

/**
 * \file
 *         A program that collects statistics from a network of Tmote Sky nodes
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "net/netstack.h"
#include "net/rime.h"
#include "net/rime/collect.h"
#include "net/rime/collect-neighbor.h"
#include "net/rime/timesynch.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include "dev/light-sensor.h"
#include "dev/sht11-sensor.h"

#include "net/mac/frame802154.h"


#include "dev/cc2420.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "contiki-net.h"

#include "random.h"

#include "net/packetbuf.h"




typedef struct base_type
{
    uint16_t base_id;
    int16_t rssi_value;
}base_rssi_type;

struct sender_broadcast_msg {
  uint8_t header[3];
  uint8_t frame_id[2];
  uint8_t dummy;
  uint16_t data_length;
  uint16_t sender_id;
  uint8_t sender_event;
  uint8_t dummy2;
  base_rssi_type base_msg[2];
  uint8_t checksum;
};



struct base_broadcast_msg {
  uint8_t header[3];
  uint8_t frame_id[2];
  uint8_t dummy;
  uint16_t data_length;
  uint16_t bs_id;
  int16_t rssi_value;
  uint8_t checksum;
};



static uint8_t frame_temp = 0x01;
char base_buff[20];//tws



/*---------------------------------------------------------------------------*/
PROCESS(base_broadcast_process, "Base Broadcast");
AUTOSTART_PROCESSES(& base_broadcast_process);
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static inline uint16_t bswap_16(uint16_t v)  
{  
   return ((v & 0xff) << 8) | (v >> 8);  
}  
/*---------------------------------------------------------------------------*/

static void
broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
  
  struct sender_broadcast_msg *s_msg;
  
  int j;
  
  s_msg =packetbuf_dataptr();

  s_msg->data_length = bswap_16(s_msg->data_length);
  s_msg->sender_id = bswap_16(s_msg->sender_id);
  
  
  if (s_msg->frame_id[0] == 0x01){
  
      //printf("Broadcast Reveived from sender:");
//      printf(" %x %x %x %x %x %x %x %x ",
//         s_msg->header[0], s_msg->header[1], s_msg->header[2], s_msg->frame_id[0],
//         s_msg->frame_id[1], s_msg->data_length, s_msg->sender_id, s_msg->sender_event);

	  


	  printf(" %d %d %d %d %d %d %d %d %d %d ",
			   s_msg->header[0], s_msg->header[1], s_msg->header[2], s_msg->frame_id[0],
			   s_msg->frame_id[1], (s_msg->data_length)>>8, (s_msg->data_length)&0xff,  
			   (s_msg->sender_id)>>8, (s_msg->sender_id)&0xff, s_msg->sender_event);
	  
	


        for (j = 0; j < ((s_msg->data_length - 3)/4); ++j)
        {
        s_msg->base_msg[j].base_id = bswap_16(s_msg->base_msg[j].base_id);
		s_msg->base_msg[j].rssi_value = bswap_16(s_msg->base_msg[j].rssi_value);

		s_msg->base_msg[j].base_id = s_msg->base_msg[j].base_id & 0x00FF;
		
          printf("%d %d %d %d ", (s_msg->base_msg[j].base_id)>>8, (s_msg->base_msg[j].base_id)&0xff, 
		  	(s_msg->base_msg[j].rssi_value)>>8, (s_msg->base_msg[j].rssi_value)&0xff);
        }
      printf("%d\n", s_msg->checksum );
  }
  
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*----------------------------------------------------*/
uint8_t 
chkSumCalc( char buff[], int length )
{
    uint8_t chksum = 0;
	int calc_num = length;
	int calc_i;

        for( calc_i = 3; calc_i < calc_num; calc_i++ )
        {
            chksum += buff[calc_i];
        }

        chksum = ~chksum + 1;
        chksum %= 256;
    
    return chksum;
}

/*---------------------------------------------------------------------------*/

#define MAX(a, b) ((a) > (b)? (a): (b))
#define MIN(a, b) ((a) < (b)? (a): (b))
struct spectrum {
  int channel[16];
};
#define NUM_SAMPLES 4
static struct spectrum rssi_samples[NUM_SAMPLES];
static int
do_rssi(void)
{
	

  static int sample;
  int channel;
  
  NETSTACK_MAC.off(0);

  cc2420_on();
  for(channel = 11; channel <= 26; ++channel) {
    cc2420_set_channel(channel);
    rssi_samples[sample].channel[channel - 11] = cc2420_rssi() + 53;
  }
  
  NETSTACK_MAC.on();
  
  sample = (sample + 1) % NUM_SAMPLES;

  {
    int channel, tot;
    tot = 0;
    for(channel = 0; channel < 16; ++channel) {
      int max = -256;
      int i;
      for(i = 0; i < NUM_SAMPLES; ++i) {
	max = MAX(max, rssi_samples[i].channel[channel]);
      }
      tot += max / 20;
    }
    return tot;
  }
  
  
}
/*---------------------------------------------------------------------------*/
uint16_t 
toHash(uint8_t str[])
{
    uint16_t nVal = 0;
    uint32_t i_hash;

	uint16_t length_hash = sizeof(str);
	//printf("length= %d\n",length_hash);
    
        for(  i_hash = 0; i_hash < length_hash; i_hash++ )
        {
            //nVal += ( str[i_hash] << ( ( i_hash & 1 ) * 8 ) );
            nVal +=str[i_hash] ;
        }

		//printf("nVAL= %d\n",nVal); 
     if(nVal>255){
		nVal = nVal%256;}
    

    return nVal;
}



/*---------------------------------------------------------------------------*/
PROCESS_THREAD(base_broadcast_process, ev, data)
{
  static struct etimer et;

  //uint16_t rimeaddr_sky;   
  struct base_broadcast_msg *b_msg;   
  uint8_t longaddr[8];
  //int base_i;

  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

  PROCESS_BEGIN();


  broadcast_open(&broadcast, 29, &broadcast_call);

  while(1) {

    /* Delay 2-4 seconds */
    etimer_set(&et, CLOCK_SECOND * 4 + random_rand() % (CLOCK_SECOND * 4));

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		


	packetbuf_clear();
      
	  b_msg = (struct base_broadcast_msg *)packetbuf_dataptr();
      packetbuf_set_datalen(sizeof(struct base_broadcast_msg));

	  b_msg->header[0] = 0xFF;
	  b_msg->header[1] = 0xFF;
	  b_msg->header[2] = 0xFF;
	  b_msg->frame_id[0] = 0x00;

	if (frame_temp < 0xFF){
		b_msg->frame_id[1] = frame_temp;
	    frame_temp++;
		}
	  else {
		b_msg->frame_id[1] = frame_temp;
		frame_temp = 0x01;
	  }

	  b_msg->dummy = 0;

	  //b_msg->data_length = sizeof(struct base_broadcast_msg);

	  b_msg->data_length = 4;

	  //rimeaddr_sky = rimeaddr_node_addr.u8[0]+rimeaddr_node_addr.u8[1];
	  
	  //b_msg->bs_id = rimeaddr_sky;
    
    
    memset(longaddr, 0, sizeof(longaddr));
    rimeaddr_copy((rimeaddr_t *)&longaddr, &rimeaddr_node_addr);
 //   printf("MAC %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x \n",
 //          longaddr[0], longaddr[1], longaddr[2], longaddr[3],
 //          longaddr[4], longaddr[5], longaddr[6], longaddr[7]);
	//printf ("longlength = %d\n", sizeof(longaddr)) ;
 

    
	b_msg->bs_id = toHash(longaddr);
    //b_msg->bs_id = 15;
	//printf ("bs_id = %d\n", b_msg->bs_id) ;
	  

	  b_msg->rssi_value = do_rssi();

      //printf ("packetbuf = %u\n", b_msg[0]) ;

      memcpy(base_buff,b_msg,sizeof(struct base_broadcast_msg));

	  b_msg->checksum = chkSumCalc(base_buff, sizeof(struct base_broadcast_msg));


	  
//      for(base_i =0; base_i<20;base_i++){
	  //printf ("packetbuf = %u\n", base_buff[0]) ;
//	  printf("base_buff[%d]=%u ",base_i,base_buff[base_i]);
//	  }
//	  printf("\n");
    
	packetbuf_copyfrom(b_msg, sizeof(struct base_broadcast_msg));
    broadcast_send(&broadcast);
#if wst
    printf("broadcast message sent:");


	printf(" %d %d %d %d %d %d %d %d %d\n",
			   b_msg->header[0], b_msg->header[1], b_msg->header[2], b_msg->frame_id[0],
			   b_msg->frame_id[1], b_msg->data_length, b_msg->bs_id, b_msg->rssi_value, b_msg->checksum );
#endif	

	leds_on(LEDS_GREEN);
	etimer_set(&et, CLOCK_SECOND/5);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    leds_off(LEDS_ALL);

	//printf("%d.%d\n",
	//rimeaddr_node_addr.u8[0],rimeaddr_node_addr.u8[1]);
	//rimeaddr_sky = rimeaddr_node_addr.u8[0]+rimeaddr_node_addr.u8[1];
	//printf("rimeaddr_sky= %d\n",rimeaddr_sky);
	

  }
   PROCESS_END();
}
/*---------------------------------------------------------------------------*/



