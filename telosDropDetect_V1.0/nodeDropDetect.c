
#include "contiki.h"
#include "net/netstack.h"
#include "net/rime.h"
#include "net/rime/collect.h"
#include "net/rime/collect-neighbor.h"
#include "net/rime/timesynch.h"
#include "dev/leds.h"
#include "dev/cc2420.h"
#include "contiki-net.h"
#include "extSensor.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>



#define DropDetectDataBufLen 6
#define DropDetectThreshold 500
#define SenderDataBufLen 20

static int flag_drop=0;
static int flag_clear_cnt=0;
static int bs_id_cnt=0;
static uint8_t frame_temp = 0x01;

uint8_t sender_buff[SenderDataBufLen];
uint16_t tmpdataBuf[DropDetectDataBufLen]={0};




typedef struct base_type{
  uint16_t base_id;
  int16_t rssi_value;
}base_rssi_type;

typedef struct sender_broadcast_msg {
  uint8_t header[3];
  uint8_t frame_id[2];
  uint8_t dummy;
  uint16_t data_length;
  uint16_t sender_id;
  uint8_t sender_event;
  uint8_t dummy2;
  base_rssi_type base_msg[2];
  uint8_t checksum;
}sender_bro_msg;

typedef struct base_broadcast_msg {
  uint8_t header[3];
  uint8_t frame_id[2];
  uint8_t dummy;
  uint16_t data_length;
  uint16_t bs_id;
  int16_t rssi_value;
  uint8_t checksum;
}base_bro_msg;

packetbuf_attr_t rssi_s;
sender_bro_msg s_msg;


static int
bs_id_compare(base_bro_msg *msg_from_base)
{
	int j=0;
	for (j = 0; j < bs_id_cnt; j++)
	{
		if (s_msg.base_msg[j].base_id == msg_from_base->bs_id)
		{
			s_msg.base_msg[j].rssi_value = rssi_s;
			break;
		}
	}
	if (j == bs_id_cnt)
	{
		s_msg.base_msg[j].base_id = msg_from_base->bs_id;
		s_msg.base_msg[j].rssi_value = rssi_s;
		bs_id_cnt = j+1;
	}
	return 0;
}

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

static inline uint16_t
bswap_16(uint16_t v)  
{  
   return ((v & 0xff) << 8) | (v >> 8);  
}

static void
broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
  struct base_broadcast_msg *b_msg;

  b_msg =packetbuf_dataptr();
  b_msg->data_length = bswap_16(b_msg->data_length);
  b_msg->bs_id = bswap_16(b_msg->bs_id);
  b_msg->rssi_value = bswap_16(b_msg->rssi_value);
  
  if (b_msg->frame_id[0] == 0x00){
  b_msg->bs_id = b_msg->bs_id & 0x00FF; 
  printf("Broadcast Reveived from base: %d %d %d %d %d %d %d %d %d %d %d %d\n",
			   b_msg->header[0], b_msg->header[1], b_msg->header[2], b_msg->frame_id[0],
			   b_msg->frame_id[1], (b_msg->data_length)>>8, (b_msg->data_length)&0xff, 
			   (b_msg->bs_id)>>8, (b_msg->bs_id)&0xff, (b_msg->rssi_value)>>8, (b_msg->rssi_value)&0xff, b_msg->checksum );
  }
  rssi_s = packetbuf_attr(PACKETBUF_ATTR_RSSI);
  bs_id_compare(b_msg);
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;


/*---------------------------------------------------------------------------*/
PROCESS(sender_broadcast_process, "Sender Broadcast");
AUTOSTART_PROCESSES(&sender_broadcast_process);

/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sender_broadcast_process, ev, data)
{
  static struct etimer et;
  static int tmpdataCnt = 0;
  static uint16_t tmpdata=0;
  static uint32_t tmpdataSum = 0;
  uint16_t rimeaddr_sky;
  int nPos = 0;
  int nLen = 0;

  PROCESS_BEGIN();

  PROCESS_EXITHANDLER(broadcast_close(&broadcast));
  broadcast_open(&broadcast, 29, &broadcast_call);
  
  SENSORS_ACTIVATE(extSensor);

  while(1)
  {
    etimer_set(&et, CLOCK_SECOND / 10);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
  
    tmpdata=extSensor.value(1);
    tmpdataBuf[tmpdataCnt]=tmpdata;
  
    int j=0;
    for(j=0;j<DropDetectDataBufLen;j++)
    {
      tmpdataSum += tmpdataBuf[j];
    }
    if(tmpdataSum>DropDetectThreshold)
      flag_drop = 1;
    else
      flag_drop = 0;
    tmpdataSum = 0;
    tmpdataCnt++;
    if(tmpdataCnt==DropDetectDataBufLen)
      tmpdataCnt = 0;
    flag_clear_cnt++;
    if(flag_clear_cnt>=35)
    {
      s_msg.base_msg[0].rssi_value = -100;
      s_msg.base_msg[1].rssi_value = -100;
      flag_clear_cnt = 0;
    }
    
    if(flag_drop == 1)
    {
      packetbuf_clear();
      packetbuf_set_datalen(sizeof(struct sender_broadcast_msg));

      s_msg.header[0] = 0xFF;
      s_msg.header[1] = 0xFF;
      s_msg.header[2] = 0xFF;
      s_msg.frame_id[0] = 0x01;

      if (frame_temp < 0xFF){
      s_msg.frame_id[1] = frame_temp;
        frame_temp++;
      }
      else {
      s_msg.frame_id[1] = frame_temp;
      frame_temp = 0x01;
      }

      s_msg.dummy = 0;
      s_msg.data_length = bs_id_cnt *4 + 3;
      rimeaddr_sky = rimeaddr_node_addr.u8[0]+rimeaddr_node_addr.u8[1];
      s_msg.sender_id =1;
      s_msg.sender_event = flag_drop;
      s_msg.dummy2 = 0;

      nLen = sizeof( s_msg.header );
      memcpy( sender_buff, s_msg.header, nLen );
      nPos += nLen;
      nLen = sizeof( s_msg.frame_id );
      memcpy( sender_buff + nPos, s_msg.frame_id, nLen );
      nPos += nLen;
      nLen = sizeof( uint16_t );
      memcpy( sender_buff + nPos, &s_msg.data_length, nLen );
      nPos += nLen;
      nLen = sizeof( uint16_t );
      memcpy( sender_buff + nPos, &s_msg.sender_id, nLen );
      nPos += nLen;
      nLen = sizeof( uint8_t );
      memcpy( sender_buff + nPos, &s_msg.sender_event, nLen );
      nPos += nLen;
      nLen = sizeof( s_msg.base_msg );
      memcpy( sender_buff + nPos, &s_msg.base_msg, nLen );
      nPos += nLen;
      s_msg.checksum = chkSumCalc(sender_buff, nPos );

    
      packetbuf_copyfrom(&s_msg, sizeof(struct sender_broadcast_msg));
      broadcast_send(&broadcast);
      printf("broadcast message sent:");
      printf(" %d %d %d %d %d %d %d %d %d %d ",
            s_msg.header[0], s_msg.header[1], s_msg.header[2], s_msg.frame_id[0],
            s_msg.frame_id[1], (s_msg.data_length)>>8, (s_msg.data_length)&0xff,  
            (s_msg.sender_id)>>8, (s_msg.sender_id)&0xff, s_msg.sender_event);
      if (bs_id_cnt>0)
      {
        int j;
        for (j = 0; j < bs_id_cnt; ++j)
        {
          printf("%d %d %d %d ", (s_msg.base_msg[j].base_id)>>8,(s_msg.base_msg[j].base_id)&0xff,  
                (uint8_t)(s_msg.base_msg[j].rssi_value)>>8, (uint8_t)(s_msg.base_msg[j].rssi_value)&0xff);
        }
      }
      printf("%d\n", s_msg.checksum );

      leds_on(LEDS_GREEN);
      etimer_set(&et, CLOCK_SECOND/5);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
      leds_off(LEDS_ALL);        
      }
    }
  
  PROCESS_END();
}