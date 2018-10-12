/*********************************************************************************
 *      Copyright:  (C) 2016 Guo Wenxue<guowenxue@gmail.com>
 *                  All rights reserved.
 *
 *       Filename:  main.c
 *    Description:  LoRa����ģ����ں���
 *                 
 *        Version:  1.0.0(09/28/2014)
 *         Author:  Guo Wenxue <guowenxue@gmail.com>
 *      ChangeLog:  1, Release initial version on "08/12/2016 10:58:17 PM"
 *                 
 ********************************************************************************/

#include <stdio.h>
#include <string.h>
#include "stm8_board.h"
#include "timing_delay.h"
#include "atcmd_proc.h"
#include "conf_eeprom.h"
#include "sx1276-LoRa.h"
#include "sx1276-LoRaMisc.h"
#include "radio.h"
#include "crc-itu-t.h"
#include "sensor_proc.h"

extern tRadioDriver *sx1278_radio;
void proc_lora_recv(void);
int fill_ack_pack_t(uint8_t *pack, uint16_t netid, uint8_t devid, uint8_t ack_flag);

int main( void )
{
  int      status;               /* RF process status */

  sys_board_init();
  msleep(20);      /* �ȴ� windows USBת����׼���� */   
  print_lora_settings();
  
  check_run_atcmd(); /* AT��������ｫ�ر����еĵ� */

  lora_init();
  sx1278_radio->StartRx();
  while(1)
  {
    status=sx1278_radio->Process(); 
     switch( status )
     {
        case RF_TX_DONE:
          sx1278_radio->StartRx();
          break;
          
        case RF_RX_DONE:
          proc_lora_recv();
          break;
          
        default:
          break;
     }

  } /* end of while(1) */
}

void proc_lora_recv(void)
{
  uint16_t                  crc = 0;
  uint16_t                  new_packet_len = 0;  
  uint16_t                  RxPacketSize;         /* RF receive pakcet size */
  uint8_t                   ack[ACK_PACK_LEN];
  uint8_t                   *RxBuffer = RFBuffer; /* RxBuffer always point to RFBuffer, which defined in sx1276-LoRa.c */  
  lora_pack_head_t          *pack = (lora_pack_head_t *)RxBuffer;
  uint8_t                   ack_flag;
 
  SX1276LoRaGetRxPacket(NULL, &RxPacketSize); /* Just get the RxPacketSize */
  if(RxPacketSize > 0)  
  {
    turn_led(LED_STATUS, ON);
#if 1   
    dbg_print("LoRa recv [%d] byte data: ", RxPacketSize);
    dump_buf(RxBuffer, RxPacketSize); 
#endif

    /* ���յ������ݰ�̫С */
    if(RxPacketSize<LORA_PACK_MINSIZE) 
    {
      dbg_print("ERROR: eLoRa receive packet size is too small<%d>\n", RxPacketSize);
      turn_led(LED_STATUS, OFF);
      return ;
    } 
    
 
    /* ����ID��ƥ������ݰ������������������ݰ� */
    if(pack->netid!=g_sys_eep_conf.settings.netid || RxPacketSize<pack->len )
    {
      dbg_print("ERROR: netid don't match\n");
      turn_led(LED_STATUS, OFF);
      return ;
    }    
     
    /* �����µı��ĳ���Ĭ��ֵΪԭ���ĵĳ��ȣ�֮�����ǽ��Ա��Ľ����޸� */
    new_packet_len = pack->len;
          
    /* ����յ��ı���CRC�Ƿ�Ϸ�:
     * ���յ���ԭʼ������������ֽ���CRCֵ����CRC��������ͷ�ʹ���������
     */
    crc = bytes_to_ushort( &RxBuffer[pack->len-2], 2); 
    if( crc != crc_itu_t(LORA_MAGIC_CRC, RxBuffer, pack->len-2) )
    {
      ack_flag = 0;
         /* ��װNAK���ٽ������� */  
     fill_ack_pack_t(ack, pack->netid, pack->addr, ack_flag);
      /* CRCУ���������ԭ����ͷ��ǰ���һ���ֽ�0xFD */ 
      memmove(&RxBuffer[1], RxBuffer, pack->len);
      RxBuffer[0]=ELORA_ERROR_TAG;    
      new_packet_len ++;
    }
    else
    {
      ack_flag = 1;
          /* ��װACK���ٽ������� */  
     fill_ack_pack_t(ack, pack->netid, pack->addr, ack_flag);
      /* ��ԭ���ݰ����һ���ֽں����RSSIֵ */
      RxBuffer[pack->len] = SX1276LoRaGetRssiReg();
      new_packet_len ++;

      /*���¼���ԭ���ݰ���RSSI��CRCֵ������ӵ����ݰ�����ȥ */
      crc = crc_itu_t(LORA_MAGIC_CRC, RxBuffer, new_packet_len);  
      ushort_to_bytes(&RxBuffer[pack->len+1], crc);
      new_packet_len += ELORA_PACK_CRC_LEN;
    }

    dbg_uart_send(RxBuffer, new_packet_len);
    memset(RxBuffer, 0, new_packet_len);
    
    /* reply ack or nak */
    SX1276LoRaSetTxPacket(ack, ACK_PACK_LEN);
    SX1276StartTx();
    
    turn_led(LED_STATUS, OFF);    
  }
   sx1278_radio->StartRx();
}


int fill_ack_pack_t( uint8_t *pack, uint16_t netid, uint8_t devid, uint8_t ack_flag)
{
  lorap_pack_t *ack = (lorap_pack_t *)pack;
  
  if( NULL == pack)
    return 0;
  
  memset(ack, 0, ACK_PACK_LEN);
  
  if(ack_flag)
    ack->tag =  LORAP_ACK_TAG;
  else 
    ack->tag =  LORAP_NAK_TAG;
  
    ack->netid = netid;
    ack->devid = devid;
    ack->cmd = LORAP_CMD_ACK;
    ack->crc = crc_itu_t(LORA_MAGIC_CRC, (uint8_t *)ack, ACK_PACK_LEN - ELORA_PACK_CRC_LEN);  
    //printf("the [tag] [netid] [devid] : %u %hu %u\n", ack->tag,ack->netid,ack->devid);
    
    return 1;
}

