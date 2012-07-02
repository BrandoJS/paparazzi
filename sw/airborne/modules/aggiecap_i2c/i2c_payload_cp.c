#include "i2c_payload_cp.h"
// Downlink
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#include <string.h>

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

uint8_t payloadstatus;
uint8_t total_packets;
uint8_t packet_num;
uint8_t data_size;
uint64_t link_data[8]; //64 bytes of link data
uint8_t link_data_s[AGCP_LINK_DATA_SIZE+1];
uint8_t errorcode;

void i2cpayloadinit_cp()
{
	payloadstatus = 0;
	errorcode = 0;
	gotpayloaddata = FALSE;
	i2c0_init_slave();
}

void checkI2CBuffer_cp()
{
	uint8_t buffdata[I2C_BUF_LEN];
	//uint8_t datacount = 0;
	uint16_t i;
	//uint8_t linkbytes = 0, linkindex = 0;

	if(i2c0_circ_buff_size() >= AGCP_LINK_PKT_SIZE)
	{
		//nom all the bytes of the packet
		for (i = 0; i < AGCP_LINK_PKT_SIZE; i++)
			 i2c0_circ_read(&buffdata[i]);

		//5 byte for payload status and meta data for the rest of the packet
		payloadstatus = buffdata[0];
		packet_num = buffdata[1];
		total_packets = buffdata[2];
		data_size = buffdata[3];
		if(data_size > AGCP_LINK_DATA_SIZE)
		{
			data_size = AGCP_LINK_DATA_SIZE;
		}
		errorcode = buffdata[4];
		
		//64 bytes for payload link data
		memcpy(link_data_s, (buffdata+5), data_size);
		
		gotpayloaddata = TRUE;
		
	}
}

void sendPayloadData_cp()
{
	if(gotpayloaddata)
	{
		//DOWNLINK_SEND_AGGIECAP_CP(DefaultChannel,DefaultDevice,&payloadstatus,&packet_num,&total_packets,&data_size,&link_data[0],&link_data[1],&link_data[2],&link_data[3],&link_data[4],&link_data[5],&link_data[6],&link_data[7],&errorcode);
		DOWNLINK_SEND_AGGIECAP_LINK(DefaultChannel,DefaultDevice,&payloadstatus,&packet_num,&total_packets,&data_size,&errorcode,data_size,link_data_s);
		gotpayloaddata = FALSE;
	}
}
