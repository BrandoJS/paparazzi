#include "i2c_payload.h"
// Downlink
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

uint8_t payloadstatus;
uint16_t capturenum;
uint8_t errorcode;
uint8_t state;
uint8_t code;

void i2cpayloadinit()
{
	payloadstatus = 0;
	capturenum = 0;
	errorcode = 0;
	gotpayloaddata = FALSE;
	i2c0_init_slave();
	state = 0;
	code = 0;
}

void checkI2CBuffer()
{
	uint8_t buffdata[I2C_BUF_LEN];
	uint8_t datacount = 0;
	uint16_t temp = 0;
	uint8_t bytesread = 0;

	if(i2cReceiveBufferIndex >= 7)
	{
		datacount = i2cReceiveBufferIndex;
		i2cReceiveBufferIndex = 0;

		for(int i = 0; i < datacount; i++)
			buffdata[i] = i2cReceiveBuffer[i];

		for(int i = 0; i < datacount; i++)
		{
			switch(state)
			{
			case 0:		//Reading code
				if(buffdata[i] == 1 || buffdata[i] == 2 || buffdata[i] == 3)
				{
					code = buffdata[i];
					state = 1;
					bytesread = 0;
				}
				break;
			case 1:			
				if(code == 1)	//Warm up and Cool down Byte
				{
					payloadstatus = buffdata[i];
					state = 0;
					bytesread = 1;
				}
				else if(code == 2)	//Capture Number
				{
					if(bytesread == 0)
					{
						temp = buffdata[i];
						bytesread = 1;
					}
					else
					{

						temp |= (((uint16_t)buffdata[i]) << 8);
						capturenum = temp;
						state = 0;
						bytesread = 2;
					}
				}
				else if(code == 3)	//Error Code
				{
					if(buffdata[i] > 0)
						errorcode = 1;
					else
						errorcode = 0;
					state = 0;
					bytesread = 1;
					gotpayloaddata = TRUE;
				}
				else
				{
					payloadstatus = 10;	
					state = 0;				
					gotpayloaddata = TRUE;
				}
				break;
			default:
				break;
			}
		}
	}
}

void sendPayloadData()
{
	if(gotpayloaddata)
	{
		DOWNLINK_SEND_AGGIECAP(DefaultChannel,DefaultDevice,&payloadstatus,&capturenum,&errorcode);
		gotpayloaddata = FALSE;
	}
}
