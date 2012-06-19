#include "i2c_payload.h"

uint8_t payloadstatus;
uint16_t capturenum;
uint8_t errorcode;

void i2cpayloadinit()
{
	payloadstatus = 0;
	capturenum = 0;
	errorcode = 0;
	gotpayloaddata = FALSE;
}

extern bool_t GotPayloadData()
{
	if(gotpayloaddata)
	{
		gotpayloaddata = FALSE;
		return TRUE;
	}
	else
		return FALSE;
}
