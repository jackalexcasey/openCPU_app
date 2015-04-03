
/*****************************************************************************
 *  Copyright Statement:
 *  --------------------
 *  This software is protected by Copyright and the information contained
 *  herein is confidential. The software may not be copied and the information
 *  contained herein may not be used or disclosed except with the written
 *  permission of Quectel Co., Ltd. 2013
 *
 *****************************************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   main.c
 *
 * Project:
 * --------
 *   OpenCPU
 *
 * Description:
 * ------------
 *   This app demonstrates how to send AT command with RIL API, and transparently
 *   transfer the response through MAIN UART. And how to use UART port.
 *   Developer can program the application based on this example.
 * 
 ****************************************************************************/
#ifdef __CUSTOMER_CODE__
#include "custom_feature_def.h"
#include "ril.h"
#include "ril_util.h"
#include "ril_telephony.h"
#include "ql_stdlib.h"
#include "ql_error.h"
#include "ql_trace.h"
#include "ql_uart.h"
#include "ql_system.h"

#include <string.h>
#include "ql_type.h"
//#include "ql_trace.h"
#include "ql_timer.h"
//#include "ql_uart.h"
//#include "ql_stdlib.h"

#include"power_ctl.h"



void proc_main_task(s32 taskId)
{
	s32 ret;
	ST_MSG msg;

	Ql_Debug_Trace(" OpenCPU: Customer Application For Power Test\r\n");

	application_init();
	// START MESSAGE LOOP OF THIS TASK
	while(TRUE)
	{
		Ql_OS_GetMessage(&msg);
		switch(msg.message)
		{
			case MSG_ID_RIL_READY:
				Ql_Debug_Trace("<-- RIL is ready -->\r\n");
				Ql_RIL_Initialize();
				break;
			case MSG_ID_URC_INDICATION:
				Ql_Debug_Trace("<-- Received URC: type: %d, -->\r\n", msg.param1);
				switch (msg.param1)
				{
					case URC_SYS_INIT_STATE_IND:

						Ql_Debug_Trace("<-- Sys Init Status %d -->\r\n", msg.param2);
						if(3 == msg.param2)
						{
							//app_start_AT_check();	
							start_power_test();
						}
						break;
					case URC_SIM_CARD_STATE_IND:
						Ql_Debug_Trace("<-- SIM Card Status:%d -->\r\n", msg.param2);
						break;
					case URC_GSM_NW_STATE_IND:
						Ql_Debug_Trace("<-- GSM Network Status:%d -->\r\n", msg.param2);
						if(0 == msg.param2)
						{
							//app_start_AT_check();	
						}
						break;
					case URC_GPRS_NW_STATE_IND:
						Ql_Debug_Trace("<-- GPRS Network Status:%d -->\r\n", msg.param2);
						break;
					case URC_CFUN_STATE_IND:
						Ql_Debug_Trace("<-- CFUN Status:%d -->\r\n", msg.param2);
						break;
					case URC_COMING_CALL_IND:
						{
							ST_ComingCall* pComingCall = (ST_ComingCall*)msg.param2;
							Ql_Debug_Trace("<-- Coming call, number:%s, type:%d -->\r\n", pComingCall->phoneNumber, pComingCall->type);
							break;
						}
					case URC_CALL_STATE_IND:
						Ql_Debug_Trace("<-- Call state:%d\r\n", msg.param2);
						break;
					case URC_NEW_SMS_IND:
						Ql_Debug_Trace("<-- New SMS Arrives: index=%d\r\n", msg.param2);
						break;
					case URC_MODULE_VOLTAGE_IND:
						Ql_Debug_Trace("<-- VBatt Voltage Ind: type=%d\r\n", msg.param2);
						break;
					default:
						Ql_Debug_Trace("<-- Other URC: type=%d\r\n", msg.param1);
						break;
				}
				break;
			default:
				break;
		}
	}
}




#endif // __CUSTOMER_CODE__
