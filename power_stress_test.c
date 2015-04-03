#ifdef __CUSTOMER_CODE__

#include <stdlib.h>
#include <stdio.h>
#include "custom_feature_def.h"
#include "ril.h"
#include "ril_util.h"
#include "ril_telephony.h"
#include "ril_system.h"

#include "ql_stdlib.h"
#include "ql_error.h"
#include "ql_trace.h"
#include "ql_uart.h"
#include "ql_system.h"
#include <string.h>
#include "ql_type.h"
//#include "ql_trace.h"
#include "ql_timer.h"
#include "power_ctl.h"



static bool flag_creg_success = FALSE;
static bool flag_cgreg_success = FALSE;

data_frame_type uart_inter = {0};//ATC control data

//u8 (uart_inter.rx_buf)[SERIAL_RX_BUFFER_LEN] = {0};


u8 RxBuf_Uart_Console[SERIAL_RX_BUFFER_LEN] = {0};

nvram_value_struct g_nv_value = {0};

unsigned int RANGE_MIN = 2000;

unsigned int RANGE_MAX = 6000;


timer_struct_type power_ctl_timer = 
{0};
timer_struct_type atc_timer = 
{0};

timer_struct_type at_sync_timer = 
{0};

timer_struct_type power_switch_timer = 
{0};

led_struct_type led = 
{0};
timer_struct_type powerkey_signal_timer = 
{0};
timer_struct_type throwout_error_timer = 
{0};

timer_struct_type system_timer = 
{0};

sys_info_statistic_type sys_info =
{0};



/**************************************************************
 * Global Param
 **************************************************************/
static u32  g_victor_EventGrpId = 0;
#define  TEST_MUTEX_NAME       "TEST_MUTEX"
#define  TEST_EVTGRP_NAME      "TEST_EVETNGRP"
typedef enum {
	TEST_EVENT_NOWAIT       = (0x00000000),
	TSET_EVENT_AT_RESPONSE  = (0x00000001),
	TEST_EVENT_AT_CALLBACK  = (0x00000002)
}Enum_TEST_WaitEvent;




//uart
static   void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg
, bool level, void* customizedPara);
static   s32 ATResponse_Handler(char* line, u32 len, void* userData);
static   void uart_init(Enum_SerialPort port);
static   void data_frame_init(data_frame_type *p_frame);
/*app*/
//void power_ctl(bool on_off);
static   void app_start_AT_check(void);
static   bool set_ipr(void);
static   bool send_ATC(unsigned char * p_str);
static   bool check_power_off_wait_time(char * str,char * p_min,char *p_max);
//time range:2,5
static   unsigned char check_gsm_network_status(char * str);
static   unsigned char check_gprs_network_status(char * str);
static   bool check_IMEI(char * str);
static   bool check_clvl(char * str);
//bool get_cmee(char * str,u8 * p_out);
static   bool check_cmee(char * str);
//bool get_nv_unit(char * str_src,char * str_search,u8 * p_out);
static   bool check_nv_unit(char * str_src,char * str_search,u8 *nv_value_now);
static   bool check_nv_apn(char *str_src,char * str_check);
static   bool check_qmic(char * str,qmic_struct * qmic_nv_now);
static   bool check_mode(char * str,u8 * mode_out);
static   bool set_mode(u8 mode);
static   bool get_mode(u8 *mode_out);
//error
static   void sys_hangup(void);
static   void start_throwout_error(Enum_error e_error);
static   void throwout_error(Enum_error e_error);
//led
static   void led_flicker(bool fast_slow);
static   void gpio_reverse(Enum_PinName gpiopin);
static   void led_ctl(bool on_off);
//for power control 
static   void power_pwren_VBAT_ctl(bool on_off);
static   void powerkey_ctl(bool flag);
static   void power_ctl_genneral(bool on_off);
static   void pwr_reset_pre(void);
//for check slave device status
static   bool init_gpio_get_status(void);
static   bool get_slave_device_status(Enum_PinLevel * p_level_out);
static   bool timer_init(timer_struct_type *p_timer);
static   void Timer_handler(u32 timerId, void* param);
static   void Timer_handler_led(u32 timerId, void* param);
static   bool timer_start(timer_struct_type * p_timer);
static   void Timer_handler_ATC_timerout(u32 timerId, void* param);
static   void Timer_handler_AT_sync_timerout(u32 timerId, void* param);
static   void Timer_handler_powerswitch_timerout(u32 timerId, void* param);
static   void all_timer_stop(void);
//gpio
static   void gpio_init(Enum_PinName  gpioPin);
static   void gpio_set(Enum_PinName  gpioPin,Enum_PinLevel gpioLvl);
//tools
static   unsigned int random(void);
static   u8 check_ok_num(char * str);








static void timer_data_init(timer_struct_type *p_timer)
{
	if(p_timer == &power_ctl_timer )
	{
		p_timer->id = 0x102;
		p_timer->interval = 2000;
		p_timer->callback_onTimer = Timer_handler;
		p_timer->param = (void*)0;
		p_timer->auto_repeat = FALSE;
		//p_timer->start = timer_start;
	}
	else if(p_timer == &(led.timer))
	{
		p_timer->id = 0x101;
		p_timer->interval = 1000;
		p_timer->callback_onTimer = Timer_handler_led;
		p_timer->param = (void*)0;
		p_timer->auto_repeat = FALSE;
		//p_timer->start = timer_start;

	}
	else if(p_timer == &powerkey_signal_timer )
	{
		p_timer->id = 0x104;
		p_timer->interval = 2000;
		p_timer->callback_onTimer = Timer_handler;
		p_timer->param = (void*)0;
		p_timer->auto_repeat = FALSE;
		//p_timer->start = timer_start;
	}
	else if(p_timer == &throwout_error_timer )
	{
		p_timer->id = 0x105;
		p_timer->interval = 2000;
		p_timer->callback_onTimer = Timer_handler;
		p_timer->param = (void*)0;
		p_timer->auto_repeat = TRUE;
		//p_timer->start = timer_start;
	}
	else if(p_timer == &system_timer )
	{
		p_timer->id = 0x103;
		p_timer->interval = 1000;// 1s
		p_timer->callback_onTimer = Timer_handler;
		p_timer->param = (void*)0;
		p_timer->auto_repeat = TRUE;
		//p_timer->start = timer_start;
	}
	else if(p_timer == &atc_timer)
	{
		p_timer->id = 0x106;
		p_timer->interval = 10000;// 10s
		p_timer->callback_onTimer = Timer_handler_ATC_timerout;
		p_timer->param = (void*)0;
		p_timer->auto_repeat = FALSE;//ONCE
		//p_timer->start = timer_start;
	}
	else if(p_timer == &at_sync_timer)
	{
		p_timer->id = 0x107;
		p_timer->interval = 500;// 500ms
		p_timer->callback_onTimer = Timer_handler_AT_sync_timerout;
		p_timer->param = (void*)0;
		p_timer->auto_repeat = FALSE;//ONCE
		//p_timer->start = timer_start;
	}
	else if(p_timer == &power_switch_timer)
	{
		p_timer->id = 0x108;
		p_timer->interval = 20000;// 20s
		p_timer->callback_onTimer = Timer_handler_powerswitch_timerout;
		p_timer->param = (void*)0;
		p_timer->auto_repeat = FALSE;//ONCE
		//p_timer->start = timer_start;
	}


}

static bool led_data_init(led_struct_type * p_led)
{
	timer_data_init(&(p_led->timer));
	p_led->flag_fast_slow = FALSE;//slow

	return timer_init(&(p_led->timer));
}

static void system_info_data_init(void)
{
	Ql_memset(&sys_info, 0x0, sizeof(sys_info_statistic_type));
	sys_info.status = SYS_STATE_IDLE;
	sys_info.error_type = ERROR_UNKNOWN;
	sys_info.mode  = MODE_POWD_PWR_EN;//init as
	sys_info.at_sync_cnt = 0;


	sys_info.system_timer_cnt = 0;
	sys_info.network_error_cnt = 0;
	sys_info.system_test_runtime_cnt = 0;//init as 0
	sys_info.creg_network_error_cnt = 0;
	sys_info.cgreg_network_error_cnt = 0;

}

static void data_frame_init(data_frame_type *p_frame)
{
	Ql_memset(p_frame, 0x0, sizeof(data_frame_type));

}
void application_init(void)
{

	flag_creg_success = FALSE;
	flag_cgreg_success = FALSE;
	Ql_memset(&g_nv_value, 0x0, sizeof(nvram_value_struct));

	data_frame_init(&uart_inter);
	system_info_data_init();


	//s_error = ERROR_UNKNOWN;


	uart_init(UART_PORT_INTER);
	uart_init(UART_PORT_CONSOLE);
	gpio_init(PINNAME_RI);
	gpio_init(PINNAME_NETLIGHT);
	gpio_init(PINNAME_DCD);//for powerkey (set D/L off first!)


	init_gpio_get_status();


	//timer_init();
	timer_init(&power_ctl_timer);
	led_data_init(&led);
	timer_init(&powerkey_signal_timer);
	timer_init(&throwout_error_timer);
	timer_init(&system_timer);
	timer_init(&atc_timer);
	timer_init(&at_sync_timer);
	timer_init(&power_switch_timer);

	g_victor_EventGrpId = Ql_OS_CreateEvent(TEST_EVTGRP_NAME);



#if 0
	s_mode = get_mode();
	if(s_mode ==  255)
	{
		s_mode = MODE_POWD_PWR_EN;
	}
	//s_mode = MODE_POWER_KEY;
#endif	
	//led_ctl(FALSE);//TURN OFF
	//led_flicker(FALSE);
}

void application_deinit(void)
{
	system_info_data_init();

	flag_creg_success = FALSE;
	flag_cgreg_success = FALSE;


}
#if 0
void start_power_test_case1(void)
{
	if(PINLEVEL_HIGH == Ql_GPIO_GetLevel(PINNAME_RI))//already power on
	{
		set_ipr();
		sys_info.status = SYS_STATE_IPR_SET;
#if 0
		power_ctl(FALSE);	//POWER OFF
		sys_info.status = SYS_STATE_POWER_OFF;
		Ql_Timer_Start(power_ctl_timer_id,power_ctl_interval,TRUE);
#endif
	}
	else
	{
		power_ctl(TRUE);	//POWER ON
		sys_info.status = SYS_STATE_POWER_ON;
		//power_ctl_interval = random();
		Ql_Timer_Start(power_ctl_timer_id,500,TRUE);
		//sys_info.status = SYS_STATE_IDLE;
	}
}
void start_power_test_case2(void)
{
	//powerkey_ctl(0);//on
	sys_info.status = SYS_STATE_POWER_ON;
	power_ctl(TRUE);
}

#endif

void start_power_test(void)
{
	Enum_PinLevel slave_status_pin_level = PINLEVEL_HIGH;

	if(FALSE ==  get_mode(&(sys_info.mode)))
	{
		sys_info.mode = MODE_POWD_PWR_EN;//set default value
	}



	if(get_slave_device_status(&slave_status_pin_level))
	{
		if(slave_status_pin_level == PINLEVEL_HIGH)//slave dev is power on 
		{

		}
		else
		{

		}


	}


	led_flicker(FALSE);


	if(sys_info.mode == MODE_POWD_PWR_EN)
	{
		//start_power_test_case1();

		if( PINLEVEL_HIGH == slave_status_pin_level )
		{
			//no need AT sync
			//sys_info.system_test_runtime_cnt++;
			//set_ipr();
			//sys_info.status = SYS_STATE_IPR_SET;
#ifdef TEST_OBJECT_UC15_NO_AT
			Ql_Debug_Trace("<-- power on -->\r\n");
			power_switch_timer.interval = 20000;//  20s
			power_switch_timer.auto_repeat = FALSE;
			timer_start(&power_switch_timer);
#else

			start_at_sync();
#endif
#if 0
			power_ctl(FALSE);	//POWER OFF
			sys_info.status = SYS_STATE_POWER_OFF;
			Ql_Timer_Start(power_ctl_timer_id,power_ctl_interval,TRUE);
#endif
		}
		else
		{

			power_pwren_VBAT_ctl(TRUE);	//POWER ON
			Ql_Debug_Trace("<-- power on -->\r\n");
			sys_info.status = SYS_STATE_POWER_ON;
			//power_ctl_interval = random();
#ifdef TEST_OBJECT_UC15_NO_AT

			power_switch_timer.interval = 20000;//  20s
			power_switch_timer.auto_repeat = FALSE;
			timer_start(&power_switch_timer);
#else
			start_at_sync();
#endif

			//power_ctl_timer.interval = 500;
			//power_ctl_timer.auto_repeat = TRUE;
			//timer_start(&power_ctl_timer);
			//Ql_Timer_Start(power_ctl_timer_id,power_ctl_timer.interval,power_ctl_timer.auto_repeat);//used to  send AT sync

		}


	}
	else if(sys_info.mode == MODE_POWER_KEY)
	{
		//start_power_test_case2();

		if( PINLEVEL_HIGH == slave_status_pin_level )
		{

			powerkey_ctl(FALSE);//SEND POWER OFF SIGNAL 

			sys_info.status = SYS_STATE_POWER_OFF;
		}
		else
		{
			powerkey_ctl(TRUE);//SEND POWER ON SIGNAL 

			sys_info.status = SYS_STATE_POWER_ON;
		}
	}


}
void start_at_sync(void)
{
	//*((s32*)param) = 0;
#if 0
	s32 ret;
	ret = Ql_Timer_Stop(power_ctl_timer_id);
	if(ret < 0)
	{
		Ql_Debug_Trace("\r\n<--failed!! stack timer Ql_Timer_Stop ret=%d-->\r\n",ret);           
	}
	Ql_Debug_Trace("\r\n<--stack timer Ql_Timer_Stop(ID=%d,) ret=%d-->\r\n",power_ctl_timer_id,ret); 
#endif

	Ql_Debug_Trace("\r\n<-- POWER ON NOW -->\r\n");
	Ql_UART_Write(UART_PORT_CONSOLE, "POWER ON\r\n", Ql_strlen("POWER ON\r\n"));

	sys_info.status = SYS_STATE_POWER_ON;
	data_frame_init(&uart_inter);//clean
	Ql_Timer_Stop(power_ctl_timer.id);




	at_sync_timer.interval = 500;//0.5s REPEAT send AT
	at_sync_timer.auto_repeat = TRUE;
	timer_start(&at_sync_timer);


	//Ql_Timer_Start(power_ctl_timer_id,500,TRUE);//
	//sys_info.status = SYS_STATE_IDLE;


}

bool send_ATC(unsigned char * p_str)
{
	bool ret  = FALSE;
	ret = Ql_Timer_Stop(atc_timer.id);
	data_frame_init(&uart_inter);//clean
	Ql_UART_Write(UART_PORT_INTER, p_str, Ql_strlen(p_str));

	atc_timer.interval = 10000;
	atc_timer.auto_repeat = FALSE;
	ret =  timer_start(&atc_timer);

	return ret;
}


void gpio_init(Enum_PinName  gpioPin)
{
	// Specify a GPIO pin
	//Enum_PinName  gpioPin = PINNAME_NETLIGHT;

	// Define the initial level for GPIO pin
	Enum_PinLevel gpioLvl = PINLEVEL_HIGH;

	// Initialize the GPIO pin (output high level, pull up)
	Ql_GPIO_Init(gpioPin, PINDIRECTION_OUT, gpioLvl, PINPULLSEL_PULLUP);

#if 0	
	Ql_Debug_Trace("<-- Initialize GPIO pin (PINNAME_STATUS): output, high level, pull up -->\r\n");

	// Get the direction of GPIO
	Ql_Debug_Trace("<-- Get the GPIO direction: %d -->\r\n", Ql_GPIO_GetDirection(gpioPin));

	// Get the level value of GPIO
	Ql_Debug_Trace("<-- Get the GPIO level value: %d -->\r\n\r\n", Ql_GPIO_GetLevel(gpioPin));
#endif

}


/*
   use RI PIN as pwrkey
   */
void gpio_init_pwrkey_emerg_off(void)
{

}

void gpio_set(Enum_PinName  gpioPin,Enum_PinLevel gpioLvl)
{
	// Set the GPIO level to high after 500ms.
	//Ql_Debug_Trace("<-- Set the GPIO level to high after 500ms -->\r\n");
	//Ql_Sleep(500);
	//Ql_GPIO_SetLevel(gpioPin, gpioLvl);
	if ( gpioLvl != Ql_GPIO_GetLevel(gpioPin))
	{
		Ql_GPIO_SetLevel(gpioPin, gpioLvl);
	}
	//Ql_Debug_Trace("<-- Get the GPIO level value: %d -->\r\n", Ql_GPIO_GetLevel(gpioPin));

}

void gpio_reverse(Enum_PinName gpiopin)
{
	if ( PINLEVEL_HIGH == Ql_GPIO_GetLevel(gpiopin))
	{
		Ql_GPIO_SetLevel(gpiopin, PINLEVEL_LOW);
	}
	else
	{
		Ql_GPIO_SetLevel(gpiopin, PINLEVEL_HIGH);
	}
}
void power_pwren_VBAT_ctl(bool on_off)
{

	if(on_off)
	{
		gpio_set(PINNAME_RI,PINLEVEL_HIGH);
	}
	else
	{
		gpio_set(PINNAME_RI,PINLEVEL_LOW);
	}

}

void power_ctl_genneral(bool on_off)
{
	if(sys_info.mode == MODE_POWD_PWR_EN)
	{
		if(on_off)
		{
			gpio_set(PINNAME_RI,PINLEVEL_HIGH);
		}
		else
		{
			gpio_set(PINNAME_RI,PINLEVEL_LOW);
		}
	}
	else if(sys_info.mode == MODE_POWER_KEY)
	{
		powerkey_ctl(on_off);
	}
}

void led_ctl(bool on_off)
{
	if(on_off)
	{
		gpio_set(PINNAME_NETLIGHT,PINLEVEL_HIGH);
	}
	else
	{
		gpio_set(PINNAME_NETLIGHT,PINLEVEL_LOW);
	}

}
/*
   NOW_STATUS  OP             result

   OFF         low:>1s  high   on
   ON          low:0.7s high   off
   ON          low:1.5s high   restart	
   */
void powerkey_ctl(bool flag)
{
	s32 ret = 0;
	if(flag == TRUE)//on
	{
		//gpio_set(PINNAME_NETLIGHT,PINLEVEL_HIGH);
		gpio_set(PINNAME_DCD,PINLEVEL_LOW);

		powerkey_signal_timer.interval = 1500;// power on signal
		powerkey_signal_timer.auto_repeat = FALSE;// ONCE

		//ret = Ql_Timer_Start(powerkey_timer_id,1500,FALSE);
		Ql_Debug_Trace("\r\n<-- PINNAME_DCD PINLEVEL_LOW 1500 -->\r\n");

		if(!timer_start(&powerkey_signal_timer))
		{
			//Ql_Debug_Trace("\r\n<-- failed!! powerkey_timer_id Ql_Timer_Start fail, ret=%d -->\r\n",ret);
		}
	}
	else //(flag == FALSE)//off
	{
		gpio_set(PINNAME_DCD,PINLEVEL_LOW);

		powerkey_signal_timer.interval = 800;// power off signal
		powerkey_signal_timer.auto_repeat = FALSE;// ONCE




		//ret = Ql_Timer_Start(powerkey_timer_id,800,FALSE);// ONCE
		Ql_Debug_Trace("\r\n<-- PINNAME_DCD PINLEVEL_LOW 800 -->\r\n");
		if(!timer_start(&powerkey_signal_timer))
		{
			//Ql_Debug_Trace("\r\n<-- failed!! powerkey_timer_id Ql_Timer_Start fail, ret=%d -->\r\n",ret);
		}


	}
#if 0
	else if(flag == 2)//restart
	{
		gpio_set(PINNAME_DCD,PINLEVEL_LOW);
		ret = Ql_Timer_Start(powerkey_timer_id,3000,FALSE);// ONCE
		if(ret < 0)
		{
			Ql_Debug_Trace("\r\n<-- failed!! powerkey_timer_id Ql_Timer_Start fail, ret=%d -->\r\n",ret);
		}
	}
#endif

}

void pwr_reset_pre(void)
{
	if(sys_info.mode == MODE_POWD_PWR_EN)
	{
		//Ql_Debug_Trace("<-- power off -->\r\n");
		sys_info.status = SYS_STATE_QPOWD;
#ifdef TEST_OBJECT_GW
		send_ATC( "AT$MYPOWEROFF\r\n");
		//send_ATC( "ATxxxx\r\n");
#else
		send_ATC( "AT+QPOWD=1\r\n");
#endif
		Ql_Timer_Stop(atc_timer.id);// no need response 
		//power_ctl(FALSE);


		//start a timer,repeat=true;
		//power_ctl_interval = random();
		//power_ctl_interval = 1000;

		power_ctl_timer.interval = random();
		if(power_ctl_timer.interval>=6000)
		{
			sys_info.random_cnt.sec_6++;
		}
		else if(power_ctl_timer.interval>=5000)
		{
			sys_info.random_cnt.sec_5++;
		}
		else if(power_ctl_timer.interval>=4000)
		{
			sys_info.random_cnt.sec_4++;
		}
		else if(power_ctl_timer.interval>=3000)
		{
			sys_info.random_cnt.sec_3++;
		}
		else if(power_ctl_timer.interval>=2000)
		{
			sys_info.random_cnt.sec_2++;
		}
		else
		{
			sys_info.random_cnt.invalid++;
		}

		power_ctl_timer.auto_repeat = TRUE;
		//timer_start(&power_ctl_timer);


		//ret = Ql_Timer_Start(power_ctl_timer_id,power_ctl_interval,TRUE);
		if(!timer_start(&power_ctl_timer))
		{
			Ql_Debug_Trace("\r\n<--failed!!  timer_start FAILED-->\r\n");        
		}
		//Ql_Debug_Trace("\r\n<-- stack timer Ql_Timer_Start(ID=%d,Interval=%d,) ret=%d -->\r\n",power_ctl_timer_id,power_ctl_interval,ret);
	}
	else if(sys_info.mode == MODE_POWER_KEY)
	{
		Ql_Debug_Trace("\r\n<-- enter  powerkey_ctl start -->\r\n");
		sys_info.status = SYS_STATE_POWER_OFF;
		powerkey_ctl(FALSE);//power restart
	}

}

bool timer_init(timer_struct_type *p_timer)
{

	s32 ret = 0;

	timer_data_init(p_timer);

	//register  a timer
	ret = Ql_Timer_Register(p_timer->id, p_timer->callback_onTimer, p_timer->param);
	if(ret <0)
	{
		Ql_Debug_Trace("\r\n<--failed!!, Ql_Timer_Register: timer(%d) fail ,ret = %d -->\r\n",p_timer->id,ret);
		return FALSE;
	}
	else
		return TRUE;


}


bool timer_start(timer_struct_type * p_timer)
{
	s32 ret =0;
	ret = Ql_Timer_Start(p_timer->id,p_timer->interval,p_timer->auto_repeat);
	if(ret == 0)
	{
		return TRUE;
	}
	else 
	{
		Ql_Debug_Trace("\r\n<-- timer_start fail  ret:%d-->\r\n",ret);
		return FALSE;
	}
}


// timer callback function
void Timer_handler(u32 timerId, void* param)
{
	unsigned int random_num = 0;
	s32 ret =0;
	//random_num = random();


	bool led_flag = FALSE;

	//Ql_Debug_Trace("\r\n<--random_num:%d-->\r\n",random_num); 
	//Ql_Debug_Trace("\r\n<--random:-->\r\n"); 
	//*((s32*)param) +=1;

	//Ql_Debug_Trace("\r\n<-- Case1: Runtimes:%d NetworkError:%d cpin: -->\r\n",timerId,system_test_runtime_cnt,network_error_cnt);
	//Ql_Debug_Trace("\r\n<--  Runtimes:%d NetworkError:%d cpin: timerId:%d mode:%d-->\r\n",sys_info.system_test_runtime_cnt,sys_info.network_error_cnt,timerId,sys_info.mode);
	//while(1);
#if 0
	if(s_mode == MODE_POWD_PWR_EN)
	{
		Ql_Debug_Trace("\r\n<-- Case1: Runtimes:%d NetworkError:%d cpin: timerId:%d get mode:%d-->\r\n",system_test_runtime_cnt,network_error_cnt,timerId);
	}
	else if(s_mode == MODE_POWER_KEY)
	{
		Ql_Debug_Trace("\r\n<-- Case2: Runtimes:%d NetworkError:%d cpin: timerId:%d get mode:%d -->\r\n",system_test_runtime_cnt,network_error_cnt,timerId);
	}
#endif



	if(power_ctl_timer.id == timerId)
	{
		//Ql_Debug_Trace("<-- Timer_handler, param:%d -->\r\n", *((s32*)param));
		// stack_timer repeat 

		switch(sys_info.status)
		{
#if 0
			case SYS_STATE_POWER_ON:
				{
#if 0
					*((s32*)param) +=1;
					if(*((s32*)param) >= TIMEOUT_COUNT)
					{
						*((s32*)param) = 0;
						Ql_UART_Write(UART_PORT_INTER, "AT\r\n", Ql_strlen(("AT\r\n")));
					}
#endif
					send_ATC( "AT\r\n");//remember to stop
				}
				break;
#endif
			case SYS_STATE_POWER_OFF:
				{

					if(1)
					{
						//*((s32*)param) = 0;
#if 0
						s32 ret;
						ret = Ql_Timer_Stop(power_ctl_timer_id);
						if(ret < 0)
						{
							Ql_Debug_Trace("\r\n<--failed!! stack timer Ql_Timer_Stop ret=%d-->\r\n",ret);           
						}
						Ql_Debug_Trace("\r\n<--stack timer Ql_Timer_Stop(ID=%d,) ret=%d-->\r\n",power_ctl_timer_id,ret); 
#endif
						//ret = Ql_Timer_Stop(power_ctl_timer_id);
						power_ctl_genneral(TRUE);
						sys_info.status = SYS_STATE_POWER_ON;
						Ql_Timer_Stop(power_ctl_timer.id);

						//Ql_Debug_Trace("\r\n<-- POWER ON NOW -->\r\n");
						//Ql_UART_Write(UART_PORT_CONSOLE, "POWER ON\r\n", Ql_strlen("POWER ON\r\n"));

						start_at_sync();

						//power_ctl_timer.interval = 500;
						//power_ctl_timer.auto_repeat = TRUE;
						//timer_start(&power_ctl_timer);

						//Ql_Timer_Start(power_ctl_timer_id,500,TRUE);//REPEAT send AT
						//sys_info.status = SYS_STATE_IDLE;


					}        

				}
				break;
			case SYS_STATE_QPOWD:
				{
					Ql_Debug_Trace("<-- SYS_STATE_QPOWD power off -->\r\n");
					Ql_UART_Write(UART_PORT_CONSOLE, "POWER OFF\r\n", Ql_strlen("POWER OFF\r\n"));
					sys_info.status = SYS_STATE_POWER_OFF;
					power_pwren_VBAT_ctl(FALSE);


					power_ctl_timer.interval = 5000;//after 5000ms power on
					power_ctl_timer.auto_repeat = FALSE;
					timer_start(&power_ctl_timer);

					//Ql_Timer_Start(power_ctl_timer_id,500,FALSE);//500S later power on
#if 0
					//start a timer,repeat=true;
					power_ctl_interval = random();
					ret = Ql_Timer_Start(power_ctl_timer_id,power_ctl_interval,TRUE);
					if(ret < 0)
					{
						Ql_Debug_Trace("\r\n<--failed!! stack timer Ql_Timer_Start ret=%d-->\r\n",ret);        
					}
					Ql_Debug_Trace("\r\n<-- stack timer Ql_Timer_Start(ID=%d,Interval=%d,) ret=%d -->\r\n",power_ctl_timer_id,power_ctl_interval,ret);
#endif					


				}
				break;
			case SYS_STATE_CREG:
				{
					if(flag_cgreg_success == TRUE)
					{
						send_ATC( "AT+CREG?\r\n");
					}
					else
					{
						sys_info.status = SYS_STATE_CGREG;
						send_ATC("AT+CGREG?\r\n");
					}

				}
				break;
			case SYS_STATE_CGREG:
				{
					if(flag_creg_success == TRUE)
					{
						send_ATC( "AT+CGREG?\r\n");
						//sys_info.status = SYS_STATE_CREG;
						//send_ATC( "AT+CREG?\r\n", Ql_strlen(("AT+CREG?\r\n")));

					}
					else
					{
						sys_info.status = SYS_STATE_CREG;
						send_ATC( "AT+CREG?\r\n");
					}
				}
				break;


		}



	}
	//else 
	else if(system_timer.id == timerId)
	{
		//\CA\E4\B3\F6\D0\C5Ϣ\B8\F1ʽ\A3\BAcase name: system runtime , test time ,
		//after POWD seconds cnt:[2s:xx ,3s:,4s:,5s:,6s:], 
		//creg_error_num,cgreg_error_num,system_status,error_type;

		sys_info.system_timer_cnt++;
		sys_info.total_time++;
		Ql_Debug_Trace("\r\n<-- Case:%d[QPOWR=1&VBAT],Count:%d,TotalTime:%ds,OnceTime:%ds,GSMError:%d,GPRSError:%d,CPINError:%d,VBATOFF:[2s:%d,3s:%d,4s:%d,5s:%d,6s:%d,inv:%d],sys_crush_cnt:%d -->\r\n",\
				sys_info.mode+1,\
				sys_info.system_test_runtime_cnt,\
				sys_info.total_time,\
				sys_info.system_timer_cnt,\	
				sys_info.creg_network_error_cnt,\
				sys_info.cgreg_network_error_cnt,\
				sys_info.cpin_error_cnt,\
				sys_info.random_cnt.sec_2,\
				sys_info.random_cnt.sec_3,\
				sys_info.random_cnt.sec_4,\
				sys_info.random_cnt.sec_5,\
				sys_info.random_cnt.sec_6,\
				sys_info.random_cnt.invalid,\
				sys_info.sys_crush_cnt);


		//while(1);
		//Ql_Debug_Trace("\r\n<-- system_timer_cnt:%d -->\r\n",system_timer_cnt); 
	}
	else if(powerkey_signal_timer.id == timerId)
	{
		if(sys_info.mode == MODE_POWER_KEY)
		{
			Ql_Debug_Trace("\r\n<-- enter  Timer_handler timerId:%d pwrkey level:%d-->\r\n",timerId,Ql_GPIO_GetLevel(PINNAME_DCD));


			//Ql_Timer_Stop(powerkey_timer_id);

			if(SYS_STATE_POWER_OFF == sys_info.status)
			{
				gpio_set(PINNAME_DCD,PINLEVEL_HIGH);

				powerkey_signal_timer.interval = 4000;//stay high 4s
				powerkey_signal_timer.auto_repeat = FALSE;
				timer_start(&powerkey_signal_timer);
				//Ql_Timer_Start(powerkey_timer_id,4000,FALSE);//
				sys_info.status = SYS_STATE_POWER_RESTART_SIG;

			}
			else if(SYS_STATE_POWER_RESTART_SIG == sys_info.status )
			{
				sys_info.status = SYS_STATE_POWER_ON;
				//powerkey_ctl(0);// on
				powerkey_ctl(TRUE);
			}
			else if(SYS_STATE_POWER_ON == sys_info.status)
			{
				gpio_set(PINNAME_DCD,PINLEVEL_HIGH);
				start_at_sync();
			}
			else

			{
				//gpio_set(PINNAME_DCD,PINLEVEL_HIGH);
				//start_at_sync();
			}
		}
	}
	else if(throwout_error_timer.id == timerId)
	{
		static unsigned char at_error_cnt = 0;
		//while(1);



		at_error_cnt++;
		if( at_error_cnt >= 3)
		{

			//sys_hangup();
			//Ql_OS_WaitEvent(g_victor_EventGrpId, TSET_EVENT_AT_RESPONSE);
			//while(1);// hangup	
		}
		else
		{
			//throwout_error(sys_info.error_type);

		}
		throwout_error(sys_info.error_type);




	}

}

void Timer_handler_led(u32 timerId, void* param)
{
	static unsigned char led_cnt = 0;
	if(led.timer.id== timerId)
	{
		//Ql_Debug_Trace("<-- GP-Timer_handler, param:%d -->\r\n", *((s32*)param));


		if(led.flag_fast_slow)//fast
		{
			gpio_reverse(PINNAME_NETLIGHT);	
		}
		else
		{
			led_cnt++;
			if(led_cnt == 5)
			{
				led_cnt = 0;
				led_ctl(FALSE);	
			}
			else
			{
				led_ctl(TRUE);
			}
		}


	}

}
void Timer_handler_AT_sync_timerout(u32 timerId, void* param)
{
	//static unsigned char at_sync_cnt = 0;
	Enum_PinLevel slave_status_pin_level = PINLEVEL_HIGH;
	//static u8 sync_failed_timeout_cnt = 0;

	if(at_sync_timer.id== timerId)
	{

		sys_info.at_sync_cnt++;
		Ql_Debug_Trace("<-- Timer_handler_AT_sync_timerout at_sync_cnt:%d -->\r\n", sys_info.at_sync_cnt);

		if(sys_info.at_sync_cnt >= 60 )
		{
			sys_info.sync_failed_timeout_cnt++;
			Ql_Timer_Stop(at_sync_timer.id);// STOP

			sys_info.at_sync_cnt = 0;

			// 1 throwout error at sync error
#ifdef TEST_OBJECT_UC15
			if(sys_info.sync_failed_timeout_cnt > 3)
			{
				sys_info.sync_failed_timeout_cnt = 0;
				start_throwout_error(ERROR_AT_SYNC_NO_RESPONSE);
			}
			else
			{
				sys_info.sys_crush_cnt++;
				pwr_reset_pre();//control VBAT to reset test object
			}
#else
			start_throwout_error(ERROR_AT_SYNC_NO_RESPONSE_TIMEOUT);
#endif			

			// 2 read slave device power status



			if(get_slave_device_status(&slave_status_pin_level))
			{
				Ql_Debug_Trace("<-- get_slave_device_status pin level:%d -->\r\n", slave_status_pin_level);
				if(slave_status_pin_level == PINLEVEL_HIGH)//slave dev is power on 
				{


				}
				else
				{
					start_throwout_error(ERROR_SYSTEM_POWER_ON_FAILED);
				}


			}



		}
		else// send again
		{
			data_frame_init(&uart_inter);//clean
			send_ATC( "AT\r\n");
		}


	}

}


void Timer_handler_ATC_timerout(u32 timerId, void* param)
{


	if(atc_timer.id== timerId)
	{
		//Ql_Debug_Trace("<-- atc_timer, param:%d -->\r\n", *((s32*)param));



		start_throwout_error(ERROR_AT_RETURN);



	}

}

void Timer_handler_powerswitch_timerout(u32 timerId, void* param)
{
	static bool flag_power = TRUE;// POWER ON
	static Enum_PinLevel slave_status_pin_level = PINLEVEL_HIGH;



	if(get_slave_device_status(&slave_status_pin_level))
	{
		if(slave_status_pin_level == PINLEVEL_HIGH)//slave dev is power on 
		{
			Ql_Debug_Trace("<-- slave_status_pin_level == PINLEVEL_HIGH -->\r\n");

		}
		else
		{
			Ql_Debug_Trace("<-- slave_status_pin_level == PINLEVEL_LOW -->\r\n");
		}


	}









	if(power_switch_timer.id== timerId)
	{
		//Ql_Debug_Trace("<-- atc_timer, param:%d -->\r\n", *((s32*)param));

		if(flag_power)//power on
		{
			if(slave_status_pin_level == PINLEVEL_LOW)//slave dev is power off
			{
				Ql_Debug_Trace("<-- slave dev is power OFF ,should be ON -->\r\n");
				//Ql_OS_WaitEvent(g_victor_EventGrpId, TSET_EVENT_AT_RESPONSE);//hang up
			}


			flag_power = FALSE;
			power_pwren_VBAT_ctl(FALSE);	//POWER OFF
			Ql_Debug_Trace("<-- send signal:power OFF,wait 5s -->\r\n");

			power_switch_timer.interval = 5000;//  1s
			power_switch_timer.auto_repeat = FALSE;
			timer_start(&power_switch_timer);

		}
		else
		{
			if(slave_status_pin_level == PINLEVEL_HIGH)//slave dev is power on
			{
				Ql_Debug_Trace("<-- slave dev is power ON ,should be OFF-->\r\n");
				//Ql_OS_WaitEvent(g_victor_EventGrpId, TSET_EVENT_AT_RESPONSE);//hang up
			}
			flag_power = TRUE;
			power_pwren_VBAT_ctl(TRUE);	//POWER ON
			Ql_Debug_Trace("<-- send signal:power ON,wait 20s -->\r\n");

			power_switch_timer.interval = 20000;//  20s
			power_switch_timer.auto_repeat = FALSE;
			timer_start(&power_switch_timer);

		}

	}

}


//\BF\D8\D6\C6\CD\F8\C2\E7\B5ƿ\EC\C9\C1(200ms/200ms) \D5\FD\B3\A3ʱ\C2\FD\C9\C1(1s/200ms)
void led_flicker(bool fast_slow)
{
	s32 ret = 0;
	//led_flag_fast_slow = fast_slow;
	led.flag_fast_slow = fast_slow; 
	led_ctl(TRUE);
	//led_timer_interval = 200;//ms

	Ql_Timer_Stop(led.timer.id);

	led.timer.interval = 200;
	led.timer.auto_repeat = TRUE;
	//ret = Ql_Timer_Start(led_timer_id,led_timer_interval,TRUE);
	if(!timer_start(&(led.timer)))
	{
		Ql_Debug_Trace("\r\n<-- led.timer start fail -->\r\n");
	}       	
}

void uart_init(Enum_SerialPort port)
{
	s32 ret = 0;
	// Register & open UART port
	ret = Ql_UART_Register(port, CallBack_UART_Hdlr, NULL);
	if (ret < QL_RET_OK)
	{
		Ql_Debug_Trace("<-- Fail to register serial port[%d], ret=%d -->\r\n", port, ret);
	}
	ret = Ql_UART_Open(port, 115200, FC_NONE);
	if (ret < QL_RET_OK)
	{
		Ql_Debug_Trace("<-- Fail to open serial port[%d], ret=%d -->\r\n", port, ret);
	}
}
s32 ReadSerialPort(Enum_SerialPort port, /*[out]*/u8* pBuffer, /*[in]*/u32 bufLen)
{
	s32 rdLen = 0;
	s32 rdTotalLen = 0;
	if (NULL == pBuffer || 0 == bufLen)
	{
		return -1;
	}
	Ql_memset(pBuffer, 0x0, bufLen);
	while (1)
	{
		rdLen = Ql_UART_Read(port, pBuffer + rdTotalLen, bufLen - rdTotalLen);
		if (rdLen <= 0)  // All data is read out, or Serial Port Error!
		{
			break;
		}
		rdTotalLen += rdLen;
		// Continue to read...
	}
	if (rdLen < 0) // Serial Port Error!
	{
		Ql_Debug_Trace("<-- Fail to read from port[%d] -->\r\n", port);
		return -99;
	}
	return rdTotalLen;
}

void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara)
{
	s32 totalBytes = 0;
	s32 ret = 0;
	//	static bool s_confirm_flag = FALSE;//

	//Ql_Debug_Trace("\r\n<-- CallBack_UART_Hdlr: port=%d, event=%d, level=%d, p=%x -->\r\n", port, msg, level, customizedPara);
	switch (msg)
	{
		case EVENT_UART_READY_TO_READ:
			{
				//Ql_Sleep(50);//wait for buffer engouh
				//Ql_Debug_Trace("\r\n< EVENT_UART_READY_TO_READ");

				if (UART_PORT_INTER == port)//from main uart send to console
				{
					totalBytes = ReadSerialPort(port, (uart_inter.rx_buf)+uart_inter.in_cnt, sizeof((uart_inter.rx_buf))-uart_inter.in_cnt);

					if (totalBytes <= 0)
					{
						Ql_Debug_Trace("<-- No data in UART buffer! -->\r\n");
						return;
					}
					else
					{
						uart_inter.in_cnt += totalBytes;
						//Ql_UART_Write(UART_PORT_CONSOLE, (uart_inter.rx_buf), sys_info.totalBytes_all);	
						//Ql_Debug_Trace("<-- From Slave UART part: -->\r\n %s\r\n",(uart_inter.rx_buf));
						if((Ql_strstr((uart_inter.rx_buf), "\r\n") != NULL)  )
						{
							//sys_info.totalBytes_all-=2;


						}	
						//use "OK" or "ERRROR" as the end mark of a whole uart data frame
						if((Ql_strstr((uart_inter.rx_buf), "OK") != NULL)  || (Ql_strstr((uart_inter.rx_buf), "ERROR") != NULL))
						{

							Ql_UART_Write(UART_PORT_CONSOLE, (uart_inter.rx_buf), uart_inter.in_cnt);	
							Ql_Debug_Trace("<-- From Slave UART: -->\r\n %s\r\n",(uart_inter.rx_buf));

							uart_inter.in_cnt=0;//

						}							
						else
						{
							Ql_Debug_Trace("<-- From Slave UART: (uart_inter.rx_buf) totalBytes_all %d-->\r\n",uart_inter.in_cnt);
							return;

						}


					}

					//Ql_UART_Write(UART_PORT_CONSOLE, (uart_inter.rx_buf), totalBytes);	
					//Ql_Debug_Trace("<-- From Slave UART: -->\r\n %s\r\n",(uart_inter.rx_buf));


#if 1
					switch (sys_info.status)
					{
						case SYS_STATE_IPR_SET:
							{
								static u8 s_tem_ok = 0; 

								if(Ql_strstr((uart_inter.rx_buf), "OK") != NULL)
								{
#ifdef TEST_OBJECT_UC15
									sys_info.status = SYS_STATE_ATW;
									send_ATC( "AT&W\r\n");
#else
									s_tem_ok += check_ok_num((uart_inter.rx_buf));
									//s_tem_ok++;
#ifdef TEST_OBJECT_GW

									if(s_tem_ok == 6)
#else
									if(s_tem_ok == 7)
#endif
									{
										sys_info.status = SYS_STATE_ATW;
										send_ATC( "AT&W\r\n");
									}
#endif
								}
								else if((Ql_strstr((uart_inter.rx_buf), "SM not ready") != NULL) )
								{
									sys_info.cpin_error_cnt++;
									start_throwout_error(ERROR_CPIN);


								}

							}
							break;
						case SYS_STATE_ATW:
							if( (Ql_strstr((uart_inter.rx_buf), "OK") != NULL) ||(Ql_strstr((uart_inter.rx_buf), "ERROR") != NULL))
							{
								sys_info.status = SYS_STATE_ATE1V1;
								send_ATC( "ATE1V1\r\n");

							}

#if 0
							{
								Ql_Debug_Trace("\r\n<-- POWER OFF NOW -->\r\n");
								sys_info.status = SYS_STATE_POWER_OFF;
								power_ctl_genneral(FALSE);


								//start a timer,repeat=true;
								//power_ctl_interval = random();


								power_ctl_timer.interval = 1000;
								power_ctl_timer.auto_repeat = TRUE;
								timer_start(&power_ctl_timer);

								//ret = Ql_Timer_Start(power_ctl_timer_id,1000,TRUE);
								//if(ret < 0)
								{
									//Ql_Debug_Trace("\r\n<--failed!! stack timer Ql_Timer_Start ret=%d-->\r\n",ret);        
								}
								//Ql_Debug_Trace("\r\n<--stack timer Ql_Timer_Start(ID=%d,Interval=%d,) ret=%d-->\r\n",power_ctl_timer_id,power_ctl_interval,ret);



							}
#endif
							break;
						case SYS_STATE_IDLE:
#if 0
							if((Ql_strstr((uart_inter.rx_buf), "RDY") != NULL) ||(Ql_strstr((uart_inter.rx_buf), "rdy") != NULL))
							{

							}
							else if((Ql_strstr((uart_inter.rx_buf), "CFUN") != NULL) )
							{


							}
							else if((Ql_strstr((uart_inter.rx_buf), "+CPIN") != NULL) )
							{
								sys_info.status = SYS_STATE_IPR_READ;
								send_ATC( "AT+IPR?\r\n", Ql_strlen(("AT+IPR?\r\n")));

							}
#endif

							break;
						case SYS_STATE_IPR_READ:
							//if((Ql_strstr((uart_inter.rx_buf), "115200") != NULL) )
							{


							}
							if((Ql_strstr((uart_inter.rx_buf), "OK") != NULL) )
							{
								//sys_info.system_test_runtime_cnt++;//
								sys_info.status = SYS_STATE_ATE1V1;
								send_ATC( "ATE1V1\r\n");
							}

							break;
						case SYS_STATE_ATE1V1:

							if((Ql_strstr((uart_inter.rx_buf), "OK") != NULL) )
							{

								sys_info.status = SYS_STATE_ATI;
								send_ATC( "ATI\r\n");



							}
							else
							{
								//start_throwout_error(ERROR_AT_RETURN);
							}

							break;
						case SYS_STATE_ATI:




							{
								if((Ql_strstr((uart_inter.rx_buf), "OK") != NULL)&& (Ql_strstr((uart_inter.rx_buf), "ATI") != NULL) )
								{
									//sys_info.status = SYS_STATE_ATI;
									//s_confirm_flag = FALSE;
									//send_ATC( "AT+GSN\r\n");
									//sys_info.status = SYS_STATE_IMEI;

									//Ql_Debug_Trace("\r\n power off");
									//power_ctl(FALSE);


									Ql_Timer_Stop(power_ctl_timer.id);//!!

									send_ATC( "AT+CREG?\r\n");
									sys_info.status = SYS_STATE_CREG;
									flag_cgreg_success = FALSE;
									flag_creg_success = FALSE;
									//while(1);

								}
								else
								{
									//start_throwout_error(ERROR_NV_ATI);
								}

							}



						case SYS_STATE_CREG://+CREG: 0,2
							{
								unsigned char gsm_status = check_gsm_network_status((uart_inter.rx_buf));
								//Ql_Debug_Trace("\r\n<-- check_gsm_network_status:%d creg_network_error_cnt:%d-->\r\n",gsm_status,sys_info.creg_network_error_cnt);

								Ql_Timer_Stop(power_ctl_timer.id);//!!


								if((Ql_strstr((uart_inter.rx_buf), "OK") != NULL)\
										&& ((gsm_status == 1)|| (gsm_status == 5)) )
								{
									sys_info.creg_network_error_cnt = 0;
									flag_creg_success = TRUE;

									if(TRUE == flag_cgreg_success)//gsm gprs all success
									{
										sys_info.status = SYS_STATE_EGMR07;
										send_ATC( "AT+EGMR=0,7\r\n");

									}
									else
									{
										sys_info.status = SYS_STATE_CGREG;
										send_ATC( "AT+CGREG?\r\n");	
									}





								}
								else
								{
									flag_creg_success = FALSE;

									if(sys_info.system_timer_cnt >= NETWORK_ERROR_MAX_TIME)//6min
									{
										sys_info.creg_network_error_cnt++;
										sys_info.network_error_cnt++;
										if(sys_info.creg_network_error_cnt >= NETWORK_ERROR_MAX_CNT)//hangup
										{
											//network_error_cnt = 0;
											start_throwout_error(ERROR_GSM);



												//sys_hangup();
										}
										else//restart
										{
											//led_flicker(FALSE);
											sys_info.status = SYS_STATE_POWER_OFF;
											power_ctl_genneral(FALSE);
											Ql_UART_Write(UART_PORT_CONSOLE, "power off\r\n", Ql_strlen("power off\r\n"));
											Ql_Timer_Stop(system_timer.id);

											power_ctl_timer.interval = 2000;
											power_ctl_timer.auto_repeat = TRUE;
											timer_start(&power_ctl_timer);
											//Ql_Timer_Start(power_ctl_timer_id,2000,TRUE);
										}

										break;
									}
									else
									{


									}


									if(TRUE == flag_cgreg_success)// gprs only success
									{
										power_ctl_timer.interval = 1000;
										power_ctl_timer.auto_repeat = FALSE;
										timer_start(&power_ctl_timer);
// 1S LATER SEND AT "creg"
											//Ql_Timer_Start(power_ctl_timer_id,1000,FALSE);
// 1S LATER SEND AT

											//sys_info.status = SYS_STATE_CGREG;
											//send_ATC( "AT+CGREG?\r\n", Ql_strlen(("AT+CGREG?\r\n")));
									}
									else
									{
										sys_info.status = SYS_STATE_CGREG;
										send_ATC( "AT+CGREG?\r\n");
									}




								}



							}
							break;
						case SYS_STATE_CGREG:
							{
								unsigned char gprs_status = check_gprs_network_status((uart_inter.rx_buf));
								//Ql_Debug_Trace("\r\n<-- check_gprs_network_status:%d cgreg_network_error_cnt:%d -->\r\n",gprs_status,sys_info.cgreg_network_error_cnt);
								if((Ql_strstr((uart_inter.rx_buf), "OK") != NULL)\
										&& ((gprs_status == 1)|| (gprs_status == 5)) )
								{
									sys_info.cgreg_network_error_cnt = 0;
									flag_cgreg_success = TRUE;
									//led_flicker(FALSE);
									Ql_Timer_Stop(power_ctl_timer.id);

									if(TRUE == flag_creg_success)//GSM
									{
										sys_info.status = SYS_STATE_EGMR07;
										send_ATC( "AT+EGMR=0,7\r\n");
									}
									else
									{
										sys_info.status = SYS_STATE_CREG;
										send_ATC( "AT+CREG?\r\n");
									}

								}
								else
								{

									flag_cgreg_success = FALSE;
									if(sys_info.system_timer_cnt >= NETWORK_ERROR_MAX_TIME)//180s
									{
										sys_info.cgreg_network_error_cnt++;//failed
										sys_info.network_error_cnt++;
										if(sys_info.cgreg_network_error_cnt >= NETWORK_ERROR_MAX_CNT)//hangup
										{
											//network_error_cnt = 0;
											start_throwout_error(ERROR_GPRS);


										}
										else//restart
										{
											//led_flicker(FALSE);
											sys_info.status = SYS_STATE_POWER_OFF;
											power_ctl_genneral(FALSE);
											Ql_UART_Write(UART_PORT_CONSOLE, "power off\r\n", Ql_strlen("power off\r\n"));
											Ql_Timer_Stop(system_timer.id);

											power_ctl_timer.interval = 2000;
											power_ctl_timer.auto_repeat = TRUE;
											timer_start(&power_ctl_timer);
											//Ql_Timer_Start(power_ctl_timer_id,2000,TRUE);
										}


									}
									else
									{
										power_ctl_timer.interval = 1000;
										power_ctl_timer.auto_repeat = FALSE;
										timer_start(&power_ctl_timer);
										//Ql_Timer_Start(power_ctl_timer_id,1000,FALSE);// 1S LATER SEND AT
									}




								}
							}

							break;
						case SYS_STATE_EGMR07:
							if(Ql_strstr((uart_inter.rx_buf), "OK") != NULL)
							{
#ifdef TEST_OBJECT_UC15
								send_ATC( "AT+QICSGP=1\r\n");
								sys_info.status = SYS_STATE_APN;
#else
#ifdef TEST_OBJECT_GW
								//send_ATC( "AT$MYNETCON?\r\n"); 
								send_ATC( "AT\r\n"); //skip the APN read & write process 
								sys_info.status = SYS_STATE_APN_SET;
#else
								send_ATC( "AT+QIREGAPP?\r\n");
								sys_info.status = SYS_STATE_APN;
#endif

#endif
								//AT+QICSGP=1,1,"abc","",""//uc15 mode

								//send_ATC( "AT+EGMR=0,5\r\n");
							}
							break;

							/*
							   at+qiregapp?

							   +QIREGAPP: "CMNET","",""

							   OK

							   at+qicsgp=1,"abc"

							   OK
							   */
							/*

							   AT+QICSGP=1

							   +QICSGP: 1,"UNINET","","",1

							   OK
							   AT+QICSGP=1,1,"abc","",""

							   OK
							   AT+QICSGP=1

							   +QICSGP: 1,"abc","","",1

							   OK

*/
						case SYS_STATE_APN:
							{


								{
#ifdef TEST_OBJECT_UC15
									if((Ql_strstr((uart_inter.rx_buf), "OK") != NULL)&& (Ql_strstr((uart_inter.rx_buf), "+QICSGP:") != NULL) )//out once
#else

#ifdef TEST_OBJECT_GW
										if((Ql_strstr((uart_inter.rx_buf), "OK") != NULL)&& (Ql_strstr((uart_inter.rx_buf), "MYNETCON") != NULL) )//out once
#else
											if((Ql_strstr((uart_inter.rx_buf), "OK") != NULL)&& (Ql_strstr((uart_inter.rx_buf), "+QIREGAPP:") != NULL) )//out once
#endif										
#endif
											{
												//check apn
												//if((((u8)(sys_info.system_test_runtime_cnt&0x00ff))&0x01 ) == 1)
												if(FALSE == sys_info.flag_apn_check)
												{
													if(check_nv_apn((uart_inter.rx_buf),"abc"))
													{
														sys_info.status = SYS_STATE_APN_SET;
#ifdef TEST_OBJECT_UC15
														send_ATC( "AT+QICSGP=1,1,\"xyz\",\"\",\"\"\r\n");

#else

#ifdef TEST_OBJECT_GW
														send_ATC( "AT$MYNETCON=0,\"APN\",\"xyz\"\r\n");
#else


														send_ATC( "AT+QICSGP=1,\"xyz\"\r\n");
#endif
#endif
														//AT+QICSGP=1,1,"abc","",""
														sys_info.flag_apn_check = TRUE;

													}
													else
													{
														start_throwout_error(ERROR_APN);
													}


												}
												else
												{
													if(check_nv_apn((uart_inter.rx_buf),"xyz"))
													{
														sys_info.status = SYS_STATE_APN_SET;
#ifdef TEST_OBJECT_UC15
														send_ATC( "AT+QICSGP=1,1,\"abc\",\"\",\"\"\r\n");
#else

#ifdef TEST_OBJECT_GW
														send_ATC( "AT$MYNETCON=0,\"APN\",\"abc\"\r\n");
#else


														send_ATC( "AT+QICSGP=1,\"abc\"\r\n");
#endif

#endif
														//AT+QICSGP=1,1,"abc","",""
														sys_info.flag_apn_check = TRUE;
														sys_info.flag_apn_check = FALSE;

													}
													else
													{
														start_throwout_error(ERROR_APN);
													}

												}
											}

											else
											{
												//start_throwout_error(ERROR_APN);
											}

								}


							}
							break;
						case SYS_STATE_APN_SET:
							{


								{
									if((Ql_strstr((uart_inter.rx_buf), "OK") != NULL) )//out once
									{

										sys_info.status = SYS_STATE_IMEI;
										send_ATC( "AT+GSN\r\n");



									}

									else
									{
										//start_throwout_error(ERROR_APN);
									}

								}


							}
							break;
						case SYS_STATE_IMEI:
							{
								//static bool flag = FALSE;

								{
									if((Ql_strstr((uart_inter.rx_buf), "OK") != NULL)&& (Ql_strstr((uart_inter.rx_buf), "GSN") != NULL) )//out once
									{
										sys_info.status = SYS_STATE_QMIC;
										send_ATC( "AT+QMIC?\r\n");
										//send_ATC( "AT+QIREGAPP?\r\n");
									}

									else
									{
										//start_throwout_error(ERROR_NV_IMEI);
									}

								}


							}
							break;


						case SYS_STATE_QMIC:


							{
								if((Ql_strstr((uart_inter.rx_buf), "OK") != NULL)&& (Ql_strstr((uart_inter.rx_buf), "QMIC?") != NULL) )//out once
								{
									if(check_qmic((uart_inter.rx_buf), &(g_nv_value.qmic)))
									{
										sys_info.status = SYS_STATE_CLVL;
										send_ATC( "AT+CLVL?\r\n");

									}
									else
									{
										start_throwout_error(ERROR_NV);
									}
								}
								else
								{
									//start_throwout_error(ERROR_NV);
								}

							}
							break;

						case SYS_STATE_CLVL:

							{
								if((Ql_strstr((uart_inter.rx_buf), "OK") != NULL)&& (Ql_strstr((uart_inter.rx_buf), "CLVL?") != NULL) )//out once
								{
									if(check_clvl((uart_inter.rx_buf)))
									{
										sys_info.status = SYS_STATE_CMEE;
										send_ATC( "AT+CMEE?\r\n");
									}
									else
									{
										start_throwout_error(ERROR_NV);

									}
								}
								else
								{
									//start_throwout_error(ERROR_NV);
								}

							}
							break;
						case SYS_STATE_CMEE:


							{
								if((Ql_strstr((uart_inter.rx_buf), "OK") != NULL)&& (Ql_strstr((uart_inter.rx_buf), "CMEE?") != NULL) )//out once
								{
#ifdef TEST_OBJECT_UC15
									if(1)//uc15 do not save cmee to nvram
#else

										if(check_nv_unit((uart_inter.rx_buf),"+CMEE: ",&(g_nv_value.cmee)))
#endif
										{
											sys_info.status = SYS_STATE_CCWA;
											send_ATC( "AT+CCWA?\r\n");
										}
										else
										{
											start_throwout_error(ERROR_NV);
										}
								}

								else
								{
									//start_throwout_error(ERROR_NV);
								}

							}

							break;
						case SYS_STATE_CCWA:


							{
								if((Ql_strstr((uart_inter.rx_buf), "OK") != NULL)&& (Ql_strstr((uart_inter.rx_buf), "CCWA?") != NULL) )
								{
#ifdef TEST_OBJECT_UC15
									if(1)//uc15 do not save cmee to nvram
#else
										if(check_nv_unit((uart_inter.rx_buf),"+CCWA: ",&(g_nv_value.ccwa)))
#endif										
										{

											sys_info.status = SYS_STATE_CMGF;
											send_ATC( "AT+CMGF?\r\n");
										}
										else
										{
											start_throwout_error(ERROR_NV);
										}

								}
								else
								{
									//start_throwout_error(ERROR_NV);

								}

							}


							break;
						case SYS_STATE_CMGF:

							{
								if(((Ql_strstr((uart_inter.rx_buf), "OK") != NULL) || (Ql_strstr((uart_inter.rx_buf), "ERROR") != NULL))&& (Ql_strstr((uart_inter.rx_buf), "CMGF?") != NULL) )
								{
									sys_info.status = SYS_STATE_CSQ;
									send_ATC( "AT+CSQ\r\n");
								}
								else
								{
									//start_throwout_error(ERROR_NV);
								}

							}




							break;
						case SYS_STATE_CSQ:
							if(Ql_strstr((uart_inter.rx_buf), "OK") != NULL)
							{




								sys_info.status = SYS_STATE_EGMR05;

								send_ATC( "AT+EGMR=0,5\r\n");

							}

							break;

						case SYS_STATE_EGMR05:
							if((Ql_strstr((uart_inter.rx_buf), "OK") != NULL) ||(Ql_strstr((uart_inter.rx_buf), "ERROR") != NULL))
							{
								sys_info.status = SYS_STATE_CPIN;

								send_ATC( "AT+CPIN?\r\n");
							}
							break;
							/*
							   [2014-12-02_11:02:27]  at+cpin?

							   [2014-12-02_11:02:31]+CME ERROR: SIM failure
							   [2014-12-02_11:02:31]
							   [2014-12-02_11:02:31]RDY
							   [2014-12-02_11:02:31]at+cpin?

							   [2014-12-02_11:02:31]+CME ERROR: SIM busy
							   [2014-12-02_11:02:32]at+cpin?

							   [2014-12-02_11:02:32]+CME ERROR: SIM busy
							   [2014-12-02_11:02:32]
							   [2014-12-02_11:02:32]+CFUN: 1
							   [2014-12-02_11:02:32]at+cpin?

							   [2014-12-02_11:02:32]+CME ERROR: SIM busy
							   [2014-12-02_11:02:32]at+cpin?

							   [2014-12-02_11:02:32]+CME ERROR: SIM busy
							   [2014-12-02_11:02:32]
							   [2014-12-02_11:02:32]+CPIN: READY
							   [2014-12-02_11:02:33]at+cpin?

							   [2014-12-02_11:02:33]+CPIN: READY

							   [2014-12-02_11:02:33]OK
							   [2014-12-02_11:02:33]at+cpin?

							   [2014-12-02_11:02:33]+CPIN: READY

							   [2014-12-02_11:02:33]OK
							   [2014-12-02_11:02:33]at+cpin?

							   [2014-12-02_11:02:33]+CPIN: READY

							   [2014-12-02_11:02:33]OK
							   [2014-12-02_11:02:36]
							   [2014-12-02_11:02:36]Call Ready
							   [2014-12-02_11:02:41]at+cpin?

							   [2014-12-02_11:02:41]+CPIN: READY

							   [2014-12-02_11:02:41]OK


							   [2014-12-02_11:03:12]at+cpin?

							   [2014-12-02_11:03:12]+CPIN: READY

							   [2014-12-02_11:03:12]OK
							   [2014-12-02_11:03:13]at+cpin?

							   [2014-12-02_11:03:13]+CPIN: READY

							   [2014-12-02_11:03:13]OK
							   [2014-12-02_11:03:14]
							   [2014-12-02_11:03:14]+CPIN: NOT READY
							   [2014-12-02_11:03:14]  at+cpin?

							   [2014-12-02_11:03:18]+CME ERROR: SIM failure
							   [2014-12-02_11:03:19]
							   [2014-12-02_11:03:19]RDY
							   [2014-12-02_11:03:19]at+cpin?

							   [2014-12-02_11:03:19]+CME ERROR: SIM busy
							   [2014-12-02_11:03:19]at+cpin?

							   [2014-12-02_11:03:19]+CME ERROR: SIM busy
							   [2014-12-02_11:03:19]at+cpin?

							[2014-12-02_11:03:19]+CME ERROR: SIM busy
								[2014-12-02_11:03:19]at+cpin?

								[2014-12-02_11:03:19]+CME ERROR: SIM busy
								[2014-12-02_11:03:19]
								[2014-12-02_11:03:19]+CFUN: 1

								[2014-12-02_11:03:19]+CPIN: NOT INSERTED
								[2014-12-02_11:03:19]at+cpin?

								[2014-12-02_11:03:19]+CME ERROR: SIM not inserted
								[2014-12-02_11:03:20]at+cpin?

								[2014-12-02_11:03:20]+CME ERROR: SIM not inserted
								[2014-12-02_11:03:20]at+cpin?


								*/
						case SYS_STATE_CPIN:
								if( (Ql_strstr((uart_inter.rx_buf), "OK") != NULL) && (Ql_strstr((uart_inter.rx_buf), "+CPIN: READY") != NULL))
								{
									pwr_reset_pre();


								}
								else if((Ql_strstr((uart_inter.rx_buf), "ERROR") != NULL))
								{
									if((Ql_strstr((uart_inter.rx_buf), "SIM failure") != NULL) \
											|| (Ql_strstr((uart_inter.rx_buf), "SIM busy") != NULL)\
											|| (Ql_strstr((uart_inter.rx_buf), "SIM not inserted") != NULL))
									{
										sys_info.cpin_error_cnt++;
										pwr_reset_pre();

									}

								}
								else
								{

								}
								break;
						case SYS_STATE_QPOWD:
								break;
						case SYS_STATE_POWER_OFF:

								break;
						case SYS_STATE_POWER_ON:
								if(Ql_strstr((uart_inter.rx_buf), "OK") != NULL)
								{
									sys_info.system_test_runtime_cnt++;//

									Ql_Debug_Trace("\r\n<-- AT sync success at_sync_cnt:%d-->\r\n",sys_info.at_sync_cnt);

									sys_info.at_sync_cnt = 0;//clean
									flag_cgreg_success = FALSE;
									flag_creg_success = FALSE;
									Ql_Timer_Stop(power_ctl_timer.id);
									Ql_Timer_Stop(atc_timer.id);

									Ql_Timer_Stop(at_sync_timer.id);
									sys_info.sync_failed_timeout_cnt = 0;

									system_timer.interval = 1000;// 1s
									system_timer.auto_repeat = TRUE;
									//ret = Ql_Timer_Start(system_timer_id,1000,TRUE);// 1s
									if(! timer_start(&system_timer))
									{
										//Ql_Debug_Trace("\r\n<--failed!! system_timer_id Ql_Timer_Start ret=%d-->\r\n",ret);        
									}

									//start:
#if 0								
									if(sys_info.mode == MODE_POWD_PWR_EN)
									{
										Ql_Debug_Trace("\r\n<-- Case1 start : Runtimes:%d NetworkError:%d cpin: s_mode:%d -->\r\n",sys_info.system_test_runtime_cnt,sys_info.network_error_cnt,sys_info.mode);
									}
									else
									{
										Ql_Debug_Trace("\r\n<-- Case2 start : Runtimes:%d NetworkError:%d cpin: s_mode:%d -->\r\n",sys_info.system_test_runtime_cnt,sys_info.network_error_cnt,sys_info.mode);
									}
#endif

									sys_info.system_timer_cnt =0;//cnt total time elapse in one test

									if(sys_info.system_test_runtime_cnt == 1)
									{
										Ql_Sleep(10000);
										set_ipr();
										sys_info.flag_apn_check = FALSE;
										sys_info.status = SYS_STATE_IPR_SET;

									}
									else
									{
										sys_info.status = SYS_STATE_IPR_READ;

										send_ATC( "AT+IPR?\r\n");
									}

#if 0
									//Ql_Debug_Trace("\r\n power off");
									sys_info.status = SYS_STATE_IPR_SET;
									Ql_Timer_Stop(power_ctl_timer_id);

									set_ipr();

									//app_start_AT_check();
									//send_ATC( "AT+CSQ\r\n", Ql_strlen(("AT+CSQ\r\n")));
									//sys_info.status = SYS_STATE_CSQ;
#endif
								}
								break;

						default:
								break;

					}
#endif

					data_frame_init(&uart_inter);//clean

				}
				else if (UART_PORT_CONSOLE == port)//from console send to main uart
				{
					totalBytes = ReadSerialPort(port, RxBuf_Uart_Console, sizeof(RxBuf_Uart_Console));
					if (totalBytes <= 0)
					{
						Ql_Debug_Trace("<-- No data in UART buffer! -->\r\n");
						return;
					}
					if(Ql_strstr(RxBuf_Uart_Console, "start pwren") != NULL)
					{
						sys_info.mode = MODE_POWD_PWR_EN;
						if(sys_info.status == SYS_STATE_IDLE)
						{
							start_power_test();
						}
					}
					else if(Ql_strstr(RxBuf_Uart_Console, "set range:") != NULL)
					{
						if (TRUE == check_power_off_wait_time(RxBuf_Uart_Console,&RANGE_MIN,&RANGE_MAX))
						{
							Ql_memset(RxBuf_Uart_Console, 0x0, SERIAL_RX_BUFFER_LEN);
							//sprintf(RxBuf_Uart_Console,"set range success:%ds,%ds",RANGE_MIN,RANGE_MAX);
							//Ql_UART_Write(UART_PORT_CONSOLE, RxBuf_Uart_Console, Ql_strlen(RxBuf_Uart_Console));
							Ql_UART_Write(UART_PORT_CONSOLE, "set range success\r\n", Ql_strlen("set range success\r\n"));
						}
					}
					else if(Ql_strstr(RxBuf_Uart_Console, "set mode:") != NULL)
					{
						u8 mode_temp = 0;
						if(check_mode(RxBuf_Uart_Console,&mode_temp))
						{
							if (set_mode(mode_temp))
							{

							}

						}




					}
					else if(Ql_strstr(RxBuf_Uart_Console, "start powerkey") != NULL)
					{
						sys_info.mode = MODE_POWER_KEY;
						sys_info.status = SYS_STATE_POWER_ON;
						start_power_test();

						//powerkey_ctl(0);//on
						//power_ctl(TRUE);
					}
					else if(Ql_strstr(RxBuf_Uart_Console, "powerkey:0") != NULL)
					{
						//power_ctl(TRUE);//power on ,in power off state
					}
					else
					{// Read data from UART
						s32 ret;
						char* pCh = NULL;

						// Echo
						Ql_UART_Write(UART_PORT_INTER, RxBuf_Uart_Console, totalBytes);
						Ql_Debug_Trace("<-- From Control UART: -->\r\n%s\r\n",RxBuf_Uart_Console);
						pCh = Ql_strstr((char*)(uart_inter.rx_buf), "\r\n");
						if (pCh)
						{
							*(pCh + 0) = '\0';
							*(pCh + 1) = '\0';
						}

						// No permission for single <cr><lf>
						if (Ql_strlen((char*)(uart_inter.rx_buf)) == 0)
						{
							return;
						}
						//ret = Ql_RIL_SendATCmd((char*)(uart_inter.rx_buf), totalBytes, ATResponse_Handler, NULL, 0);
					}
				}
#if 0
				else if (UART_PORT_INTER == port)//from main uart send to console
				{
					totalBytes = ReadSerialPort(port, (uart_inter.rx_buf), sizeof((uart_inter.rx_buf)));
					if (totalBytes <= 0)
					{
						Ql_Debug_Trace("<-- No data in UART buffer! -->\r\n");
						return;
					}
					Ql_UART_Write(UART_PORT_CONSOLE, (uart_inter.rx_buf), totalBytes);	
					Ql_Debug_Trace("\r\n< to UART_PORT_CONSOLE (uart_inter.rx_buf): %s",(uart_inter.rx_buf));
				}
#endif
				break;
			}
		case EVENT_UART_READY_TO_WRITE:
			break;
		default:
			break;
	}
}

bool check_power_off_wait_time(char * str,char * p_min,char *p_max)//set range:2,5
{
	char *pchar = NULL;
	unsigned char ret = 255;
	pchar = Ql_strstr(str, "set range:");
	if( pchar != NULL)
	{
		*p_min = *(pchar+10) - '0';	
		*p_max = *(pchar+12) - '0';	
	}
	else
		return FALSE;
	if((*p_min)>(*p_max))
	{
		return FALSE;	
	}
	else
		return TRUE;

}

bool check_mode(char * str,u8 * p_mode_out)//set mode:0
{
	char *pchar = NULL;
	//u8 ret = 0;
	pchar = Ql_strstr(str, "set mode:");

	if( pchar != NULL)
	{
		pchar = Ql_strstr(str, "\r\n");
		if(pchar != NULL)
		{
			*p_mode_out = *(pchar-1) - '0';
			Ql_Debug_Trace("<-- check_mode   mode:%d -->\r\n",(*p_mode_out));

			if((*p_mode_out) >=2)//only support mode 0 1
			{
				return FALSE;
			}
			else 
			{
				return TRUE;
			}
		}
		else
		{
			return FALSE;
		}


	}
	else
	{
		return FALSE;
	}


}

u8 check_ok_num(char * str)
{
	char *pchar = NULL;
	u8 cnt = 0;
	if(NULL == str)
		return 0;
	//u8 ret = 0;
	pchar = str;


	do
	{
		pchar = Ql_strstr(pchar, "OK\r\n");
		if(pchar != NULL)
		{
			cnt++;
			pchar+=4;
		}



	}while( pchar != NULL);

	return cnt;



}


bool set_mode(u8 mode)
{
	s32 ret = -1;

	ret = Ql_SecureData_Store(1, (u8*)&mode,1);
	if(0 == ret)//success
	{
		Ql_memset(RxBuf_Uart_Console, 0x0, SERIAL_RX_BUFFER_LEN);
		Ql_UART_Write(UART_PORT_CONSOLE, "set mode success\r\n", Ql_strlen("set mode success\r\n"));
		Ql_Debug_Trace("<-- set mode success  mode:%d -->\r\n",mode);
		return TRUE;

	}
	else
	{
		Ql_Debug_Trace("<-- set mode failed mode:%d -->\r\n",mode);
		return FALSE;
	}



}

bool get_mode(u8 *p_mode_out)
{
	s32 ret = -1;
	//u8 mode = 255;
	u8 lenth = 1;
	ret = Ql_SecureData_Read(1,p_mode_out, lenth);//attention return value

	if(lenth == ret)//success
	{

		if(*p_mode_out >=2)
		{
			goto label_error;
			//return FALSE;//error
		}
		else
			goto label_success;
		//return TRUE;
	}
	else
	{
		goto label_error;
	}

label_error:
	Ql_Debug_Trace("<-- get mode failed ret:%d mode:%d-->\r\n",ret,*p_mode_out);
	return FALSE;//error

label_success:
	Ql_memset(RxBuf_Uart_Console, 0x0, SERIAL_RX_BUFFER_LEN);
	Ql_UART_Write(UART_PORT_CONSOLE, "get mode success\r\n", Ql_strlen("get mode success\r\n"));
	Ql_Debug_Trace("<-- get mode success  mode :%d -->\r\n",*p_mode_out);
	return TRUE;


}

bool init_gpio_get_status(void)
{
	s32 ret = QL_RET_OK;
	ret = Ql_GPIO_Init(PINNAME_DTR, PINDIRECTION_IN, PINLEVEL_LOW, PINPULLSEL_PULLDOWN);//it will be high when no signal

	if (QL_RET_OK == ret)
		return TRUE;
	else
		return FALSE;

	//return ret;
}
bool get_slave_device_status(Enum_PinLevel * p_level_out)
{
	s32 ret = QL_RET_OK;
	ret = Ql_GPIO_GetLevel(PINNAME_DTR);
	if(ret <0) //error
	{
		Ql_Debug_Trace("<-- get_slave_device_status failed ret:%d -->\r\n" ,ret);
		return FALSE;
	}
	else
	{
		* p_level_out = (Enum_PinLevel)ret;
		return TRUE;
	}
}

/*
   at+gsn

   359231033484583

   OK

*/
bool check_IMEI(char * str)
{
	char * pchar = NULL;
	//unsigned char ret = 255;
	unsigned char imei_str_temp[IMEI_LENGTH+1] = {0};
	pchar = Ql_strstr(str, "GSN\r\n");
	if( pchar == NULL)
	{
		pchar = Ql_strstr(str, "gsn\r\n");

	}

	//if( pchar != NULL)
	if(1)
	{

		if(sys_info.system_test_runtime_cnt == 1)
		{
			Ql_memset(g_nv_value.imei_str, 0x0, IMEI_LENGTH+1);
			Ql_memcpy(g_nv_value.imei_str, str, IMEI_LENGTH);//fisrt get 
			Ql_Debug_Trace("<-- IMEI first get:%s -->\r\n",g_nv_value.imei_str);
			return TRUE;
		}
		else
		{
			Ql_memcpy(imei_str_temp, str, IMEI_LENGTH);
			if(Ql_strcmp(imei_str_temp, g_nv_value.imei_str) == 0 )
			{
				Ql_Debug_Trace("<-- IMEI same from:%s to:%s -->\r\n",g_nv_value.imei_str,imei_str_temp);
				return TRUE;
			}
			else
			{
				Ql_Debug_Trace("<-- IMEI changed from:%s to:%s -->\r\n",g_nv_value.imei_str,imei_str_temp);
				return FALSE;
			}
		}
	}
	else
	{

	}
	return FALSE;

}


bool get_clvl(char * str,u8 * p_out)//+CLVL: 60
{
	char * pchar = NULL;
	//unsigned char ret = 255;
	u8 unit = 0;
	u8 ten =0;
	u8 hundred = 0;
	pchar = Ql_strstr(str, "+CLVL:");
	if( pchar != NULL)
	{
#if 1
		unit = *(pchar+8) -'0';
		ten  = *(pchar+7) -'0';

		unit = ((unit<'0') || (unit>'9'))?'0':unit;
		ten = ((ten<'0') || (ten>'9'))?'0':ten;

		*p_out = ten*10+unit;
		Ql_Debug_Trace("<-- get_clvl value:%d -->\r\n",*p_out);
		return TRUE;
#endif

#if 0
		pchar = Ql_strstr(str, "\r\n");
		if( pchar != NULL)
		{
			unit = *(pchar-1) -'0';
			ten  = *(pchar-2) -'0';
			*p_out = ten*10+unit;
			Ql_Debug_Trace("<-- get_clvl value:%d -0:%d,-1:%d,-2:%d,-3:%d-->\r\n",*p_out,*(pchar),*(pchar-1),*(pchar-2),*(pchar-3));
			return TRUE;
		}
		else
		{
			return FALSE;
		}
#endif

	}
	else
	{
		return FALSE;
	}


}
bool check_clvl(char * str)
{
	u8 clvl_temp = 0;
	if(!get_clvl(str,&clvl_temp))
	{
		Ql_Debug_Trace("<-- get_clvl failed! -->\r\n");
		return FALSE;
	}

	if(sys_info.system_test_runtime_cnt == 1)
	{
		g_nv_value.clvl = clvl_temp;
		Ql_Debug_Trace("<-- clvl first get:%d -->\r\n",g_nv_value.clvl);
		return TRUE;

	}
	else
	{
		if(clvl_temp != g_nv_value.clvl)
		{
			Ql_Debug_Trace("<-- clvl check failed! pre:%d,now:%d -->\r\n",clvl_temp,g_nv_value.clvl);
			return FALSE;
		}
		else
		{
			return TRUE;
		}
	}
}

bool get_cmee(char * str,u8 * p_out)//+CLVL: 60
{
	char * pchar = NULL;
	//unsigned char ret = 255;
	u8 unit = 0;
	u8 ten =0;
	u8 hundred = 0;
	u8 gap = Ql_strlen("+CMEE: ")-1;
	pchar = Ql_strstr(str, "+CMEE: ");
	if( pchar != NULL)
	{

		unit = *(pchar+gap) -'0';
		//ten  = *(pchar+7) -'0';
		*p_out = unit;
		Ql_Debug_Trace("<-- get_cmee value:%d -->\r\n",*p_out);
		return TRUE;

	}
	else
	{
		Ql_Debug_Trace("<-- get_cmee failed! -->\r\n");
		return FALSE;
	}


}
bool check_cmee(char * str)
{
	u8 temp = 0;
	if(!get_cmee(str,&temp))
	{

		return FALSE;
	}

	if(sys_info.system_test_runtime_cnt == 1)
	{
		g_nv_value.cmee = temp;
		Ql_Debug_Trace("<-- cmee first get:%d -->\r\n",g_nv_value.cmee);
		return TRUE;

	}
	else
	{
		if(temp != g_nv_value.cmee)
		{
			Ql_Debug_Trace("<-- cmee check failed! pre:%d,now:%d -->\r\n",temp,g_nv_value.cmee);
			return FALSE;
		}
		else
		{
			return TRUE;
		}
	}
}

bool get_qmic(char * str,qmic_struct * p_out)//+CLVL: 60
{
	char * pchar = NULL;
	//unsigned char ret = 255;
	u8 unit = 0;
	u8 ten =0;
	u8 hundred = 0;
	u8 gap = Ql_strlen("+QMIC: ");
	pchar = Ql_strstr(str, "+QMIC: ");
	if( pchar != NULL)
	{
		pchar+=gap;

		do
		{
			unit*=10;
			unit += *(pchar) -'0';
			pchar++;
		}
		while((*pchar) != ',') ;
		pchar++;

		p_out->normal = unit;
		unit = 0;
		do
		{
			unit*=10;
			unit += *(pchar) -'0';
			pchar++;
		}
		while((*pchar) != ',') ;
		pchar++;
		p_out->headset = unit;
		unit = 0;
#if 0
		do
		{
			unit*=10;
			unit += *(pchar) -'0';
			pchar++;
		}
		while((*pchar) != '\r') ;
#endif
		//unit*=10;
		ten = *(pchar) -'0';
		pchar++;
		unit = ten*10;
		unit += *(pchar) -'0';
		pchar++;


		p_out->loudspeaker= unit;


		Ql_Debug_Trace("<-- get QMIC normal:%d,headset:%d,loudspeaker:%d -->\r\n",p_out->normal,p_out->headset,p_out->loudspeaker);
		return TRUE;

	}
	else
	{
		Ql_Debug_Trace("<-- get QMIC failed! -->\r\n");
		return FALSE;
	}


}

bool check_qmic(char * str,qmic_struct *p_qmic_nv_now)
{
	qmic_struct * p_temp = NULL;//{0};
	p_temp = (qmic_struct * )Ql_MEM_Alloc(sizeof(qmic_struct));
	if(!get_qmic(str,p_temp))
	{

		return FALSE;
	}

	if(sys_info.system_test_runtime_cnt == 1)
	{
		p_qmic_nv_now->normal = p_temp->normal;
		p_qmic_nv_now->headset= p_temp->headset;
		p_qmic_nv_now->loudspeaker= p_temp->loudspeaker;

		Ql_Debug_Trace("<-- qmic first get normal:%d,headset:%d,loudspeaker:%d -->\r\n",p_qmic_nv_now->normal,p_qmic_nv_now->headset,p_qmic_nv_now->loudspeaker);
		if(p_temp != NULL)
		{
			Ql_MEM_Free(p_temp);
			p_temp = NULL;
		}
		return TRUE;

	}
	else
	{
#if 1
		if((p_temp->normal != p_qmic_nv_now->normal)|| (p_temp->headset != p_qmic_nv_now->headset) || (p_temp->loudspeaker!= p_qmic_nv_now->loudspeaker))
		{
			Ql_Debug_Trace("<-- qmic check failed! -->\r\n");
			Ql_Debug_Trace("<-- qmic pre: normal:%d,headset:%d,loudspeaker:%d -->\r\n",p_temp->normal,p_temp->headset,p_temp->loudspeaker);
			Ql_Debug_Trace("<-- qmic now: normal:%d,headset:%d,loudspeaker:%d -->\r\n",p_qmic_nv_now->normal,p_qmic_nv_now->headset,p_qmic_nv_now->loudspeaker);

			if(p_temp != NULL)
			{
				Ql_MEM_Free(p_temp);
				p_temp = NULL;
			}
			return FALSE;
		}
		else
		{
			if(p_temp != NULL)
			{
				Ql_MEM_Free(p_temp);
				p_temp = NULL;
			}
			return TRUE;
		}
#endif
	}
}

bool get_nv_unit(char * str_src,char * str_search,u8 * p_out)//+CLVL: 60
{
	char * pchar = NULL;
	//unsigned char ret = 255;
	u8 unit = 0;
	u8 ten =0;
	u8 hundred = 0;
	u8 gap = Ql_strlen(str_search);
	pchar = Ql_strstr(str_src, str_search);
	if( pchar != NULL)
	{

		unit = *(pchar+gap) -'0';
		//ten  = *(pchar+7) -'0';
		*p_out = unit;
		Ql_Debug_Trace("<-- get %s value :%d -->\r\n",str_search,*p_out);
		return TRUE;

	}
	else
	{
		Ql_Debug_Trace("<-- get %s value failed! -->\r\n",str_search);
		return FALSE;
	}


}
bool check_nv_unit(char * str_src,char * str_search,u8* p_nv_value_now)
{
	u8 temp = 0;
	if(!get_nv_unit(str_src,str_search,&temp))
	{

		return FALSE;
	}

	if(sys_info.system_test_runtime_cnt == 1)
	{
		*p_nv_value_now = temp;
		Ql_Debug_Trace("<--  NV: %s first get:%d -->\r\n",str_search ,*p_nv_value_now);
		return TRUE;

	}
	else
	{
		if(temp != *p_nv_value_now)
		{
			Ql_Debug_Trace("<-- NV:%s check failed! pre:%d,now:%d -->\r\n",str_search,temp,*p_nv_value_now);
			return FALSE;
		}
		else
		{
			return TRUE;
		}
	}
}

/*

   AT+QIREGAPP?

   +QIREGAPP: "abc","",""
   OK
   */

/*
   AT+QICSGP=1

   +QICSGP: 1,"abc","","",1

   OK
   */

bool get_nv_apn(char * str_src,char * p_apn)
{
	u8* p_char = NULL;
	u8* p_start = NULL;
	u8* p_end = NULL;
#ifdef TEST_OBJECT_UC15	
	p_char = Ql_strstr(str_src, "+QICSGP: ");
#else


	p_char = Ql_strstr(str_src, "+QIREGAPP: ");
#endif
	if(p_char != NULL)
	{

		if((p_start = Ql_strstr(p_char,"\"")) != NULL)
		{
			if((p_end = Ql_strstr(p_start+1,"\"")) != NULL)
			{
				Ql_memset(p_apn,0x0,APN_LENGTH);
				Ql_memcpy(p_apn,p_start+1,(u8)(p_end-p_start -1));
				Ql_Debug_Trace("<-- get apn:%s -->\r\n",p_apn);
				return TRUE;	
			}
			else
			{
				Ql_Debug_Trace("<-- get apn end \" not found -->\r\n");
			}

		}
		else
		{
			Ql_Debug_Trace("<-- get apn start \" not found -->\r\n");
		}
	}
	else
	{
		Ql_Debug_Trace("<-- get apn +QIREGAPP: not found -->\r\n");

	}
	return FALSE;	


}
/*
   [2015-02-05_16:10:03]AT$MYNETCON?



   [2015-02-05_16:10:03]$MYNETCON: 0,"APN","ABC"
   [2015-02-05_16:10:03]$MYNETCON: 0,"USERPWD",""
   [2015-02-05_16:10:03]$MYNETCON: 0,"AUTH",1
   [2015-02-05_16:10:03]$MYNETCON: 0,"CFGT",100
   [2015-02-05_16:10:03]$MYNETCON: 0,"CFGP",1024
   [2015-02-05_16:10:03]$MYNETCON: 1,"APN",""
   [2015-02-05_16:10:03]$MYNETCON: 1,"USERPWD",""
   [2015-02-05_16:10:03]$MYNETCON: 1,"AUTH",1
   [2015-02-05_16:10:03]$MYNETCON: 1,"CFGT",100
   [2015-02-05_16:10:03]$MYNETCON: 1,"CFGP",1024

   [2015-02-05_16:10:03]OK
   */
bool get_nv_apn_gw(char * str_src,char * p_apn)
{
	u8* p_char = NULL;
	u8* p_start = NULL;
	u8* p_end = NULL;

	p_char = Ql_strstr(str_src, "$MYNETCON: 0,\"APN\"");
	if(p_char != NULL)
	{
		if((p_start = Ql_strstr(p_char,"\"")) != NULL)
		{
			if((p_end = Ql_strstr(p_start+1,"\"")) != NULL)
			{


			}
			else
			{
				return FALSE;	
			}
		}
		else
		{
			return FALSE;	
		}
		if((p_start = Ql_strstr(p_end+1,"\"")) != NULL)
		{
			if((p_end = Ql_strstr(p_start+1,"\"")) != NULL)
			{
				Ql_memset(p_apn,0x0,APN_LENGTH);
				Ql_memcpy(p_apn,p_start+1,(u8)(p_end-p_start -1));
				Ql_Debug_Trace("<-- get apn:%s -->\r\n",p_apn);
				return TRUE;	
			}
			else
			{
				Ql_Debug_Trace("<-- get apn end \" not found -->\r\n");
			}

		}
		else
		{
			Ql_Debug_Trace("<-- get apn start \" not found -->\r\n");
		}
	}
	else
	{
		Ql_Debug_Trace("<-- get apn : not found -->\r\n");

	}
	return FALSE;	


}


bool check_nv_apn(char *str_src,char * str_check)
{
	s32 ret = -1;
#ifdef TEST_OBJECT_GW
	if(!get_nv_apn_gw(str_src,(g_nv_value.apn)))
	{
		return FALSE;	
	}
#else

	if(!get_nv_apn(str_src,(g_nv_value.apn)))
	{
		return FALSE;	
	}
#endif

	ret = Ql_strcmp(g_nv_value.apn,str_check);
	Ql_Debug_Trace("<-- g_nv_value.apn:%s input check:%s->\r\n",g_nv_value.apn,str_check);
	if(ret != 0)
		return FALSE;
	else
	{
		Ql_memcpy(&(g_nv_value.apn_pre), &(g_nv_value.apn), sizeof(g_nv_value.apn));
		return TRUE;
	}

}


unsigned char check_gsm_network_status(char * str)//+CREG: 0,2
{
	char * pchar = NULL;
	unsigned char ret = 255;
	pchar = Ql_strstr(str, "+CREG:");
	if( pchar != NULL)
	{
		ret = *(pchar+9) -'0';	
	}
	return ret;

}

unsigned char check_gprs_network_status(char * str)//+CGREG: 0,2
{
	char * pchar = NULL;
	unsigned char ret = 255;
	pchar = Ql_strstr(str, "+CGREG:");
	if( pchar != NULL)
	{
		ret = *(pchar+10) -'0';	
	}
	return ret;

}
#if 0
s32 ATResponse_Handler(char* line, u32 len, void* userData)
{
	send_ATC( (u8*)line, len);

	if (Ql_RIL_FindLine(line, len, "OK"))
	{  
		return  RIL_ATRSP_SUCCESS;
	}
	else if (Ql_RIL_FindLine(line, len, "ERROR"))
	{  
		return  RIL_ATRSP_FAILED;
	}
	else if (Ql_RIL_FindString(line, len, "+CME ERROR"))
	{
		return  RIL_ATRSP_FAILED;
	}
	else if (Ql_RIL_FindString(line, len, "+CMS ERROR:"))
	{
		return  RIL_ATRSP_FAILED;
	}
	return RIL_ATRSP_CONTINUE; //continue wait
}



void test_send_at(void)
{
	unsigned char i =0;
	unsigned char at_string1[] = "AT\r\n";
	unsigned char at_string2[] = "ATI\r\n";	


	for(i= 0;i<20;i++)
	{
		send_ATC( at_string1, Ql_strlen(at_string1));
		Ql_Sleep(1500);
		//Ql_GPIO_SetLevel(gpioPin, PINLEVEL_LOW);
		//Ql_Debug_Trace("<-- Get the GPIO level value: %d -->\r\n\r\n", Ql_GPIO_GetLevel(gpioPin));

		// Set the GPIO level to high after 500ms.
		//Ql_Debug_Trace("<-- Set the GPIO level to high after 500ms -->\r\n");

		send_ATC( at_string2, Ql_strlen(at_string2));
		Ql_Sleep(1000);
		// Ql_GPIO_SetLevel(gpioPin, PINLEVEL_HIGH);
		//Ql_Debug_Trace("<-- Get the GPIO level value: %d -->\r\n", Ql_GPIO_GetLevel(gpioPin));
	}
}
#endif
void app_start_AT_check(void)
{
	//unsigned char at_string1[50] = {0};
	//Ql_memcpy(void * dest, const void * src, u32 size)
	Ql_UART_Write(UART_PORT_INTER, "AT+CSQ\r\n", Ql_strlen(("AT+CSQ\r\n")));
	//sys_info.status = SYS_STATE_CSQ;
}
bool set_ipr(void)
{
	//Ql_UART_Write(UART_PORT_INTER, "AT+IPR=115200\r\n", Ql_strlen(("AT+IPR=115200\r\n"))); 
	//return send_ATC("AT+IPR=115200;\r\nAT+QMIC=2,10;\r\n\AT+CLVL=10;\r\nAT+CMGF=1;\r\nAT+CMEE=2;\r\nAT+CCWA=1;\r\nAT+QICSGP=1,\"abc\";\r\n");
	//return send_ATC("AT+IPR=115200;AT+QMIC=2,10;AT+CLVL=10;AT+CMGF=1;AT+CMEE=2;AT+CCWA=1;AT+QICSGP=1,\"abc\";\r\n");
	//return send_ATC("AT+IPR=115200;+QMIC=2,10\r\n");

	//return send_ATC("AT+IPR=115200;+CLVL=6;+CMGF=1;+CMEE=2;+CCWA=1;+QICSGP=1,\"abc\"\r\n");

	//return send_ATC("AT+IPR=115200\r\n");
#ifdef TEST_OBJECT_UC15
	return send_ATC("AT+IPR=115200;+CLVL=6;+CMGF=1;+CMEE=2;+CCWA=1;+QICSGP=1,1,\"abc\",\"\",\"\"\r\n");
#else
#ifdef TEST_OBJECT_GW
	//return send_ATC("AT+IPR=115200;\r\nAT+QMIC=2,10;\r\n\AT+CLVL=10;\r\nAT+CMGF=1;\r\nAT+CMEE=2;\r\nAT+CCWA=1;\r\nAT$MYNETCON=0,\"APN\",\"abc\";\r\n");
	return send_ATC("AT+IPR=115200;\r\nAT+QMIC=2,10;\r\n\AT+CLVL=10;\r\nAT+CMGF=1;\r\nAT+CMEE=2;\r\nAT+CCWA=1;\r\n");
#else
	return send_ATC("AT+IPR=115200;\r\nAT+QMIC=2,10;\r\n\AT+CLVL=10;\r\nAT+CMGF=1;\r\nAT+CMEE=2;\r\nAT+CCWA=1;\r\nAT+QICSGP=1,\"abc\";\r\n");
#endif

#endif

	//at+IPR=115200;+QMIC=1,10;+CLVL=1

}


unsigned int random(void)
{

	unsigned int ret = 0;
	u64 totalMS = Ql_GetMsSincePwrOn();

	srand(totalMS);
	ret = (rand()%(RANGE_MAX-RANGE_MIN))+RANGE_MIN;

	return ret;
}
void sys_hangup(void)
{
	//power_ctl(FALSE);
	//Ql_UART_Write(UART_PORT_CONSOLE, "power off the slave device\r\n", Ql_strlen("power off the slave device\r\n"));
	Ql_Timer_Stop(power_ctl_timer.id);

	Ql_Timer_Stop(system_timer.id);
	Ql_Timer_Stop(at_sync_timer.id);
	Ql_Timer_Stop(throwout_error_timer.id);


	//sys_info.status = SYS_STATE_IDLE;
	sys_info.system_timer_cnt = 0;
	sys_info.network_error_cnt = 0;
	flag_creg_success = FALSE;
	flag_cgreg_success = FALSE;

	//while(1);
}

void all_timer_stop(void)
{
	Ql_Timer_Stop(power_ctl_timer.id);
	//Ql_Timer_Stop(system_timer.id);
	Ql_Timer_Stop(at_sync_timer.id);
	Ql_Timer_Stop(atc_timer.id);
	Ql_Timer_Stop(led.timer.id);
	Ql_Timer_Stop(throwout_error_timer.id);

}

void start_throwout_error(Enum_error e_error)
{
	s32 ret = 0;
	//sys_hangup();
	all_timer_stop();
	led_flicker(TRUE);
	sys_info.error_type = e_error;

	throwout_error_timer.interval = 1000;
	throwout_error_timer.auto_repeat = TRUE;
	//ret = Ql_Timer_Start(throwout_err_timer_id,1000,TRUE);//
	if(!timer_start(&throwout_error_timer))
	{
		//Ql_Debug_Trace("\r\n<-- failed!! throwout_err_timer_id Ql_Timer_Start fail, ret=%d -->\r\n",ret);
	}

}
void throwout_error(Enum_error e_error)
{

#if 1
	Ql_Debug_Trace("<-- start_throwout_error error num:%d sys status:%d ->\r\n",e_error,sys_info.status);

	switch(e_error)
	{
		case ERROR_CPIN:
			{
				Ql_memset(RxBuf_Uart_Console, 0x0, SERIAL_RX_BUFFER_LEN);
				Ql_UART_Write(UART_PORT_CONSOLE, "ERROR_CPIN\r\n", Ql_strlen("ERROR_CPIN\r\n"));
			}
			break;
		case ERROR_NV:
			{
				Ql_memset(RxBuf_Uart_Console, 0x0, SERIAL_RX_BUFFER_LEN);
				Ql_UART_Write(UART_PORT_CONSOLE, "ERROR_NV\r\n", Ql_strlen("ERROR_NV\r\n"));
			}
			break;
		case ERROR_NV_ATI:
			{
				Ql_memset(RxBuf_Uart_Console, 0x0, SERIAL_RX_BUFFER_LEN);
				Ql_UART_Write(UART_PORT_CONSOLE, "ERROR_NV_ATI\r\n", Ql_strlen("ERROR_NV_ATI\r\n"));
			}
			break;
		case ERROR_NV_IMEI:
			{
				Ql_memset(RxBuf_Uart_Console, 0x0, SERIAL_RX_BUFFER_LEN);
				Ql_UART_Write(UART_PORT_CONSOLE, "ERROR_NV_IMEI\r\n", Ql_strlen("ERROR_NV_IMEI\r\n"));
			}
			break;
		case ERROR_APN:
			{
				Ql_memset(RxBuf_Uart_Console, 0x0, SERIAL_RX_BUFFER_LEN);
				Ql_UART_Write(UART_PORT_CONSOLE, "ERROR_APN\r\n", Ql_strlen("ERROR_APN\r\n"));
				Ql_Debug_Trace("<-- ERROR_APN apn_pre:%s apn now:%s->\r\n",g_nv_value.apn_pre,g_nv_value.apn);

			}

			break;
		case ERROR_IMEI:
			{
				Ql_memset(RxBuf_Uart_Console, 0x0, SERIAL_RX_BUFFER_LEN);
				Ql_UART_Write(UART_PORT_CONSOLE, "ERROR_IMEI\r\n", Ql_strlen("ERROR_IMEI\r\n"));
			}
			break;
		case ERROR_GSM:
			{
				Ql_memset(RxBuf_Uart_Console, 0x0, SERIAL_RX_BUFFER_LEN);
				Ql_UART_Write(UART_PORT_CONSOLE, "ERROR_GSM\r\n", Ql_strlen("ERROR_GSM\r\n"));
			}

			break;
		case ERROR_GPRS:
			{
				Ql_memset(RxBuf_Uart_Console, 0x0, SERIAL_RX_BUFFER_LEN);
				Ql_UART_Write(UART_PORT_CONSOLE, "ERROR_GPRS\r\n", Ql_strlen("ERROR_GPRS\r\n"));
			}
			break;
		case ERROR_UNKNOWN:
			{
				Ql_memset(RxBuf_Uart_Console, 0x0, SERIAL_RX_BUFFER_LEN);
				Ql_UART_Write(UART_PORT_CONSOLE, "ERROR_UNKNOWN\r\n", Ql_strlen("ERROR_UNKNOWN\r\n"));
			}
			break;
		case ERROR_AT_RETURN:
			{
				Ql_memset(RxBuf_Uart_Console, 0x0, SERIAL_RX_BUFFER_LEN);
				Ql_UART_Write(UART_PORT_CONSOLE, "ERROR_AT_RETURN\r\n", Ql_strlen("ERROR_AT_RETURN\r\n"));
			}
			break;

		case ERROR_AT_SYNC_NO_RESPONSE:
			{
				Ql_memset(RxBuf_Uart_Console, 0x0, SERIAL_RX_BUFFER_LEN);
				Ql_UART_Write(UART_PORT_CONSOLE, "ERROR_AT_SYNC_NO_RESPONSE\r\n", Ql_strlen("ERROR_AT_SYNC_NO_RESPONSE\r\n"));
			}
			break;
		case ERROR_AT_SYNC_NO_RESPONSE_TIMEOUT:
			{
				Ql_memset(RxBuf_Uart_Console, 0x0, SERIAL_RX_BUFFER_LEN);
				Ql_UART_Write(UART_PORT_CONSOLE, "ERROR_AT_SYNC_NO_RESPONSE_TIMEOUT\r\n", Ql_strlen("ERROR_AT_SYNC_NO_RESPONSE_TIMEOUT\r\n"));
			}
			break;	
		case ERROR_SYSTEM_POWER_ON_FAILED:
			{
				Ql_memset(RxBuf_Uart_Console, 0x0, SERIAL_RX_BUFFER_LEN);
				Ql_UART_Write(UART_PORT_CONSOLE, "ERROR_SYSTEM_POWER_ON_FAILED\r\n", Ql_strlen("ERROR_SYSTEM_POWER_ON_FAILED\r\n"));
			}
			break;
		default:
			break;

	}


	//while(1);
#endif
}


#endif
