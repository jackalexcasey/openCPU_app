#ifndef __CUS_POWER_CTL_H__
#define __CUS_POWER_CTL_H__


//#define TEST_OBJECT_UC15
#define TEST_OBJECT_GW

//#define TEST_OBJECT_UC15_NO_AT
#define TEST_MODE_PWRKEY_EMERGOFF

#define SERIAL_RX_BUFFER_LEN  2048

// Define the UART port and the receive data buffer
#define  UART_PORT_INTER  UART_PORT3 //to comunicate with two module
#define  UART_PORT_CONSOLE UART_PORT1//for controle from pc to main module

#define TIMEOUT_COUNT 2
#define MODE_POWD_PWR_EN 0
#define MODE_POWER_KEY    1

#define NETWORK_ERROR_MAX_TIME 360 //S
#define NETWORK_ERROR_MAX_CNT  3 //


typedef enum 
{
	SYS_STATE_IDLE=0,
	SYS_STATE_CSQ ,
	SYS_STATE_CREG,
	SYS_STATE_CGREG,
	SYS_STATE_EGMR07,
	SYS_STATE_EGMR05,
	SYS_STATE_CPIN,
	SYS_STATE_QPOWD,
	SYS_STATE_POWER_ON,
	SYS_STATE_POWER_OFF,
	SYS_STATE_POWER_RESTART_SIG,
	SYS_STATE_IPR_SET,
	SYS_STATE_IPR_READ,
	SYS_STATE_ATW,
	SYS_STATE_ATI,
	SYS_STATE_ATE1V1,
	SYS_STATE_IMEI,
	SYS_STATE_APN,
	SYS_STATE_APN_SET,//18
	SYS_STATE_QMIC,
	SYS_STATE_CLVL,
	SYS_STATE_CMGF,
	SYS_STATE_CMEE,
	SYS_STATE_CCWA,
	SYS_STATE_END
}Enum_SYS_State;
typedef enum 
{
	ERROR_CPIN=0,
	ERROR_NV,
	ERROR_NV_ATI,
	ERROR_NV_IMEI,
	ERROR_APN,
	ERROR_IMEI,
	ERROR_GSM,
	ERROR_GPRS,
	ERROR_AT_RETURN,//8
	ERROR_AT_SYNC_NO_RESPONSE,
	ERROR_AT_SYNC_NO_RESPONSE_TIMEOUT,
	ERROR_SYSTEM_POWER_ON_FAILED,
	ERROR_UNKNOWN,
	ERROR_END
}Enum_error;


typedef struct 
{
	u32 id;
	Callback_Timer_OnTimer callback_onTimer;
	void * param;
	u32 interval;
	bool auto_repeat;
	//Timer_Start_type start;

}timer_struct_type;
typedef s32 (*Timer_Start_type)(timer_struct_type * p_timer);
typedef struct 
{
	timer_struct_type timer;
	bool flag_fast_slow;

}led_struct_type;

typedef struct
{
	u32 sec_2;
	u32 sec_3;
	u32 sec_4;
	u32 sec_5;
	u32 sec_6;
	u32 invalid;
	

}random_time_cnt_type;
//Statistical information system
typedef struct
{
	Enum_SYS_State status;
	u32 system_timer_cnt;//for network time check
	u32 total_time;
	u16 network_error_cnt;
	u16 creg_network_error_cnt;
	u16 cgreg_network_error_cnt;
	u16 cpin_error_cnt;
	u32 system_test_runtime_cnt;
	Enum_error error_type;
	u8 mode;
	unsigned char at_sync_cnt ;
	bool  flag_apn_check;
	random_time_cnt_type	random_cnt;
	u8 sync_failed_timeout_cnt;
	u16 sys_crush_cnt ;
	//s32 totalBytes_all ;
}sys_info_statistic_type;


#define IMEI_LENGTH 15
#define APN_LENGTH 5
//u8 imei_str[IMEI_LENGTH+1] = {0};

typedef struct
{
	u8 normal;
	u8 headset;
	u8 loudspeaker;

}qmic_struct;
typedef struct tag_nvram_value
{
	qmic_struct qmic;
	u8 clvl;
	u8 cmee;
	u8 cmgf;
	u8 ccwa;
	u8 imei_str[IMEI_LENGTH+1];
	u8 apn[APN_LENGTH+1];
	u8 apn_pre[APN_LENGTH+1];

}nvram_value_struct;


typedef struct
{
	u8 rx_buf[SERIAL_RX_BUFFER_LEN] ;//buffer
	u16 in_cnt;//Effective data length

}data_frame_type;


//extern u8 RxBuf_Uart_Console[];
extern u8 RxBuf_Uart_Inter[] ;


//init 
void application_init(void);

void start_power_test(void);

#endif
