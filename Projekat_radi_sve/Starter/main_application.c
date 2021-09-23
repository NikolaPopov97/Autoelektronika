/* Standard includes. */
#include <stdio.h>l.g 
#include <conio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "extint.h"

/* Hardware simulator utility functions */
#include "HW_access.h"

/* SERIAL SIMULATOR CHANNEL TO USE */
#define COM_CH (0)
#define COM_CH_1 (1)

	/* TASK PRIORITIES */
#define SM_TASK_PRI					( tskIDLE_PRIORITY + 1 )
#define	SERVICE_TASK_PRI			( tskIDLE_PRIORITY + 2 )
#define TASK_SDH_PRI				( tskIDLE_PRIORITY + 3 )
#define	TASK_SERIAL_SEND_PRI		( tskIDLE_PRIORITY + 4 )
#define TASK_PC_SERIAL_REC			( tskIDLE_PRIORITY + 5 )
#define TASK_SERIAl_REC_PRI			( tskIDLE_PRIORITY + 6 )


/* TASKS: FORWARD DECLARATIONS */
void led_bar_tsk( void *pvParameters );
void SerialSend_Task(void* pvParameters);
void SerialReceive_Task(void* pvParameters);
void PC_SerialReceive_Task(void* pvParameters);
void SensorDataHandler(void* pvParameters);
void SM_Task(void* pvParameters);

/*Local function declarations*/
void takeAllStateSem(void);

/* TRASNMISSION DATA - CONSTANT IN THIS APPLICATION */
const char trigger[] = "XYZ";
unsigned volatile t_point;

/* RECEPTION DATA BUFFER */
#define R_BUF_SIZE (6)
uint8_t r_buffer[R_BUF_SIZE];
unsigned volatile r_point;

/* 7-SEG NUMBER DATABASE - ALL HEX DIGITS */
static const char hexnum[] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 
								0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71 };

/*STATES*/
#define MONITOR 'm'
#define DRIVE   'd'
#define SPEED   's'
#define OFF     'o'

/*DATA STRUCTS*/
typedef struct sensor_val{/*struct for holding sensor values*/
	uint8_t air_temp;
	uint8_t coolant_temp;
	uint16_t revs;
	uint8_t manifold_air_press;
	uint8_t gas_pedal_pos;
}sensor_val;

typedef struct monitor_val {
	uint8_t air_temp;
	uint8_t coolant_temp;
}monitor_val;

typedef struct drive_val {
	uint16_t revs;
	uint8_t manifold_air_press;
}drive_val;

typedef struct speed_val {
	uint16_t revs;
	uint8_t gas_pedal_pos;
}speed_val;


/* GLOBAL OS-HANDLES */
SemaphoreHandle_t LED_INT_BinarySemaphore;
SemaphoreHandle_t TBE_BinarySemaphore;
SemaphoreHandle_t RXC_BinarySemaphore;
SemaphoreHandle_t RXC_PCSemaphore;
SemaphoreHandle_t MonitorStateSem;
SemaphoreHandle_t DriveStateSem;
SemaphoreHandle_t SpeedStateSem;
SemaphoreHandle_t OffStateSem;
SemaphoreHandle_t AlarmStateSem;
TimerHandle_t per_TimerHandle;
QueueHandle_t SensorQueue;
QueueHandle_t MessageQueue;
QueueHandle_t MonitorQueue;
QueueHandle_t DriveQueue;
QueueHandle_t SpeedQueue;

/* OPC - ON INPUT CHANGE - INTERRUPT HANDLER */
static uint32_t OnLED_ChangeInterrupt(void)
{
	BaseType_t xHigherPTW = pdFALSE;

	xSemaphoreGiveFromISR(LED_INT_BinarySemaphore, &xHigherPTW);

	portYIELD_FROM_ISR(xHigherPTW);
}

/* TBE - TRANSMISSION BUFFER EMPTY - INTERRUPT HANDLER */
static uint32_t prvProcessTBEInterrupt(void)
{
	BaseType_t xHigherPTW = pdFALSE;

	xSemaphoreGiveFromISR(TBE_BinarySemaphore, &xHigherPTW);

	portYIELD_FROM_ISR(xHigherPTW);
}

/* RXC - RECEPTION COMPLETE - INTERRUPT HANDLER */
static uint32_t prvProcessRXCInterrupt(void)
{
	BaseType_t xHigherPTW = pdFALSE;

	if (get_RXC_status(0) != 0)
		xSemaphoreGiveFromISR(RXC_BinarySemaphore, &xHigherPTW);

	if (get_RXC_status(1) != 0)
		xSemaphoreGiveFromISR(RXC_PCSemaphore, &xHigherPTW);

	portYIELD_FROM_ISR(xHigherPTW);
}

/* PERIODIC TIMER CALLBACK */
static void TimerCallback(TimerHandle_t xTimer)
{ 
	xSemaphoreGive(TBE_BinarySemaphore);
}


/* MAIN - SYSTEM STARTUP POINT */
void main_demo( void )
{
	init_7seg_comm();
	init_LED_comm();
	init_serial_uplink(COM_CH);  // inicijalizacija serijske TX na kanalu 0
	init_serial_downlink(COM_CH);// inicijalizacija serijske TX na kanalu 0

	init_serial_uplink(COM_CH_1);
	init_serial_downlink(COM_CH_1);

	/* QUEUES  */
	SensorQueue = xQueueCreate(1u, sizeof(sensor_val));
	MessageQueue = xQueueCreate(5u, sizeof(uint8_t));
	MonitorQueue = xQueueCreate(1u, sizeof(monitor_val));
	DriveQueue = xQueueCreate(1u, sizeof(drive_val));
	SpeedQueue = xQueueCreate(1u, sizeof(speed_val));
	

	/*STATE MACHINE TASK*/
	xTaskCreate(SM_Task, "SM", configMINIMAL_STACK_SIZE, NULL, SM_TASK_PRI, NULL);

	/*Create sensor data handler task*/
	xTaskCreate(SensorDataHandler, "Sdh", configMINIMAL_STACK_SIZE, NULL, TASK_SDH_PRI, NULL);

	/* ON INPUT CHANGE INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_OIC, OnLED_ChangeInterrupt);

	/* Create LED interrapt semaphore */
	LED_INT_BinarySemaphore = xSemaphoreCreateBinary();

	/* create a timer task */
	per_TimerHandle = xTimerCreate("Timer", pdMS_TO_TICKS(100), pdTRUE, NULL, TimerCallback);
	xTimerStart(per_TimerHandle, 0);

	
	/* SERIAL TRANSMITTER TASK */
	xTaskCreate(SerialSend_Task, "STx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);

	/* SERIAL RECEIVER TASKS */
	xTaskCreate(SerialReceive_Task, "SRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAl_REC_PRI, NULL);
	r_point = 0;
	xTaskCreate(PC_SerialReceive_Task, "PCRx", configMINIMAL_STACK_SIZE, NULL, TASK_PC_SERIAL_REC, NULL);


	/* Create TBE semaphore - serial transmit comm */
	TBE_BinarySemaphore = xSemaphoreCreateBinary();

	/* Create TBE semaphore - serial receive comm */
	RXC_BinarySemaphore = xSemaphoreCreateBinary();
	RXC_PCSemaphore = xSemaphoreCreateBinary();

	/*Create State Semaphores*/
	MonitorStateSem = xSemaphoreCreateBinary();
	DriveStateSem = xSemaphoreCreateBinary();
	SpeedStateSem = xSemaphoreCreateBinary();
	OffStateSem = xSemaphoreCreateBinary();
	AlarmStateSem = xSemaphoreCreateBinary(); 

	/* SERIAL TRANSMISSION INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_TBE, prvProcessTBEInterrupt);

	/* SERIAL RECEPTION INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);

	/* create a led bar TASK */
	xTaskCreate(led_bar_tsk, "ST",	configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);

	vTaskStartScheduler();

	while (1);
}



/*STATE MACHINE FOR THE SYSTEM*/
/*This task determens the next state based on Msg input, and unblocks state tasks*/
void SM_Task(void* pvParameters)
{
	uint8_t Msg;
	while (1) {
		xQueueReceive(MessageQueue, &Msg, portMAX_DELAY);/*Task is blocked until a new message arrives*/
		takeAllStateSem();/*wait until a state task finishes and then block them so another state can be enabled*/
		switch (Msg) 
		{
		case MONITOR:
			xSemaphoreGive(MonitorStateSem, portMAX_DELAY);/*enable state task*/
		case DRIVE:
			xSemaphoreGive(DriveStateSem, portMAX_DELAY);/*enable state task*/
		case SPEED:
			xSemaphoreGive(SpeedStateSem, portMAX_DELAY);/*enable state task*/
		case OFF:
			xSemaphoreGive(OffStateSem, portMAX_DELAY);/*Turn off system*/
		default:/*turn on alarm unexpected message is sent*/
			xSemaphoreGive(AlarmStateSem, portMAX_DELAY);
		}
	}
}

/*This task checks if sensor values are within normal operating range,*/
/*then sends sensor data to their state queues*/
void SensorDataHandler(void* pvParameters)
{
	/*Buffers for queues*/
	sensor_val SensTemp;
	monitor_val MonitorTemp;
	drive_val DriveTemp;
	speed_val SpeedTemp;
	while (1)
	{	
		xQueueReceive(SensorQueue,&SensTemp,portMAX_DELAY);/*recieve new sensor data*/
		if(110u < SensTemp.coolant_temp)
		{/*check if engine is overheating*/
			xSemaphoreGive(AlarmStateSem, portMAX_DELAY);
		}
		else if (6000 < SensTemp.gas_pedal_pos)
		{/*check if engine revs are too high*/
			xSemaphoreGive(AlarmStateSem, portMAX_DELAY);
		}
		else
		{/*if sensor data is within range only then disable alarm*/
			xSemaphoreTake(AlarmStateSem, portMAX_DELAY);
		}

		/*Send monitor data to monitor queue*/
		MonitorTemp.air_temp = SensTemp.air_temp;
		MonitorTemp.coolant_temp = SensTemp.coolant_temp;
		xQueueOverwrite(MonitorQueue, &MonitorTemp);
		/*Send drive data to drive queue*/
		DriveTemp.revs = SensTemp.revs;
		DriveTemp.manifold_air_press = SensTemp.manifold_air_press;
		xQueueOverwrite(DriveQueue, &DriveTemp);
		/*Send speed data to drive queue*/
		SpeedTemp.revs = SensTemp.revs;
		SpeedTemp.gas_pedal_pos = SensTemp.gas_pedal_pos;
		xQueueOverwrite(SpeedQueue, &SpeedTemp);

	}
}



void led_bar_tsk(void* pvParameters)
{
	unsigned i;
	uint8_t d;
			while (1)
	{  
		xSemaphoreTake(LED_INT_BinarySemaphore, portMAX_DELAY);
				get_LED_BAR(1, &d);
		i = 3;
		do
		{
			i--;
			select_7seg_digit(i);
			set_7seg_digit(hexnum[d % 10]);
			d /= 10;
		} while (i > 0);
	}
}

/*Task for receiving pc commands and sending them to another task to handle them*/
void PC_SerialReceive_Task(void* pvParameters) 
{
	uint8_t cc;
	uint8_t temp;
	while (1)
	{
		xSemaphoreTake(RXC_PCSemaphore, portMAX_DELAY);/*suspend task until a character is received*/
		get_serial_character(COM_CH, &cc);
		if (0x0d == cc)
		{
			xQueueSend(MessageQueue, &temp, portMAX_DELAY);
		}
		else
		{
			temp = cc;
		}
	}
}

/*Task is used for polling sensor serial line*/
void SerialSend_Task(void* pvParameters)
{
	t_point = 0;
	
	while (1)
	{
		if (t_point > (sizeof(trigger) - 1))/*if we sent all trigger char reset trigger position*/
		{
			t_point = 0;
		}
		if (0u == t_point)/*if next on turn is first char of the trigger block task*/
		{
			xSemaphoreTake(TBE_BinarySemaphore, portMAX_DELAY);/*block task until 100ms have passed*/
		}
		send_serial_character(COM_CH, trigger[t_point++]); /*send trigger for auto transmision*/
		xSemaphoreTake(TBE_BinarySemaphore, portMAX_DELAY);/*block task until transmit buffer is empty*/	
	}
}


/*Task handles receiving sensor data and then puts it in a queue*/
void SerialReceive_Task(void* pvParameters)
{
	uint8_t cc = 0;
	static uint8_t loca = 0;
	sensor_val SensTemp;
	while (1)
	{
		xSemaphoreTake(RXC_BinarySemaphore, portMAX_DELAY);/*suspend task until a character is received*/
		get_serial_character(COM_CH, &cc);
		printf("primio karakter: %u\n", (unsigned)cc);/*for debug*/
		
		if ((0x00 == cc)&&(R_BUF_SIZE == r_point)) /*initialise recieve buffer*/
		{/*second check is if some sensor values are 0x00, so that we don't reinitialise the buffer */
			r_point = 0;
			
		}
		else if ((cc == 0xff)&&(R_BUF_SIZE == r_point))/*end character case*/
		{/*second check is if some sensor values are 0xff, so that we don't finish earlier*/
			/*load SensTemp with r_buffer*/
			SensTemp.air_temp = r_buffer[0];
			SensTemp.coolant_temp = r_buffer[1];
			SensTemp.revs = ((uint16_t)r_buffer[2]<<8u)|(uint16_t)r_buffer[3];
			SensTemp.manifold_air_press = r_buffer[4];
			SensTemp.gas_pedal_pos = r_buffer[5];
			xQueueSend(SensorQueue, &SensTemp, 0U);/*pass r_buffer to queue*/
		}
		else if (r_point < R_BUF_SIZE)// pamti karaktere izmedju 0 i FF
		{
			r_buffer[r_point++] = cc;
		}
		else/*comm error case, activate alarm*/
		{
			/*alarm task or smtng*/
		}
	}
}

/*Function used to take all semaphores*/
void takeAllStateSem(void) {
	xSemaphoreTake(MonitorStateSem, portMAX_DELAY);
	xSemaphoreTake(DriveStateSem, portMAX_DELAY);
	xSemaphoreTake(SpeedStateSem, portMAX_DELAY);
	xSemaphoreTake(OffStateSem, portMAX_DELAY);
}