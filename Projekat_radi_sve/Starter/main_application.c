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

	/* TASK PRIORITIES */
#define SM_TASK_PRI					( tskIDLE_PRIORITY + 1 )
#define	SERVICE_TASK_PRI			( tskIDLE_PRIORITY + 2 )
#define	TASK_SERIAL_SEND_PRI		( tskIDLE_PRIORITY + 3 )
#define TASK_SERIAl_REC_PRI			( tskIDLE_PRIORITY + 4 )


/* TASKS: FORWARD DECLARATIONS */
void led_bar_tsk( void *pvParameters );
void SerialSend_Task(void* pvParameters);
void SerialReceive_Task(void* pvParameters);
void SM_Task(void* pvParameters);

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

/*DATA STRUCTS*/
typedef struct sensor_val{/*struct for holding sensor values*/
	uint8_t air_temp;
	uint8_t coolant_temp;
	uint16_t revs;
	uint8_t manifold_air_press;
	uint8_t gas_pedal_pos;
}sensor_val;


/* GLOBAL OS-HANDLES */
SemaphoreHandle_t LED_INT_BinarySemaphore;
SemaphoreHandle_t TBE_BinarySemaphore;
SemaphoreHandle_t RXC_BinarySemaphore;
TimerHandle_t per_TimerHandle;
QueueHandle_t SensorQueue = NULL;

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

	xSemaphoreGiveFromISR(RXC_BinarySemaphore, &xHigherPTW);

	portYIELD_FROM_ISR(xHigherPTW);
}

/* PERIODIC TIMER CALLBACK */
static void TimerCallback(TimerHandle_t xTimer)
{ 
	xSemaphoreGive(TBE_BinarySemaphore);
}

/*STATE MACHINE FOR SYSTEM*/
void SM_Task(void* pvParameters) 
{
	sensor_val SensTemp;
	xQueueReceive(SensorQueue, &SensTemp , portMAX_DELAY);
	printf("u sm: %u\n", (unsigned)SensTemp.air_temp);/*for debug*/
	printf("u sm: %u\n", (unsigned)SensTemp.coolant_temp);/*for debug*/
	printf("u sm: %u\n", (unsigned)SensTemp.revs);/*for debug*/
	printf("u sm: %u\n", (unsigned)SensTemp.manifold_air_press);/*for debug*/
	printf("u sm: %u\n", (unsigned)SensTemp.gas_pedal_pos);/*for debug*/

}


/* MAIN - SYSTEM STARTUP POINT */
void main_demo( void )
{
	init_7seg_comm();
	init_LED_comm();
	init_serial_uplink(COM_CH);  // inicijalizacija serijske TX na kanalu 0
	init_serial_downlink(COM_CH);// inicijalizacija serijske TX na kanalu 0

	/* QUEUE  */
	SensorQueue = xQueueCreate(5u, sizeof(sensor_val));

	/*STATE MACHINE TASK*/
	xTaskCreate(SM_Task, "SM", configMINIMAL_STACK_SIZE, NULL, SM_TASK_PRI, NULL);

	/* ON INPUT CHANGE INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_OIC, OnLED_ChangeInterrupt);

	/* Create LED interrapt semaphore */
	LED_INT_BinarySemaphore = xSemaphoreCreateBinary();

	/* create a timer task */
	per_TimerHandle = xTimerCreate("Timer", pdMS_TO_TICKS(100), pdTRUE, NULL, TimerCallback);
	xTimerStart(per_TimerHandle, 0);

	

	/* SERIAL TRANSMITTER TASK */
	xTaskCreate(SerialSend_Task, "STx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);

	/* SERIAL RECEIVER TASK */
	xTaskCreate(SerialReceive_Task, "SRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAl_REC_PRI, NULL);
	r_point = 0;

	/* Create TBE semaphore - serial transmit comm */
	TBE_BinarySemaphore = xSemaphoreCreateBinary();

	/* Create TBE semaphore - serial transmit comm */
	RXC_BinarySemaphore = xSemaphoreCreateBinary();

	/* SERIAL TRANSMISSION INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_TBE, prvProcessTBEInterrupt);

	/* SERIAL RECEPTION INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);

	/* create a led bar TASK */
	xTaskCreate(led_bar_tsk, "ST",	configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);

	vTaskStartScheduler();

	while (1);
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

void SerialReceive_Task(void* pvParameters)
{
	uint8_t cc = 0;
	static uint8_t loca = 0;
	sensor_val SensTemp;
	while (1)
	{
		xSemaphoreTake(RXC_BinarySemaphore, portMAX_DELAY);/*suspend task until a character is recieved*/
		get_serial_character(COM_CH, &cc);
		printf("primio karakter: %u\n", (unsigned)cc);/*for debug*/
		
		if (cc == 0x00) /*initialise recieve buffer*/
		{		
			r_point = 0;
			
		}
		else if (cc == 0xff)/*end character case*/
		{
			/*load SensTemp with r_buffer*/
			SensTemp.air_temp = r_buffer[0];
			SensTemp.coolant_temp = r_buffer[1];
			SensTemp.revs = ((uint16_t)(r_buffer[2]<<8u))|(uint16_t)r_buffer[3];
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