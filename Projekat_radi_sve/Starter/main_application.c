/* Standard includes. */
#include <stdio.h>l.g 
#include <conio.h>
#include <string.h>

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
#define COM_CH1 (1)
#define COM_CH2 (2)

	/* TASK PRIORITIES */
#define OBRADA_TASK_PRI ( tskIDLE_PRIORITY + 1 )
#define	TASK_SERIAL_SEND_PRI		( tskIDLE_PRIORITY + 3 )
#define TASK_SERIAl_REC_PRI			( tskIDLE_PRIORITY + 4 )
#define	SERVICE_TASK_PRI		( tskIDLE_PRIORITY + 2 )
#define SENZORI_PRI ( tskIDLE_PRIORITY + 5)

/* TASKS: FORWARD DECLARATIONS */
void led_bar_tsk( void *pvParameters ); //ocitavanje sa led bara
void SerialSend_Task(void* pvParameters); //ispis na serijsku 
void SerialReceive_Task(void* pvParameters); //prijem komandi sa serijske
void prijem_sa_senzora_tsk(void* pvParameters); //odredjivanje trenutne temperature i minimalne i maksimalne izmjerene temperature
void obrada_podataka_task(void* pvParameters); 
void Primio_kanal_0(void* pvParameters); //prijem sa senzora 1
void Primio_kanal_1(void* pvParameters); //prijem sa senzora 2 
void Seg7_ispis_task(void* pvParameters); //ispisivanje trazenih informacija na 7-segmentni displej
void Serijska_stanje_task(void* pvParameters); //redovni ispis stanja sistema na serijsku (rezim rada, ukljuceno\iskljuceno, temperatura)

/* TIMER FUNCTIONS*/
static void RX_senzori_callback(TimerHandle_t Tmh); //svakih 200ms primi vrijednosti sa senzora i obradi ih
static void ispis_tajmer_callback(TimerHandle_t Tmh); //svakih 10s ispisuje stanje sistema

/* Funkcije deklaracija prije upotrebe */



/* Globalne promjenljive za generalnu upotrebu */
#define R_BUF_SIZE (32)

static uint8_t ukljuceno = 0;
uint8_t automatski;
double trenutna_temperatura = 0;

/* 7-SEG NUMBER DATABASE - ALL HEX DIGITS */
static const char hexnum[] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 
								0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71 };

/* GLOBAL OS-HANDLES */
SemaphoreHandle_t LED_INT_BinarySemaphore;
SemaphoreHandle_t TBE_BS_0, TBE_BS_1, TBE_BS_2;
SemaphoreHandle_t RXC_BS_0, RXC_BS_1, RXC_BS_2;
SemaphoreHandle_t RX_senzori_semafor;
SemaphoreHandle_t seg7_ispis;
SemaphoreHandle_t mutex_serijska;
SemaphoreHandle_t s1;
SemaphoreHandle_t serijska_stanje;

TimerHandle_t per_TimerHandle;
TimerHandle_t RX_senzori_timer; 
TimerHandle_t ispis_podaci_tajmer;


static QueueHandle_t seg7_queue = NULL;
static QueueHandle_t seg7_double_queue = NULL;
static QueueHandle_t seg7_auto_queue = NULL;
static QueueHandle_t serijska_ispis_queue = NULL;
static QueueHandle_t serijska_ispis_duzina = NULL;
static QueueHandle_t queue_senzor1 = NULL;
static QueueHandle_t queue_senzor2 = NULL;
static QueueHandle_t serijska_prijem_niz = NULL;
static QueueHandle_t serijska_prijem_duzina = NULL;
static QueueHandle_t stanje_formiranje = NULL;

/* Strukture za redove */
typedef struct seg7_podaci { //svi potrebni podaci za ispis na 7-segmentni displej 
	uint8_t min_max_indikator;
	uint8_t automatski;
	double trenutna_temperatura;
	double min;
	double max;
}seg7_podaci;


typedef struct podaci_za_stanje { //svi potrebni podaci za formiranje poruke za stanje
	double temperatura;
	uint8_t ukljuceno;
	uint8_t automatski;
}podaci_za_stanje;


typedef struct ispis_serijska_info { //svi potrebni podaci za ispis na serijsku
	uint8_t duzina_stringa;
	uint8_t poruka[60];
}ispis_serijska_info;


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

	if (get_TBE_status(0) != 0)
		xSemaphoreGiveFromISR(TBE_BS_0, &xHigherPTW);

	if (get_TBE_status(1) != 0)
		xSemaphoreGiveFromISR(TBE_BS_1, &xHigherPTW);

	if (get_TBE_status(2) != 0)
		xSemaphoreGiveFromISR(TBE_BS_2, &xHigherPTW);

	portYIELD_FROM_ISR(xHigherPTW);
}


/* RXC - RECEPTION COMPLETE - INTERRUPT HANDLER */
static uint32_t prvProcessRXCInterrupt(void)
{
	BaseType_t xHigherPTW = pdFALSE;

	if(get_RXC_status(0) != 0)
		xSemaphoreGiveFromISR(RXC_BS_0, &xHigherPTW);

	if (get_RXC_status(1) != 0)
		xSemaphoreGiveFromISR(RXC_BS_1, &xHigherPTW);

	if (get_RXC_status(2) != 0)
		xSemaphoreGiveFromISR(RXC_BS_2, &xHigherPTW);

	portYIELD_FROM_ISR(xHigherPTW);
}


/* PERIODIC TIMER CALLBACK */
static void TimerCallback(TimerHandle_t xTimer)
{ 
	xSemaphoreGive(seg7_ispis);
} //svakih 200ms osvjezavanje displeja  


static void ispis_tajmer_callback(TimerHandle_t ispis_podaci_tajmer) {
	xSemaphoreGive(serijska_stanje);
	
}


static void RX_senzori_callback(TimerHandle_t RX_senzori_timer) {
	xSemaphoreGive(RX_senzori_semafor);
}


/* MAIN - SYSTEM STARTUP POINT */
void main_demo( void )
{
	init_7seg_comm();
	init_LED_comm();
	init_serial_uplink(COM_CH);  // inicijalizacija serijske TX na kanalu 0
	init_serial_downlink(COM_CH);// inicijalizacija serijske TX na kanalu 0
	init_serial_uplink(COM_CH1);
	init_serial_downlink(COM_CH1);
	init_serial_uplink(COM_CH2);
	init_serial_downlink(COM_CH2);

	/* ON INPUT CHANGE INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_OIC, OnLED_ChangeInterrupt);

	/* Create LED interrapt semaphore */
	LED_INT_BinarySemaphore = xSemaphoreCreateBinary();

	/* create a timer task */
	per_TimerHandle = xTimerCreate("Timer", pdMS_TO_TICKS(80), pdTRUE, NULL, TimerCallback);
	xTimerStart(per_TimerHandle, 0);

	RX_senzori_timer = xTimerCreate("Timer1", pdMS_TO_TICKS(200), pdTRUE, NULL, RX_senzori_callback);
	xTimerStart(RX_senzori_timer, 0);

	ispis_podaci_tajmer = xTimerCreate("Timer2", pdMS_TO_TICKS(10000), pdTRUE, NULL, ispis_tajmer_callback);
	xTimerStart(ispis_podaci_tajmer, 0);

	/* SERIAL TRANSMITTER TASK */
	xTaskCreate(SerialSend_Task, "STx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);

	/* SERIAL RECEIVER TASK */
	xTaskCreate(SerialReceive_Task, "SRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAl_REC_PRI, NULL);

	/* Create TBE semaphore - serial transmit comm */
	TBE_BS_0 = xSemaphoreCreateBinary();
	TBE_BS_1 = xSemaphoreCreateBinary();
	TBE_BS_2 = xSemaphoreCreateBinary();



	/* Create RXC semaphore - serial transmit comm */
	RXC_BS_0 = xSemaphoreCreateBinary();
	RXC_BS_1 = xSemaphoreCreateBinary();
	RXC_BS_2 = xSemaphoreCreateBinary();

	/* Ostali semafori */
	RX_senzori_semafor = xSemaphoreCreateBinary();
	mutex_serijska = xSemaphoreCreateMutex();
	s1 = xSemaphoreCreateBinary();
	seg7_ispis = xSemaphoreCreateBinary();
	serijska_stanje = xSemaphoreCreateBinary();

	/* SERIAL TRANSMISSION INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_TBE, prvProcessTBEInterrupt);

	/* SERIAL RECEPTION INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);
	
	/* Kreiranje redova za komunikaciju izmedju taskova */

	seg7_queue = xQueueCreate(1, sizeof(uint8_t));// red za seg7 ispis
	seg7_double_queue = xQueueCreate(2, sizeof(double[3]));
	seg7_auto_queue = xQueueCreate(2, sizeof(uint8_t));


	//serijska_ispis_queue = xQueueCreate(3, sizeof(ispis_serijska_info)); //red za skladistenje strukture za ispis koja ne radi
	serijska_ispis_queue = xQueueCreate(3, sizeof(uint8_t [60])); //red za skladistenje poruke za ispis
	serijska_ispis_duzina = xQueueCreate(3, sizeof(uint8_t)); //red za skladistenje duzine rijeci

	serijska_prijem_niz = xQueueCreate(3, sizeof(uint8_t[12])); //red za skladistenje primljene rijeci (komande)
	serijska_prijem_duzina = xQueueCreate(3, sizeof(uint8_t)); //red za skladistenje duzine primljene rijeci

	queue_senzor1 = xQueueCreate(2, sizeof(double)); //red za primanje vrijednosti sa senzora 1
	queue_senzor2 = xQueueCreate(2, sizeof(double)); //red za primanje vrijednosti sa senzora 2

	stanje_formiranje = xQueueCreate(1, sizeof(uint8_t[3]));

	/* create a led bar TASK */
	xTaskCreate(led_bar_tsk, "ST",	configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);
	xTaskCreate(prijem_sa_senzora_tsk, "senzori", configMINIMAL_STACK_SIZE, NULL, SENZORI_PRI, NULL);
	xTaskCreate(obrada_podataka_task, "obrada", configMINIMAL_STACK_SIZE, NULL, OBRADA_TASK_PRI, NULL);
	xTaskCreate(Primio_kanal_0, "kanal0", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAl_REC_PRI, NULL);
	xTaskCreate(Primio_kanal_1, "kanal1", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAl_REC_PRI, NULL);
	xTaskCreate(Seg7_ispis_task, "Seg_7", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);
	xTaskCreate(Serijska_stanje_task, "Stanje", configMINIMAL_STACK_SIZE, NULL, OBRADA_TASK_PRI, NULL);

	vTaskStartScheduler();

	while (1);
}

void led_bar_tsk(void* pvParameters) //ocitati prekidace i reci da li je ukljuceno ili iskljuceno
{
	uint8_t ventilator = 0;
	uint8_t min_max_indikator = 0;
	uint8_t d;
	ispis_serijska_info* ispis_info;
	uint8_t duzina_niza_ispis = 0;
	uint8_t pomocni_niz[60] = { 0 };

		while (1)
	{  
		xSemaphoreTake(LED_INT_BinarySemaphore, portMAX_DELAY);
		get_LED_BAR(0, &d); //ocitaj stanje prvog stubca led bara
		
		if ((d & 0x01) != 0) { //provjeri da li je pritisnut prvi prekidac na led baru, ako jeste, ukljuci siste, ako nije, iskljucen sistem
			ukljuceno = 1;
			set_LED_BAR(1, 0x01);
		}
		else {
			ukljuceno = 0;
			set_LED_BAR(1, 0x00);
		}

		if (ukljuceno && !automatski) {//ako je manuelno i ukljuceno provjeri koji je ventilator aktivan pomocu maski (naredna 3 prekidaca)
			if ((d & 0x02) != 0) {
				ventilator = 1;

				xSemaphoreTake(mutex_serijska, portMAX_DELAY); //uzmi mutex
				//strcpy(ispis_info->poruka, "VENT:1");
				//ispis_info->duzina_stringa = sizeof("VENT:1") - 1;
				//xQueueSend(serijska_ispis_queue, &ispis_info, 0U);
				//printf("Ovo je duzina %u\n", ispis_info->duzina_stringa);
				//printf("Ovo je duzina %u\n A ovo je poruka %s\n", (ispis_info->duzina_stringa), ispis_info->poruka);
				strcpy(pomocni_niz, "VENT:1"); //formiraj zeljeni ispis
				duzina_niza_ispis = sizeof("VENT:1") - 1; //kazi kolika je duzina onoga sto treba ispisati 
				xQueueSend(serijska_ispis_queue, &pomocni_niz, 0U); //posalji preko reda tasku za ispis
				xQueueSend(serijska_ispis_duzina, &duzina_niza_ispis, 0U); //posalji preko reda tasku za ispis
				send_serial_character(COM_CH2, 13); //zapocni ispis
				xSemaphoreTake(s1, portMAX_DELAY); //sacekaj da se zavrsi ispis
				xSemaphoreGive(mutex_serijska); //vrati mutex da mogu i drugi da ispisuju
				
			}

			else if ((d & 0x04) != 0) {
				ventilator = 2;
				xSemaphoreTake(mutex_serijska, portMAX_DELAY);
				strcpy(pomocni_niz, "VENT:2");
				duzina_niza_ispis = sizeof("VENT:2") - 1;
				xQueueSend(serijska_ispis_queue, &pomocni_niz, 0U);
				xQueueSend(serijska_ispis_duzina, &duzina_niza_ispis, 0U);
				send_serial_character(COM_CH2, 13);
				xSemaphoreTake(s1, portMAX_DELAY);
				xSemaphoreGive(mutex_serijska);
			}

			else if ((d & 0x08) != 0) {
				ventilator = 3;
				xSemaphoreTake(mutex_serijska, portMAX_DELAY);
				strcpy(pomocni_niz, "VENT:3");
				duzina_niza_ispis = sizeof("VENT:3") - 1;
				xQueueSend(serijska_ispis_queue, &pomocni_niz, 0U);
				xQueueSend(serijska_ispis_duzina, &duzina_niza_ispis, 0U);
				send_serial_character(COM_CH2, 13);
				xSemaphoreTake(s1, portMAX_DELAY);
				xSemaphoreGive(mutex_serijska);
			}
			else {
				ventilator = 0;
			}

			printf("ventilator: %u\n", (unsigned)ventilator);
		}

		if ((d & 0x80) != 0) { //ocitaj posljednji prekidac na prvom stubcu da vidis da li treba min ili max prikazati na 7-segmentnom displeju
			min_max_indikator = 1;
		}
		else {
			min_max_indikator = 0;
		}
		xQueueSend(seg7_queue, &min_max_indikator, 0U); //posalji informaciju preko reda tasku kom treba
	}
}

void SerialSend_Task(void* pvParameters)
{
	uint8_t t_point = 0;
	uint8_t r[60];
	uint8_t duzina_niza_ispis = 0;

	while (1)
	{
		xSemaphoreTake(TBE_BS_2, portMAX_DELAY);// sacekaj da TX registar bude prazan 
		xQueueReceive(serijska_ispis_queue, &r, pdMS_TO_TICKS(10)); //pogledaj da li ima novih vrijednosti (ako nema za 10ms radi dalje)
		xQueueReceive(serijska_ispis_duzina, &duzina_niza_ispis, pdMS_TO_TICKS(10)); //pogledaj ima li sta novo (ako nema za 10ms radi dalje)

		if (t_point < duzina_niza_ispis) { //dok nije ispisan posljednji karakter salji slovo po slovo 
			send_serial_character(COM_CH2, r[t_point++]);
		}
		else { //kada se ispise posljednji karakter, onda resetuj sistem i daj semafor da je ispis rijeci zavrsen
			t_point = 0;
			xSemaphoreGive(s1);
			duzina_niza_ispis = 0;
		}
	}
}




void prijem_sa_senzora_tsk(void* pvParameters)
{
	double senzor1 = 0, senzor2 = 0;
	double slanje[3] = { 0 };
	double min = 99, max = 0;
	while (1) {
		xQueueReceive(queue_senzor1, &senzor1, pdMS_TO_TICKS(100)); //ocitaj senzor jedan
		xQueueReceive(queue_senzor2, &senzor2, pdMS_TO_TICKS(100)); //ocitaj senzor 2, ako nema novih vrijednosti, radi sa starim

		trenutna_temperatura = (senzor1 + senzor2) / 2; //racunanje srednje vrijednosti 
			if (trenutna_temperatura > max) { //odredjivanje maksimuma i minimuma
				max = trenutna_temperatura;
			}
		if (trenutna_temperatura < min && senzor1 != 0 && senzor2 != 0) { //minimum ne smije inicijalno biti 0, pa to moramo sprijeciti
			min = trenutna_temperatura;
		}
		slanje[0] = min;
		slanje[1] = max;
		slanje[2] = trenutna_temperatura;

		xQueueSend(seg7_double_queue, &slanje, 0U);
	}
}


void Primio_kanal_0(void* pvParameters) //prijem sa kanala 0 (senzor 1), kada stigne karakter za kraj poruke, onda ono ispred pretvori u float
{ //i posalji preko reda tasku za ocitavanje senzora
	double senzor1 = 0;
	uint8_t cc = 0;
	uint8_t br_karaktera = 0;
	uint8_t temp_kanal0[6] = { 0 };

	while (1) {
		xSemaphoreTake(RXC_BS_0, portMAX_DELAY);
		get_serial_character(COM_CH, &cc);
		//printf("primio kanal 0 %u\n", (unsigned)cc);
		if (cc == 0x0d) {
			senzor1 = atof(temp_kanal0);
			br_karaktera = 0;
			xQueueSend(queue_senzor1, &senzor1, 0U);
		}
		else {
			temp_kanal0[br_karaktera++] = cc;
		}
	}
}

void Primio_kanal_1(void* pvParameters) //isti task kao primio_kanal_0 samo je ovo sa kanala 1 sto simulira drugi senzor
{
	double senzor2 = 0;
	uint8_t cc = 0;
	uint8_t br_karaktera = 0;
	uint8_t temp_kanal1[6] = { 0 };

	while (1) {
		xSemaphoreTake(RXC_BS_1, portMAX_DELAY);
		get_serial_character(COM_CH1, &cc);

		if (cc == 0x0d) {
			senzor2 = atof(temp_kanal1);
			xQueueSend(queue_senzor2, &senzor2, 0U);
			br_karaktera = 0;
		}
		else {
			temp_kanal1[br_karaktera++] = cc;
		}

	}
}

void SerialReceive_Task(void* pvParameters) //prima komandnu rijec koja se zavrsava karakterom 13(0x0d) i prosljedjuje je tasku obrada_podataka
{
	uint8_t r_point = 0;
	uint8_t r_buffer[12];
	uint8_t cc = 0;
	uint8_t duzina_primljene_rijeci = 0;

	while (1)
	{
		xSemaphoreTake(RXC_BS_2, portMAX_DELAY);// ceka na serijski prijemni interapt
		get_serial_character(COM_CH2, &cc);//ucitava primljeni karakter u promenjivu cc


		if (cc == 0x0d) // oznaciti kraj poruke i ako je kraj, preko reda poslati informacije o poruci i restartovati ovaj taks
		{
			duzina_primljene_rijeci = r_point; 
			xQueueSend(serijska_prijem_niz, &r_buffer, 0U);
			xQueueSend(serijska_prijem_duzina, &r_point, 0U);
			r_point = 0;
		}
		else if (r_point < R_BUF_SIZE)// pamti karaktere prije FF
		{
			r_buffer[r_point++] = cc;
		}
	}
}


void obrada_podataka_task(void* pvParameters)
{
	podaci_za_stanje* stanje;
	uint8_t r_buffer[12] = { 0 };
	uint8_t duzina_primljene_rijeci = 0;

	uint8_t pomocni_niz[60] = { 0 }; 
	uint8_t duzina_niza_ispis = 0;

	uint8_t zeljena_temp = 0;
	double histerezis = 0.1;

	uint8_t podaci_za_stanje[3] = { 0 };

	while (1)
	{

		xQueueReceive(serijska_prijem_duzina, &duzina_primljene_rijeci, pdMS_TO_TICKS(20)); //primi komandnu poruku
		xQueueReceive(serijska_prijem_niz, &r_buffer, pdMS_TO_TICKS(20)); //primi duzinu komandne poruke

		/* ispitujemo sta je stiglo, AUTO, MANU, zeljena_temp ili histerezis i ono sto treba ispisujemo na serijsku */

		if ((duzina_primljene_rijeci == sizeof("AUTOMATSKI") - 1) && (strncmp(r_buffer, ("AUTOMATSKI"), duzina_primljene_rijeci) == 0)) {
			printf("Dobro uneseno automatski \n");

			xSemaphoreTake(mutex_serijska, portMAX_DELAY);
			strcpy(pomocni_niz, "OK AUTOMATSKI");
			duzina_niza_ispis = sizeof("OK AUTOMATSKI") - 1;
			xQueueSend(serijska_ispis_queue, &pomocni_niz, 0U);
			xQueueSend(serijska_ispis_duzina, &duzina_niza_ispis, 0U);
			send_serial_character(COM_CH2, 13);
			xSemaphoreTake(s1, portMAX_DELAY);
			xSemaphoreGive(mutex_serijska);

			automatski = 1;
		}

		else if ((duzina_primljene_rijeci == sizeof("MANUELNO") - 1) && (strncmp(r_buffer, ("MANUELNO"), duzina_primljene_rijeci) == 0))
		{
			automatski = 0;
			printf("Dobro uneseno manuelno\n");

			xSemaphoreTake(mutex_serijska, portMAX_DELAY);
			strcpy(pomocni_niz, "OK MANUELNO");
			duzina_niza_ispis = sizeof("OK MANUELNO") - 1;
			xQueueSend(serijska_ispis_queue, &pomocni_niz, 0U);
			xQueueSend(serijska_ispis_duzina, &duzina_niza_ispis, 0U);
			send_serial_character(COM_CH2, 13);
			xSemaphoreTake(s1, portMAX_DELAY);
			xSemaphoreGive(mutex_serijska);
		}
		else if (duzina_primljene_rijeci == 3) //radi i ovo
		{ 
			if (atof(r_buffer) < 5 && atof(r_buffer) > 0) 
			{
				histerezis = atof(r_buffer);
			}
			else if (atof(r_buffer) > 16 && atof(r_buffer) < 30) 
			{
				zeljena_temp = (uint8_t)atof(r_buffer);
			}
			printf("Histerezis: %f , zeljena temperatura: %u\n", histerezis, zeljena_temp);
		}
		duzina_primljene_rijeci = 0;

		/* ukljucujemo ili iskljucujemo signalne lampice na led baru u zavisnosti da li je ukljucena ili iskljucena klima i da li je ventilator
		aktivan ili je neaktivan (ukljuceno drugi stubac prva lampica, ventilator treci stubac prva lampica)*/
		/*ako je ukljuceno i ako je automatski, onda imamo automatsko upravljanje kao na PLC-ovima sto smo radili, a manuelno uvijek duva*/

		if (ukljuceno) {
			if (automatski == 1) {
				if (trenutna_temperatura > (double)zeljena_temp + histerezis) {
					set_LED_BAR(2, 0x01);
				}
				else {
					set_LED_BAR(2, 0x00);
				}
			}
			else {
				set_LED_BAR(2, 0x01);
			}
		}
		else {
			set_LED_BAR(2, 0x00);
		}

		xQueueSend(seg7_auto_queue, &automatski, 0U);
		podaci_za_stanje[0] = (uint8_t)trenutna_temperatura;
		podaci_za_stanje[1] = ukljuceno;
		podaci_za_stanje[2] = automatski;
		xQueueSend(stanje_formiranje, &podaci_za_stanje, 0U);

		/*stanje->automatski = automatski;
		stanje->temperatura = trenutna_temperatura;
		stanje->ukljuceno = ukljuceno;
		*/

		//vTaskDelay(pdMS_TO_TICKS(20));
	}
}

void Seg7_ispis_task(void* pvParameters) {
	uint8_t min_max_indikator=0;
	uint8_t automatski = 0;
	double min, max, trenutna_temperatura;
	double prijem[3] = { 0 };

	while (1) {
		xSemaphoreTake(seg7_ispis, portMAX_DELAY);
		xQueueReceive(seg7_queue, &min_max_indikator, pdMS_TO_TICKS(20));
		xQueueReceive(seg7_auto_queue, &automatski, pdMS_TO_TICKS(20));
		xQueueReceive(seg7_double_queue, &prijem, pdMS_TO_TICKS(20));

		min = prijem[0];
		max = prijem[1];
		trenutna_temperatura = prijem[2];

		if (automatski) { //na prvu cifru ispisuje 1 ako je rezim rada automatski, a 0 ako je manuelno
			select_7seg_digit(0);
			set_7seg_digit(hexnum[1]);
		}
		else {
			select_7seg_digit(0);
			set_7seg_digit(hexnum[0]);
		}

		select_7seg_digit(1); //na drugu i trecu cifru ispisujemo trenutnu temperaturu 
		set_7seg_digit(hexnum[(uint8_t)trenutna_temperatura / 10]);
		select_7seg_digit(2);
		set_7seg_digit(hexnum[(uint8_t)trenutna_temperatura % 10]);

		if (min_max_indikator == 0) { //u zavisnosti sta je oznaceno da se prikaze na 4. i 5. cifru ispisujemo minimum ili maksimum
			select_7seg_digit(3);
			set_7seg_digit(hexnum[(uint8_t)min / 10]);
			select_7seg_digit(4);
			set_7seg_digit(hexnum[(uint8_t)min % 10]);
		}
		else {
			select_7seg_digit(3);
			set_7seg_digit(hexnum[(uint8_t)max / 10]);
			select_7seg_digit(4);
			set_7seg_digit(hexnum[(uint8_t)max % 10]);
		}
	}
}


void Serijska_stanje_task(void* pvParameters) { /*formiramo niz za redovan ispis stanja sistema i saljemo pomocu reda poruku i duzinu poruke
												tasku za ispis na serijsku*/
	uint8_t pomocni_niz[60] = { 0 };
	uint8_t duzina_niza_ispis = 0;
	//uint8_t stanje[3] = { 0 };
	//uint8_t trenutna_temperatura = 0, automatski = 0, ukljuceno = 0;

	while (1) {
		xSemaphoreTake(serijska_stanje, portMAX_DELAY);
		//xQueueReceive(stanje_formiranje, &stanje, pdMS_TO_TICKS(20));

		//trenutna_temperatura = stanje[0];
		//ukljuceno = stanje[1];
		//automatski = stanje[2];

		xSemaphoreTake(mutex_serijska, portMAX_DELAY);

		strcpy(pomocni_niz, "Stanje: ");
		duzina_niza_ispis = sizeof("Stanje: ") - 1;

		if (automatski) {
			strcat(pomocni_niz, "AUTOMATSKI");
			duzina_niza_ispis += sizeof("AUTOMATSKI") - 1;
		}
		else {
			strcat(pomocni_niz, "MANUELNO");
			duzina_niza_ispis += sizeof("MANUELNO") - 1;
		}

		strcat(pomocni_niz, ", radi: ");
		duzina_niza_ispis += sizeof(", radi: ") - 1;

		if (ukljuceno) {
			strcat(pomocni_niz, "UKLJUCENO");
			duzina_niza_ispis += sizeof("UKLJUCENO") - 1;
		}
		else {
			strcat(pomocni_niz, "ISKLJUCENO");
			duzina_niza_ispis += sizeof("ISKLJUCENO");
		}

		strcat(pomocni_niz, ", temp:");
		duzina_niza_ispis += sizeof(", temp:") - 1;
		pomocni_niz[duzina_niza_ispis++] = (unsigned)trenutna_temperatura / 10 + '0';
		pomocni_niz[duzina_niza_ispis++] = (unsigned)trenutna_temperatura % 10 + '0';
		xQueueSend(serijska_ispis_queue, &pomocni_niz, 0U);
		xQueueSend(serijska_ispis_duzina, &duzina_niza_ispis, 0U);
		send_serial_character(COM_CH2, 13);
		xSemaphoreTake(s1, portMAX_DELAY);
		xSemaphoreGive(mutex_serijska);
	}
}