#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* BOARD CONFIG                                                         */
/************************************************************************/

// Configuracoes dos LEDS
#define LED1_PIO           PIOA
#define LED1_PIO_ID        ID_PIOA
#define LED1_PIO_IDX       0
#define LED1_PIO_IDX_MASK  (1 << LED1_PIO_IDX)

#define LED2_PIO           PIOC
#define LED2_PIO_ID        ID_PIOC
#define LED2_PIO_IDX       30
#define LED2_PIO_IDX_MASK  (1 << LED2_PIO_IDX)

#define LED3_PIO           PIOB
#define LED3_PIO_ID        ID_PIOB
#define LED3_PIO_IDX       2
#define LED3_PIO_IDX_MASK  (1 << LED3_PIO_IDX)

// Configuracoes dos botoes
#define BUT1_PIO		  PIOD
#define BUT1_PIO_ID		  ID_PIOD
#define BUT1_PIO_IDX	  28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX)

/************************************************************************/
/* RTOS                                                                */
/************************************************************************/

#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* prototypes local                                                     */
/************************************************************************/

void but1_callback(void);
static void io_init(void);
volatile int lastTcFlag = 0;

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t second;
} calendar;

/************************************************************************/
/* recursos RTOS                                                        */
/************************************************************************/

/** Semaphores */
// SemaphoreHandle_t xSemaphoreAlarm;
SemaphoreHandle_t xSemaphoreOLED;

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}
extern void vApplicationIdleHook(void) { }
extern void vApplicationTickHook(void) { }
extern void vApplicationMallocFailedHook(void) { configASSERT( ( volatile void * ) NULL ); }


void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	pmc_enable_periph_clk(ID_TC);
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);
  	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {
  uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
  rtt_sel_source(RTT, false);
  rtt_init(RTT, pllPreScale);
  if (rttIRQSource & RTT_MR_ALMIEN) {
	uint32_t ul_previous_time;
  	ul_previous_time = rtt_read_timer_value(RTT);
  	while (ul_previous_time == rtt_read_timer_value(RTT));
  	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
  }
  NVIC_DisableIRQ(RTT_IRQn);
  NVIC_ClearPendingIRQ(RTT_IRQn);
  NVIC_SetPriority(RTT_IRQn, 4);
  NVIC_EnableIRQ(RTT_IRQn);
  if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
  else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
}

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	pmc_enable_periph_clk(ID_RTC);
	rtc_set_hour_mode(rtc, 0);
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);
	rtc_enable_interrupt(rtc,  irq_type);
}

void pin_toggle(Pio *pio, uint32_t mask) {
  if(pio_get_output_data_status(pio, mask))
    pio_clear(pio, mask);
  else
    pio_set(pio, mask);
}

// void pisca_led(){
// 	pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
// 	delay_ms(500);
// 	pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
// 	delay_ms(500);
// }

/************************************************************************/
/* Handlers / callbacks					                                */
/************************************************************************/

void TC3_Handler(void) {
	volatile uint32_t status = tc_get_status(TC1, 0);
	pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);  
}

void TC6_Handler(void) {
	volatile uint32_t status = tc_get_status(TC2, 0);
	pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);  
}

void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);
		RTT_init(4, 16, RTT_MR_ALMIEN);
	}  
}

void RTC_Handler(void) {
    uint32_t ul_status = rtc_get_status(RTC);
    if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {	
		xSemaphoreGiveFromISR(xSemaphoreOLED, 0);
    }
    if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		lastTcFlag = 1;
		// pisca_led();
    }
    rtc_clear_status(RTC, RTC_SCCR_SECCLR);
    rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
    rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
    rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
    rtc_clear_status(RTC, RTC_SCCR_CALCLR);
    rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

void but1_callback(void) {                                                                          
    uint32_t current_hour, current_min, current_sec;
    uint32_t current_year, current_month, current_day, current_week;
    rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
    rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);                                                              
    rtc_set_date_alarm(RTC, 1, current_month, 1, current_day);                              
    rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min, 1, current_sec + 20);
}

/************************************************************************/
/* TASK									                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	io_init();
	gfx_mono_ssd1306_init();
	char str[25];
	calendar rtc_initial = {2018, 3, 19, 12, 8, 55, 1};

	TC_init(TC1, ID_TC3, 0, 4);
  	tc_start(TC1, 0);

	RTT_init(4, 16, RTT_MR_ALMIEN);

	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_SECEN);
	uint32_t current_hour, current_min, current_sec;
	uint32_t current_year, current_month, current_day, current_week;
	rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
	rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);

	for (;;)  {
		if (xSemaphoreTake(xSemaphoreOLED, 500 / portTICK_PERIOD_MS) == pdTRUE){
			rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
			rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
			sprintf(str, "%02d:%02d:%02d", current_hour, current_min, current_sec);
			gfx_mono_draw_string(str, 0, 0, &sysfont);

		}
		if (lastTcFlag == 1){
			TC_init(TC2, ID_TC6, 0, 2);
  			tc_start(TC2, 0);
			lastTcFlag = 0;
		}
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}

/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};
	stdio_serial_init(CONF_UART, &uart_serial_options);
	setbuf(stdout, NULL);
}

static void io_init(void) {
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 1, 0, 0);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 1, 0, 0);
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 1, 0, 0);

  	pmc_enable_periph_clk(BUT1_PIO_ID);
  	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
  	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID,BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but1_callback);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4);
}

/************************************************************************/
/* Main			                                                        */
/************************************************************************/

int main(void) {
	sysclk_init();
	board_init();
	configure_console();

	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}
	xSemaphoreOLED = xSemaphoreCreateBinary();
	if (xSemaphoreOLED == NULL)
		printf("falha em criar o semaforo \n");

	vTaskStartScheduler();

	while(1){}

	return 0;
}