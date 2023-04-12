#include "conf_board.h"
#include <asf.h>
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* BOARD CONFIG                                                         */
/************************************************************************/

// Configuracao do ECHO
#define ECHO			  PIOD
#define ECHO_ID			  ID_PIOD
#define ECHO_PIN		  27
#define ECHO_PIN_MASK	  (1 << ECHO_PIN)

// Configuracao do TRIG
#define TRIG			  PIOD
#define TRIG_ID			  ID_PIOD
#define TRIG_PIN		  20u
#define TRIG_PIN_MASK	  (1u << TRIG_PIN)

/************************************************************************/
/* RTOS                                                                */
/************************************************************************/

#define TASK_STACK_SIZE (6144 / sizeof(portSTACK_TYPE))
#define TASK_STACK_PRIORITY (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* recursos RTOS                                                        */
/************************************************************************/

/** Queues */
QueueHandle_t xQueueDeltaTempo;
QueueHandle_t xQueueDistancia;

/************************************************************************/
/* prototypes local                                                     */
/************************************************************************/

void callback_echo(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
  printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
  for (;;) { }
}
extern void vApplicationIdleHook(void) { pmc_sleep(SAM_PM_SMODE_SLEEP_WFI); }
extern void vApplicationTickHook(void) {}
extern void vApplicationMallocFailedHook(void) { configASSERT((volatile void *)NULL); }

/************************************************************************/
/* callbacks			                                                */
/************************************************************************/

void callback_echo(void){
	if(pio_get(ECHO, PIO_INPUT, ECHO_PIN_MASK)){ rtt_init(RTT, 1); }
		else {
		uint32_t deltaTempo = rtt_read_timer_value(RTT);
		xQueueSendFromISR(xQueueDeltaTempo, &deltaTempo, 1000);
	}
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
  int distancia;
  char msg[128]; 
  for (;;) {
    if (xQueueReceive(xQueueDistancia, &distancia, 10)) {
      if (distancia < 400 && distancia > 2){
		  if (distancia > 99){
			  sprintf(msg, "dist = %d cm", distancia);
			  gfx_mono_draw_string(msg, 0, 5, &sysfont);
		  }
		  else if (distancia > 9){
			  sprintf(msg, "dist. = %d cm", distancia);
			  gfx_mono_draw_string(msg, 0, 5, &sysfont);
		  } else {
			  sprintf(msg, "dist. = %d  cm", distancia);
			  gfx_mono_draw_string(msg, 0, 5, &sysfont);
		  }
		} else{ gfx_mono_draw_string("Fora do range", 0, 5, &sysfont); }
    }
  }
}

static void task_distancia(void *pvParameters) {
  uint32_t deltaTempo;
	for (;;)  {
		pio_set(TRIG, TRIG_PIN_MASK);
		delay_us(10);
		pio_clear(TRIG, TRIG_PIN_MASK);
		if(xQueueReceive(xQueueDeltaTempo, &deltaTempo, 1000)){
			int distancia = (17000 * deltaTempo / 32768);
			xQueueSend(xQueueDistancia, &distancia, 10);
			printf("%d cm\n", distancia);
		} else {
			printf("erro na leitura \n");
		}
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

/************************************************************************/
/* funcoes                                                              */
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

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
  sysclk_init();
  board_init();
  gfx_mono_ssd1306_init();
  configure_console();
  
  pmc_enable_periph_clk(TRIG_ID);
  pmc_enable_periph_clk(ECHO_ID);
  pio_configure(ECHO, PIO_INPUT, ECHO_PIN_MASK, 0);
  pio_configure(TRIG, PIO_OUTPUT_0, TRIG_PIN_MASK, PIO_DEFAULT);
  pio_handler_set(ECHO, ECHO_ID, ECHO_PIN_MASK, PIO_IT_EDGE, callback_echo);
  pio_enable_interrupt(ECHO, ECHO_PIN_MASK);
  NVIC_EnableIRQ(ECHO_ID);
  NVIC_SetPriority(ECHO_ID, 4);

  xQueueDistancia = xQueueCreate(32, sizeof(uint32_t));
  xQueueDeltaTempo = xQueueCreate(32, sizeof(uint32_t));

  if (xTaskCreate(task_oled, "oled", TASK_STACK_SIZE, NULL, TASK_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Nao criou task oled\r\n");
  }
  if (xTaskCreate(task_distancia, "distancia", TASK_STACK_SIZE, NULL, TASK_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Nao criou task distancia\r\n");
  }

  vTaskStartScheduler();
  while (1) {
  }

  return 0;
}
