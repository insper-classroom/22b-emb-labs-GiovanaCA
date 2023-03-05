#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// Configuracoes do LED
#define LED1_PIO           PIOA
#define LED1_PIO_ID        ID_PIOA
#define LED1_PIO_IDX       0
#define LED1_PIO_IDX_MASK  (1 << LED1_PIO_IDX)

// Configuracoes dos botoes
#define BUT1_PIO		  PIOD
#define BUT1_PIO_ID		  ID_PIOD
#define BUT1_PIO_IDX	  28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX)

#define BUT2_PIO		  PIOC
#define BUT2_PIO_ID		  ID_PIOC
#define BUT2_PIO_IDX	  31
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX)

#define BUT3_PIO		  PIOA
#define BUT3_PIO_ID		  ID_PIOA
#define BUT3_PIO_IDX	  19
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX)

// Flag
volatile char but_flag;
volatile char but_flag2;
volatile char but_flag3;

// Delay
int delay = 500;

void but_callback(void) { but_flag = 1; }
void but_callback2(void) { but_flag2 = 1; }
void but_callback3(void) { but_flag3 = 1; }

// pisca led N vez no periodo T
void pisca_led(int n, int t){
	double k = 10;
	double j = 10;
	char freq[128];
	for (int i=0;i<n;i++){
		pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
		delay_ms(t);
		pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
		delay_ms(t);
		if (but_flag2 == 1) {
			but_flag2 = 0;
			n = i;
			j = k + 1;
		}
		if (but_flag3 == 1) {
			but_flag3 = 0;
			delay_ms(300);
			if (pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK)) {
				t += 100;
				delay += 100;
				double frequencia = (double) 1000 / (2 * t);
				if (delay > 1000) {
					delay = 1000;
					t = 1000;
				}
			}
		}
		for(j; j<=k;j+=1){
			gfx_mono_draw_rect(j, 10, 2, 5, GFX_PIXEL_SET);
			delay_ms(10);
		}
		k += (double) 110/n;
		double frequencia = (double) 1000 / (2 * delay);
		sprintf(freq, "%f Hz", frequencia);
		gfx_mono_draw_string(freq, 5,18, &sysfont);
	}
}

void io_init(void)
{
	// Configura led
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEFAULT);

	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);

	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 60);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but_callback);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but_callback2);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but_callback3);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	pio_get_interrupt_status(BUT2_PIO);
	pio_get_interrupt_status(BUT3_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4
	NVIC_SetPriority(BUT2_PIO_ID, 3); // Prioridade 4
	NVIC_SetPriority(BUT3_PIO_ID, 3); // Prioridade 4
}

int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	
	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	// configura botao com interrupcao
	io_init();
	
    // Init OLED
	gfx_mono_ssd1306_init();
  
	// gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);

	// Calculo da frequencia
	char freq[128];
	double frequencia = (double) 1000 / (2 * delay);

  /* Insert application code here, after the board has been initialized. */
	while(1) {
		
		if (but_flag == 1) {
			but_flag = 0;
			delay_ms(500);
			if (pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)) {
	    		delay -= 100;
				if (delay < 100) { delay = 100; }
			} else {
				delay += 100;
				if (delay > 1000) { delay = 1000; }
			}
			pisca_led(30, delay);
			frequencia = (double) 1000 / (2 * delay);
			sprintf(freq, "%f Hz", frequencia);
			gfx_mono_draw_string(freq, 5,18, &sysfont);
			for(int i=120;i>=10;i-=1){
				gfx_mono_draw_rect(i, 10, 2, 5, GFX_PIXEL_CLR);
				delay_ms(10);
			}
		}
		
		// Entra em sleep mode
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
