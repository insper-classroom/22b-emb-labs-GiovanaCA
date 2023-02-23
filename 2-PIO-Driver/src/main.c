/**
 * 5 semestre - Eng. da Computação - Insper
 * Rafael Corsi - rafael.corsi@insper.edu.br
 *
 * Projeto 0 para a placa SAME70-XPLD
 *
 * Objetivo :
 *  - Introduzir ASF e HAL
 *  - Configuracao de clock
 *  - Configuracao pino In/Out
 *
 * Material :
 *  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
 */

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"

/************************************************************************/
/* defines                                                              */
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

#define BUT2_PIO		  PIOC
#define BUT2_PIO_ID		  ID_PIOC
#define BUT2_PIO_IDX	  31
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX)

#define BUT3_PIO		  PIOA
#define BUT3_PIO_ID		  ID_PIOA
#define BUT3_PIO_IDX	  19
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX)

/*  Default pin configuration (no attribute). */
#define _PIO_DEFAULT             (0u << 0)
/*  The internal pin pull-up is active. */
#define _PIO_PULLUP              (1u << 0)
/*  The internal glitch filter is active. */
#define _PIO_DEGLITCH            (1u << 1)
/*  The internal debouncing filter is active. */
#define _PIO_DEBOUNCE            (1u << 3)


/************************************************************************/
/* Funcoes LAB 2                                                              */
/************************************************************************/

// Funções
void _pio_set(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_SODR = ul_mask;
}

void _pio_clear(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_CODR = ul_mask;
}

void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask,
        const uint32_t ul_pull_up_enable)
{
	if (ul_pull_up_enable == 1) { p_pio->PIO_PUER = ul_mask; }
	else { p_pio->PIO_PUDR = ul_mask; }
}

void _pio_set_input(Pio *p_pio, const uint32_t ul_mask,
		const uint32_t ul_attribute)
{
	if (ul_attribute & _PIO_PULLUP){ _pio_pull_up(p_pio, ul_mask, 1); }
	p_pio->PIO_PER = ul_mask;
	p_pio->PIO_ODR = ul_mask;
	if (ul_attribute & _PIO_DEBOUNCE){
		p_pio->PIO_IFER = ul_mask;
		p_pio->PIO_IFSCDR = ul_mask;
	}
}

void _pio_set_output(Pio *p_pio, const uint32_t ul_mask,
		const uint32_t ul_default_level,
		const uint32_t ul_multidrive_enable,
		const uint32_t ul_pull_up_enable)
{
	p_pio->PIO_PER = ul_mask;
	p_pio->PIO_OER = ul_mask;
	if (ul_default_level == 1) { _pio_set(p_pio, ul_mask); }
	else { _pio_clear(p_pio, ul_mask); }
	if (ul_multidrive_enable == 1) { p_pio->PIO_MDER = ul_mask; }
	if (ul_pull_up_enable == 1) { _pio_pull_up(p_pio, ul_mask, 1); }
	else { _pio_pull_up(p_pio, ul_mask, 0); }
}

uint32_t _pio_get(Pio *p_pio, const pio_type_t ul_type,
		const uint32_t ul_mask)
{
	uint32_t valor = 0;
	if (ul_type == PIO_INPUT) { valor = p_pio->PIO_PDSR & ul_mask; } 
	if (ul_type == PIO_OUTPUT_0) {		
		if (p_pio->PIO_ODSR & ul_mask) { valor = 1; }
		else { valor = 0; }
	}
	return valor;
}

void _delay_ms(int tms){
	for(int i = 0; i < 200000*tms; i++){ __asm__("nop"); }
}

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

// Função de inicialização do uC
void init(void)
{
	// Initialize the board clock
	sysclk_init();
	
	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	// Ativa o PIO na qual o LED foi conectado
	// para que possamos controlar o LED.
	pmc_enable_periph_clk(LED1_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);
	
	// Inicializa PIO do botao
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	
	// configura pino ligado ao botao como entrada com um pull-up.
	_pio_set_input(BUT1_PIO, BUT1_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_set_input(BUT2_PIO, BUT2_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_set_input(BUT3_PIO, BUT3_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	
	// Inicializa PC8 como saida
	_pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 0, 0, 0);
	_pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 0, 0, 0);
	_pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 0, 0, 0);
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void)
{
  init();

  // super loop
  // aplicacoes embarcadas não devem sair do while(1).
  while (1)
  {
	  // BOTAO 1 E LED 1
	  if(!_pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)) {
		  for (int i = 0; i < 5; i++) {
			  _pio_clear(LED1_PIO, LED1_PIO_IDX_MASK); 
			  _delay_ms(100);						
			  _pio_set(LED1_PIO, LED1_PIO_IDX_MASK);	
			  _delay_ms(100);	 				
		  }
	  } else {
		  _pio_set(LED1_PIO, LED1_PIO_IDX_MASK); 
	  }
	  
	  // BOTAO 2 E LED 2
	  if(!_pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK)) {
		  for (int i = 0; i < 5; i++) {
			  _pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
			  _delay_ms(100);						
			  _pio_set(LED2_PIO, LED2_PIO_IDX_MASK);	
			  _delay_ms(100);	 			
		  }
	  } else {
		  _pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
	  }
	  
	  // BOTAO 3 E LED 3
	  if(!_pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK)) {
		  for (int i = 0; i < 5; i++) {
			  _pio_clear(LED3_PIO, LED3_PIO_IDX_MASK); 
			  _delay_ms(100);					
			  _pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
			  _delay_ms(100);	 					
		  }
	  } else {
		  _pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
	  }
  }
  return 0;
}
