/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include <asf.h>
#include <string.h>
#include "ili9341.h"
#include "lvgl.h"
#include "touch/touch.h"

LV_FONT_DECLARE(dseg70);
LV_FONT_DECLARE(dseg40);
LV_FONT_DECLARE(dseg25);
LV_FONT_DECLARE(dseg15);
LV_FONT_DECLARE(dseg10);
LV_FONT_DECLARE(monts15);
LV_FONT_DECLARE(monts10);
LV_IMG_DECLARE(clock);
LV_IMG_DECLARE(fumaca);
/************************************************************************/
/* LCD / LVGL                                                           */
/************************************************************************/

#define LV_HOR_RES_MAX          (320)
#define LV_VER_RES_MAX          (240)

static lv_disp_draw_buf_t disp_buf;
static lv_color_t buf_1[LV_HOR_RES_MAX * LV_VER_RES_MAX];
static lv_disp_drv_t disp_drv;
static lv_indev_drv_t indev_drv;

static  lv_obj_t * labelBtn1;
static  lv_obj_t * labelBtn2;
static  lv_obj_t * labelBtn3;
static  lv_obj_t * labelBtn4;
static  lv_obj_t * labelBtn5;
static  lv_obj_t * labelBtn6;
lv_obj_t * labelFloor;
lv_obj_t * labelFloorDigit;
lv_obj_t * labelSetValue;
lv_obj_t * labelClock;
lv_obj_t * labelChave;
lv_obj_t * labelHome;
lv_obj_t * labelCelsius;
lv_obj_t * labelCelsius2;
lv_obj_t * labelConfig;
lv_obj_t * labelSet;
lv_obj_t * labelClock2;
lv_obj_t * labelFumaca;
lv_obj_t * labelDia;
lv_obj_t * labelFT;
lv_obj_t * labelFT2;
lv_obj_t * scr1;
lv_obj_t * scr2;

volatile char setPower;

SemaphoreHandle_t xMutexLVGL;

typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t second;
} calendar;

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_LCD_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}
extern void vApplicationIdleHook(void) { }
extern void vApplicationTickHook(void) { }
extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* lvgl                                                                 */
/************************************************************************/

static void event_handler(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	if(code == LV_EVENT_CLICKED) {
		if (setPower == 0){
			setPower = 1;
			lv_scr_load(scr2);
		} else {
			setPower = 0;
			lv_scr_load(scr1);
		}
	}
}

static void but2_callback(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	if(code == LV_EVENT_CLICKED) { LV_LOG_USER("Clicked"); }
	else if(code == LV_EVENT_VALUE_CHANGED) { LV_LOG_USER("Toggled"); }
}

static void but3_callback(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	if(code == LV_EVENT_CLICKED) { LV_LOG_USER("Clicked"); }
	else if(code == LV_EVENT_VALUE_CHANGED) { LV_LOG_USER("Toggled"); }
}

static void down_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    char *c;
    int temp;
    if(code == LV_EVENT_CLICKED) {
        c = lv_label_get_text(labelSetValue);
        temp = atoi(c);
        lv_label_set_text_fmt(labelSetValue, "%02d", temp - 1);
    }
}

static void up_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    char *c;
    int temp;
    if(code == LV_EVENT_CLICKED) {
        c = lv_label_get_text(labelSetValue);
        temp = atoi(c);
        lv_label_set_text_fmt(labelSetValue, "%02d", temp + 1);
    }
}

void lv_termostato(void) {
	static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_bg_color(&style, lv_color_black());
    lv_style_set_border_color(&style, lv_color_black());
    lv_style_set_border_width(&style, 5);
	
	scr1 = lv_obj_create(NULL);
	scr2 = lv_obj_create(NULL);
	
	lv_scr_load(scr1);
	
    lv_obj_t * btn1 = lv_btn_create(scr1);
    lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn1, LV_ALIGN_BOTTOM_LEFT, 5, -10);
    labelBtn1 = lv_label_create(btn1);
	lv_label_set_text(labelBtn1, "[  "LV_SYMBOL_POWER);
    lv_obj_center(labelBtn1);
	lv_obj_add_style(btn1, &style, 0);
	
	lv_obj_t * btn6 = lv_btn_create(scr2);
	lv_obj_add_event_cb(btn6, event_handler, LV_EVENT_ALL, NULL);
	lv_obj_align(btn6, LV_ALIGN_BOTTOM_LEFT, 20, -20);
	labelBtn6 = lv_label_create(btn6);
	lv_label_set_text(labelBtn6, LV_SYMBOL_POWER);
	lv_obj_center(labelBtn6);
	lv_obj_add_style(btn6, &style, 0);

	lv_obj_t * btn2 = lv_btn_create(scr1);
    lv_obj_add_event_cb(btn2, but2_callback, LV_EVENT_ALL, NULL);
	lv_obj_align_to(btn2, btn1, LV_ALIGN_OUT_RIGHT_TOP, -5, 0);
    labelBtn2 = lv_label_create(btn2);
	lv_label_set_text(labelBtn2, "| M |" );
    lv_obj_center(labelBtn2);
	lv_obj_add_style(btn2, &style, 0);

	lv_obj_t * btn3 = lv_imgbtn_create(scr1);
	lv_imgbtn_set_src(btn3, LV_IMGBTN_STATE_RELEASED, 0, &clock, 0);
	lv_obj_add_flag(btn3, LV_OBJ_FLAG_CHECKABLE);
	lv_obj_add_event_cb(btn3, but3_callback, LV_EVENT_ALL, NULL);
	lv_obj_set_height(btn3, 25);
	lv_obj_set_width(btn3, 25);
	lv_obj_align_to(btn3, btn2, LV_ALIGN_OUT_RIGHT_TOP, 3, 9);

	labelChave = lv_label_create(scr1);
	lv_obj_align_to(labelChave, btn3, LV_ALIGN_OUT_RIGHT_MID, 0, -3);
	lv_obj_set_style_text_color(labelChave, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelChave, "  ]");

	labelHome = lv_label_create(scr1);
	lv_obj_align_to(labelHome, labelChave, LV_ALIGN_OUT_RIGHT_MID, 0, -30);
	lv_obj_set_style_text_color(labelHome, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelHome, LV_SYMBOL_HOME"");

	lv_obj_t * btn4 = lv_btn_create(scr1);
    lv_obj_add_event_cb(btn4, up_handler, LV_EVENT_ALL, NULL);
	lv_obj_align_to(btn4, labelChave, LV_ALIGN_OUT_RIGHT_TOP, 20, -7);
    labelBtn4 = lv_label_create(btn4);
	lv_label_set_text(labelBtn4, "[ "LV_SYMBOL_UP"  |");
    lv_obj_center(labelBtn4);
	lv_obj_add_style(btn4, &style, 0);

	lv_obj_t * btn5 = lv_btn_create(scr1);
    lv_obj_add_event_cb(btn5, down_handler, LV_EVENT_ALL, NULL);
	lv_obj_align_to(btn5, btn4, LV_ALIGN_OUT_RIGHT_TOP, -5, 0);
    labelBtn5 = lv_label_create(btn5);
	lv_label_set_text(labelBtn5, LV_SYMBOL_DOWN"  ]");
    lv_obj_center(labelBtn5);
	lv_obj_add_style(btn5, &style, 0);

	labelFloor = lv_label_create(scr1);
    lv_obj_align(labelFloor, LV_ALIGN_LEFT_MID, 35 , -25);
    lv_obj_set_style_text_font(labelFloor, &dseg70, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(labelFloor, lv_color_white(), LV_STATE_DEFAULT);
    lv_label_set_text_fmt(labelFloor, "%02d", 23);

	labelFloorDigit = lv_label_create(scr1);
    lv_obj_align_to(labelFloorDigit, labelFloor, LV_ALIGN_OUT_RIGHT_BOTTOM, 0, -15);
    lv_obj_set_style_text_font(labelFloorDigit, &dseg40, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(labelFloorDigit, lv_color_white(), LV_STATE_DEFAULT);
    lv_label_set_text_fmt(labelFloorDigit, ".%d", 4);

	labelClock = lv_label_create(scr1);
	lv_obj_align(labelClock, LV_ALIGN_TOP_RIGHT, -3 , 3);
    lv_obj_set_style_text_font(labelClock, &dseg25, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(labelClock, lv_color_white(), LV_STATE_DEFAULT);
    lv_label_set_text_fmt(labelClock, "17:05");

	labelSetValue = lv_label_create(scr1);
    lv_obj_align_to(labelSetValue, labelClock, LV_ALIGN_OUT_BOTTOM_LEFT, 3, 30);    
    lv_obj_set_style_text_font(labelSetValue, &dseg40, LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(labelSetValue, lv_color_white(), LV_STATE_DEFAULT);
    lv_label_set_text_fmt(labelSetValue, "%02d", 22);

	labelCelsius = lv_label_create(scr1);
	lv_obj_align_to(labelCelsius, labelFloor, LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
	lv_obj_set_style_text_font(labelCelsius, &monts15, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelCelsius, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelCelsius, "°C");

	labelConfig = lv_label_create(scr1);
	lv_obj_align_to(labelConfig, labelSetValue, LV_ALIGN_OUT_LEFT_TOP, 17, 0);
	lv_obj_set_style_text_color(labelConfig, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelConfig, LV_SYMBOL_SETTINGS);

	labelSet = lv_label_create(scr1);
	lv_obj_align_to(labelSet, labelConfig, LV_ALIGN_OUT_BOTTOM_LEFT, -10, 10);
	lv_obj_set_style_text_font(labelSet, &monts15, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelSet, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelSet, "MAIO");

	labelClock2 = lv_img_create(scr1);
	lv_img_set_src(labelClock2, &clock);
	lv_obj_set_height(labelClock2, 25);
	lv_obj_set_width(labelClock2, 25);
	lv_obj_align_to(labelClock2, labelSetValue, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 15);

	labelFumaca = lv_img_create(scr1);
	lv_img_set_src(labelFumaca, &fumaca);
	lv_obj_set_height(labelFumaca, 25);
	lv_obj_set_width(labelFumaca, 30);
	lv_obj_align_to(labelFumaca, labelClock2, LV_ALIGN_OUT_RIGHT_MID, 8, -2);

	labelDia = lv_label_create(scr1);
	lv_obj_align(labelDia, LV_ALIGN_TOP_LEFT, 10, 10);
	lv_obj_set_style_text_font(labelDia, &monts15, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelDia, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelDia, "SEG");

	labelCelsius2 = lv_label_create(scr1);
	lv_obj_align_to(labelCelsius2, labelSetValue, LV_ALIGN_OUT_RIGHT_TOP, 2, 0);
	lv_obj_set_style_text_font(labelCelsius2, &monts15, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelCelsius2, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelCelsius2, "°C");

	labelFT = lv_label_create(scr1);
	lv_obj_align(labelFT, LV_ALIGN_LEFT_MID, 2 , -55);
	lv_obj_set_style_text_font(labelFT, &monts10, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelFT, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelFT, "FLOOR");

	labelFT2 = lv_label_create(scr1);
	lv_obj_align_to(labelFT2, labelFT, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 2);
	lv_obj_set_style_text_font(labelFT2, &monts10, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelFT2, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelFT2, "TEMP");
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_lcd(void *pvParameters) {
	int px, py;
	lv_termostato();
	for (;;)  {
		xSemaphoreTake(xMutexLVGL, portMAX_DELAY);
		lv_tick_inc(50);
		lv_task_handler();
		xSemaphoreGive(xMutexLVGL);
		vTaskDelay(50);
	}
}

static void task_clock(void *pvParameters) {
	calendar rtc_initial = {2023, 5, 3, 1, 15, 50, 1};                                            
    RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN);  
	uint32_t current_hour, current_min, current_sec;
	uint32_t current_year, current_month, current_day, current_week;
	for (;;)  {
		rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
		rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
		lv_label_set_text_fmt(labelClock, "%02d:%02d", current_hour, current_min);
		vTaskDelay(1000);
		lv_label_set_text_fmt(labelClock, "%02d %02d", current_hour, current_min);
		vTaskDelay(1000);
	}
}

/************************************************************************/
/* configs                                                              */
/************************************************************************/

static void configure_lcd(void) {
	pio_configure_pin(LCD_SPI_MISO_PIO, LCD_SPI_MISO_FLAGS);  //
	pio_configure_pin(LCD_SPI_MOSI_PIO, LCD_SPI_MOSI_FLAGS);
	pio_configure_pin(LCD_SPI_SPCK_PIO, LCD_SPI_SPCK_FLAGS);
	pio_configure_pin(LCD_SPI_NPCS_PIO, LCD_SPI_NPCS_FLAGS);
	pio_configure_pin(LCD_SPI_RESET_PIO, LCD_SPI_RESET_FLAGS);
	pio_configure_pin(LCD_SPI_CDS_PIO, LCD_SPI_CDS_FLAGS);
	ili9341_init();
	ili9341_backlight_on();
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT,
	};
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);
	setbuf(stdout, NULL);
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


/************************************************************************/
/* port lvgl                                                            */
/************************************************************************/

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
	ili9341_set_top_left_limit(area->x1, area->y1);   ili9341_set_bottom_right_limit(area->x2, area->y2);
	ili9341_copy_pixels_to_screen(color_p,  (area->x2 + 1 - area->x1) * (area->y2 + 1 - area->y1));
	lv_disp_flush_ready(disp_drv);
}

void my_input_read(lv_indev_drv_t * drv, lv_indev_data_t*data) {
	int px, py, pressed;
	if (readPoint(&px, &py))
		data->state = LV_INDEV_STATE_PRESSED;
	else
		data->state = LV_INDEV_STATE_RELEASED; 
	data->point.x = px;
	data->point.y = py;
}

void configure_lvgl(void) {
	lv_init();
	lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX);
	
	lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
	disp_drv.draw_buf = &disp_buf;          /*Set an initialized buffer*/
	disp_drv.flush_cb = my_flush_cb;        /*Set a flush callback to draw to the display*/
	disp_drv.hor_res = LV_HOR_RES_MAX;      /*Set the horizontal resolution in pixels*/
	disp_drv.ver_res = LV_VER_RES_MAX;      /*Set the vertical resolution in pixels*/

	lv_disp_t * disp;
	disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/
	
	/* Init input on LVGL */
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = my_input_read;
	lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* board and sys init */
	board_init();
	sysclk_init();
	configure_console();
	/* Disable the watchdog */                                                                      
    WDT->WDT_MR = WDT_MR_WDDIS;  

	/* LCd, touch and lvgl init*/
	configure_lcd();
	configure_touch();
	configure_lvgl();
	
	xMutexLVGL = xSemaphoreCreateMutex();

	/* Create task to control oled */
	if (xTaskCreate(task_lcd, "LCD", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create lcd task\r\n");
	}

	if (xTaskCreate(task_clock, "Clock", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create clock task\r\n");
	}
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){ }
}
