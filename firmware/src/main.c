/************************************************************************
* 5 semestre - Eng. da Computao - Insper
*
* 2021 - Exemplo com HC05 com RTOS
*
*/

#include <asf.h>
#include "conf_board.h"
#include <string.h>

/************************************************************************/
/* defines                                                              */
/************************************************************************/

// LEDs
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)
#define SW_PIO 		 PIOC 			   // periferico que controla o O SW
#define SW_PIO_ID	 ID_PIOC  // ID do periférico PIOC (controla SW)
#define SW_PIO_IDX	 13 			   // ID do SW no PIO
#define SW_PIO_IDX_MASK  (1 << SW_PIO_IDX)   // Mascara para CONTROLARMOS o SW

#define DT_PIO 		 PIOD 			   // periferico que controla o O DT
#define DT_PIO_ID	 ID_PIOD  // ID do periférico PIOC (controla DT)
#define DT_PIO_IDX	 11 			   // ID do DT no PIO
#define DT_PIO_IDX_MASK  (1 << DT_PIO_IDX)   // Mascara para CONTROLARMOS o DT

#define CLK_PIO 		 PIOD 			   // periferico que controla o O CLK
#define CLK_PIO_ID	 ID_PIOD  // ID do periférico PIOC (controla CLK)
#define CLK_PIO_IDX	 26 			   // ID do CLK no PIO
#define CLK_PIO_IDX_MASK  (1 << CLK_PIO_IDX)   // Mascara para CONTROLARMOS o CLK
// Botão VERDE
#define BUT_PIO      PIOD
#define BUT_PIO_ID   ID_PIOD
#define BUT_IDX      11
#define BUT_IDX_MASK (1 << BUT_IDX)

// Botão AMARELO
#define BUTAMARELO_PIO      PIOA
#define BUTAMARELO_PIO_ID   ID_PIOA
#define BUTAMARELO_IDX      6
#define BUTAMARELO_IDX_MASK (1 << BUTAMARELO_IDX)

// Botão VERMELHO
#define BUTVERMELHO_PIO      PIOD
#define BUTVERMELHO_PIO_ID   ID_PIOD
#define BUTVERMELHO_IDX      26
#define BUTVERMELHO_IDX_MASK (1 << BUTVERMELHO_IDX)

// Botão AZUL
#define BUTAZUL_PIO      PIOC
#define BUTAZUL_PIO_ID   ID_PIOC
#define BUTAZUL_IDX      19
#define BUTAZUL_IDX_MASK (1 << BUTAZUL_IDX)
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);
void but4_callback(void);
volatile int inteiro;
/*
#define BUT_PIO      PIOA
#define BUT_PIO_ID   ID_PIOA
#define BUT_IDX      11
#define BUT_IDX_MASK (1 << BUT_IDX)*/

/*
#define BUTVERDE_PIO      PIOA
#define BUTVERDE_PIO_ID   ID_PIOA
#define BUTVERDE_IDX      11
#define BUTVERDE_IDX_MASK (1 << BUT_IDX)*/

// usart (bluetooth ou serial)
// Descomente para enviar dados
// pela serial debug

//#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
#define USART_COM USART1
#define USART_COM_ID ID_USART1
#else
#define USART_COM USART0
#define USART_COM_ID ID_USART0
#endif

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_BLUETOOTH_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_BLUETOOTH_STACK_PRIORITY        (tskIDLE_PRIORITY)

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);
// Queues
QueueHandle_t xQueue;
/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/
void sw_callback(void){
	
	
}

void dt_callback(void){
	

	
}

void clk_callback(void){
	
	// se for borda de descida
	
	
	
}
void but1_callback(void){
	printf("1");
	//xQueueSendFromISR(xQueue,1,NULL);
}

void but2_callback(void){
	printf("2");
	//xQueueSendFromISR(xQueue,2,NULL);
}

void but3_callback(void){
	printf("3");
	//xQueueSendFromISR(xQueue,3,NULL);
}
void but4_callback(void){
	printf("4");
	//xQueueSendFromISR(xQueue,4,NULL);
}
/************************************************************************/
/* RTOS application HOOK                                                */
/************************************************************************/

/* Called if stack overflow during execution */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	* identify which task has overflowed its stack.
	*/
	for (;;) {
	}
}

/* This function is called by FreeRTOS idle task */
extern void vApplicationIdleHook(void) {
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/* This function is called by FreeRTOS each tick */
extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void io_init(void) {

	// Ativa PIOs
	pmc_enable_periph_clk(LED_PIO_ID);
	pmc_enable_periph_clk(BUT_PIO_ID);
	pmc_enable_periph_clk(BUTAMARELO_PIO_ID);
	pmc_enable_periph_clk(BUTVERMELHO_PIO_ID);
	pmc_enable_periph_clk(BUTAZUL_PIO_ID);
	
	pio_set_input(BUT_PIO, BUT_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUTAMARELO_PIO, BUTAMARELO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUTAZUL_PIO, BUTAZUL_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUTVERMELHO_PIO, BUTVERMELHO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	
	pio_set_debounce_filter(BUT_PIO, BUT_IDX_MASK, 10);
	pio_set_debounce_filter(BUTAMARELO_PIO, BUTAMARELO_IDX_MASK, 10);
	pio_set_debounce_filter(BUTVERMELHO_PIO, BUTVERMELHO_IDX_MASK, 10);
	pio_set_debounce_filter(BUTAZUL_PIO, BUTAZUL_IDX_MASK, 10);
	
	

	// configura NVIC para receber interrupcoes do PIO do botão 1
	// configura interrupção no pino referente ao botão 2
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_IDX_MASK, PIO_IT_RISE_EDGE, but1_callback);

	// habilita interrupção
	pio_enable_interrupt(BUT_PIO, BUT_IDX_MASK);
	// limpa interrupção
	pio_get_interrupt_status(BUT_PIO);
	
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);
	
	
	// configura interrupção no pino referente ao botão 2
	pio_handler_set(BUTVERMELHO_PIO, BUTVERMELHO_PIO_ID, BUTVERMELHO_IDX_MASK, PIO_IT_RISE_EDGE, but2_callback);
	
	
	// habilita interrupção
	pio_enable_interrupt(BUTVERMELHO_PIO, BUTVERMELHO_IDX_MASK);
	
	// limpa interrupção
	pio_get_interrupt_status(BUTVERMELHO_PIO);
	
	// configura NVIC para receber interrupcoes do PIO do botão 2
	NVIC_EnableIRQ(BUTVERMELHO_PIO_ID);
	NVIC_SetPriority(BUTVERMELHO_PIO_ID, 4);
	
	// configura interrupção no pino referente ao botão 3
	pio_handler_set(BUTAMARELO_PIO, BUTAMARELO_PIO_ID, BUTAMARELO_IDX_MASK, PIO_IT_RISE_EDGE, but3_callback);

	// habilita interrupção
	pio_enable_interrupt(BUTAMARELO_PIO, BUTAMARELO_IDX_MASK);
	// limpa interrupção
	pio_get_interrupt_status(BUTAMARELO_PIO);

	// configura NVIC para receber interrupcoes do PIO do botão 3
	NVIC_EnableIRQ(BUTAMARELO_PIO_ID);
	NVIC_SetPriority(BUTAMARELO_PIO_ID, 4);
	
	// configura interrupção no pino referente ao botão 3
	pio_handler_set(BUTAZUL_PIO, BUTAZUL_PIO_ID, BUTAZUL_IDX_MASK, PIO_IT_RISE_EDGE, but4_callback);

	// habilita interrupção
	pio_enable_interrupt(BUTAZUL_PIO, BUTAZUL_IDX_MASK);
	// limpa interrupção
	pio_get_interrupt_status(BUTAZUL_PIO);

	// configura NVIC para receber interrupcoes do PIO do botão 3
	NVIC_EnableIRQ(BUTAZUL_PIO_ID);
	NVIC_SetPriority(BUTAZUL_PIO_ID, 4);
	
	/*
	// configura o botão sw como entrada com pull-up e debounce
	pmc_enable_periph_clk(SW_PIO_ID);
	pio_set_input(SW_PIO, SW_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(SW_PIO, SW_PIO_IDX_MASK, 10);

	// configura interrupção no pino referente ao botão sw
	pio_handler_set(SW_PIO, SW_PIO_ID, SW_PIO_IDX_MASK, PIO_IT_EDGE, sw_callback);

	// habilita interrupção
	pio_enable_interrupt(SW_PIO, SW_PIO_IDX_MASK);
	// limpa interrupção
	pio_get_interrupt_status(SW_PIO);

	// configura NVIC para receber interrupcoes do PIO do botão sw
	NVIC_EnableIRQ(SW_PIO_ID);
	NVIC_SetPriority(SW_PIO_ID, 4);

	// configura o pino do dt como entrada 
	pmc_enable_periph_clk(DT_PIO_ID);
	pio_set_input(DT_PIO, DT_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(DT_PIO, DT_PIO_IDX_MASK, 60);

	// configura o pino do clk como entrada
	pmc_enable_periph_clk(CLK_PIO_ID);
	pio_set_input(CLK_PIO, CLK_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(CLK_PIO, CLK_PIO_IDX_MASK, 60);

	// configura interrupção no pino referente ao dt
	pio_handler_set(DT_PIO, DT_PIO_ID, DT_PIO_IDX_MASK, PIO_IT_EDGE, dt_callback);

	// habilita interrupção
	pio_enable_interrupt(DT_PIO, DT_PIO_IDX_MASK);
	// limpa interrupção
	pio_get_interrupt_status(DT_PIO);

	// configura NVIC para receber interrupcoes do PIO do dt
	NVIC_EnableIRQ(DT_PIO_ID);
	NVIC_SetPriority(DT_PIO_ID, 2);

	// configura interrupção no pino referente ao clk
	pio_handler_set(CLK_PIO, CLK_PIO_ID, CLK_PIO_IDX_MASK, PIO_IT_EDGE, clk_callback);

	// habilita interrupção
	pio_enable_interrupt(CLK_PIO, CLK_PIO_IDX_MASK);
	// limpa interrupção
	pio_get_interrupt_status(CLK_PIO);

	// configura NVIC para receber interrupcoes do PIO do clk
	NVIC_EnableIRQ(CLK_PIO_ID);
	NVIC_SetPriority(CLK_PIO_ID, 2);
	*/
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
		#endif
		.paritytype = CONF_UART_PARITY,
		#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
		#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	#if defined(__GNUC__)
	setbuf(stdout, NULL);
	#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	* emits one character at a time.
	*/
	#endif
}

uint32_t usart_puts(uint8_t *pstring) {
	uint32_t i ;

	while(*(pstring + i))
	if(uart_is_tx_empty(USART_COM))
	usart_serial_putchar(USART_COM, *(pstring+i++));
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, uint timeout_ms) {
	uint timecounter = timeout_ms;
	uint32_t rx;
	uint32_t counter = 0;

	while( (timecounter > 0) && (counter < bufferlen - 1)) {
		if(usart_read(usart, &rx) == 0) {
			buffer[counter++] = rx;
		}
		else{
			timecounter--;
			vTaskDelay(1);
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen,
char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void config_usart0(void) {
	sysclk_enable_peripheral_clock(ID_USART0);
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);

	// RX - PB0  TX - PB1
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_init(void) {
	char buffer_rx[128];
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+NAMEDiscord", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+PIN0000", 100);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

void task_bluetooth(void) {
	printf("Task Bluetooth started \n");
	
	printf("Inicializando HC05 \n");
	config_usart0();
	hc05_init();
	printf("Inicializando IO\n");
	// configura LEDs e Botões
	io_init();
	printf("Teste IO");

	char button1 = '0';
	char eof = 'X';

	// Task não deve retornar.
	while(1) {
		
			// envia status botão
			while(!usart_is_tx_ready(USART_COM)) {
				vTaskDelay(10 / portTICK_PERIOD_MS);
			}
			usart_write(USART_COM, button1);
			
			// envia fim de pacote
			while(!usart_is_tx_ready(USART_COM)) {
				vTaskDelay(10 / portTICK_PERIOD_MS);
			}
			usart_write(USART_COM, eof);

			// dorme por 500 ms
			vTaskDelay(500 / portTICK_PERIOD_MS);
		
	}
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;

	configure_console();

	/* Create task to make led blink */
	xTaskCreate(task_bluetooth, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL,	TASK_BLUETOOTH_STACK_PRIORITY, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
