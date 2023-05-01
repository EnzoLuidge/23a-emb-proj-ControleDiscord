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
#define DT_PIO_IDX	 28			   // ID do DT no PIO
#define DT_PIO_IDX_MASK  (1 << DT_PIO_IDX)   // Mascara para CONTROLARMOS o DT

#define CLK_PIO 	 PIOA 			   // periferico que controla o O CLK
#define CLK_PIO_ID	 ID_PIOA  // ID do periférico PIOC (controla CLK)
#define CLK_PIO_IDX	 9 			   // ID do CLK no PIO
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

volatile int counter = 1;
// Botão VERMELHO
#define BUTVERMELHO_PIO      PIOD
#define BUTVERMELHO_PIO_ID   ID_PIOD
#define BUTVERMELHO_IDX      26
#define BUTVERMELHO_IDX_MASK (1 << BUTVERMELHO_IDX)
volatile char button1;
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

#define USART_COM_ID ID_USART1
#define USART_COM USART1

#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PD30

/************************************************************************/
/* RTOS                                                                */
/************************************************************************/

#define TASK_ADC_STACK_SIZE (1024 * 10 / sizeof(portSTACK_TYPE))
#define TASK_ADC_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_PROC_STACK_SIZE (1024 * 10 / sizeof(portSTACK_TYPE))
#define TASK_PROC_STACK_PRIORITY (tskIDLE_PRIORITY)


TimerHandle_t xTimer;

/** Queue for msg log send data */
QueueHandle_t xQueueADC;
QueueHandle_t xQueueProc;

typedef struct {
  uint value;
} adcData;

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback);

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
volatile flag_sw = 0;
volatile flag_dt = 0;
volatile flag_clk = 0;
/************************************************************************/

static void AFEC_pot_callback(void) {
  adcData adc;
  adc.value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
  BaseType_t xHigherPriorityTaskWoken = pdTRUE;
  xQueueSendFromISR(xQueueProc, &adc, &xHigherPriorityTaskWoken);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

void vTimerCallback(TimerHandle_t xTimer) {
  /* Selecina canal e inicializa conversão */
  afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
  afec_start_software_conversion(AFEC_POT);
}

static void task_proc(void *pvParameters) {
    // configura ADC e TC para controlar a leitura
  config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_callback);

  xTimer = xTimerCreate(/* Just a text name, not used by the RTOS
                        kernel. */
                        "Timer",
                        /* The timer period in ticks, must be
                        greater than 0. */
                        100,
                        /* The timers will auto-reload themselves
                        when they expire. */
                        pdTRUE,
                        /* The ID is used to store a count of the
                        number of times the timer has expired, which
                        is initialised to 0. */
                        (void *)0,
                        /* Timer callback */
                        vTimerCallback);
  xTimerStart(xTimer, 0);

  // variável para recever dados da fila
  adcData adc;
  // variavel para guardar quantidade de dados
  uint32_t count = 0;
  // variavel para guardar a soma dos dados
  uint32_t sum = 0;
  
  BaseType_t xHigherPriorityTaskWoken = pdTRUE;
  // recebendo dados da fila
  while (1) {
    if (xQueueReceive(xQueueProc, &(adc), 1000)) {
      count++;
      sum += adc.value;
      if (count == 18) {
        adc.value = sum / 18;
        count = 0;
        sum = 0;
        xQueueSendFromISR(xQueueADC, &adc, &xHigherPriorityTaskWoken);
      }
    } else {
      printf("Nao chegou um novo dado em 1 segundo em proc");
    }
  }
}


static void task_adc(void *pvParameters) {

  // configura ADC e TC para controlar a leitura
  config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_callback);

  xTimer = xTimerCreate(/* Just a text name, not used by the RTOS
                        kernel. */
                        "Timer",
                        /* The timer period in ticks, must be
                        greater than 0. */
                        100,
                        /* The timers will auto-reload themselves
                        when they expire. */
                        pdTRUE,
                        /* The ID is used to store a count of the
                        number of times the timer has expired, which
                        is initialised to 0. */
                        (void *)0,
                        /* Timer callback */
                        vTimerCallback);
  xTimerStart(xTimer, 0);

  // variável para recever dados da fila
  adcData adc;
  adcData lastadc;
  // incializa o valor de lastadc para 0
  lastadc.value = 0;

  while (1) {
    if (xQueueReceive(xQueueADC, &(adc), 1000)) {
      printf("ADC: %d \n", adc);
	  if (adc.value - 50 > lastadc.value) {
		printf("Subindo \n");
		button1 = 6;
		// enviar para a fila
		xQueueSend(xQueue, &button1, 0);
	  } else if (adc.value + 50 < lastadc.value) {
		printf("Descendo \n");
		button1 = 7;
		// enviar para a fila
		xQueueSend(xQueue, &button1, 0);
	  }
	  lastadc.value = adc.value;
    } else {
      printf("Nao chegou um novo dado em 1 segundo");
    }
  }
}


static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback) {
  /*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
  afec_enable(afec);

  /* struct de configuracao do AFEC */
  struct afec_config afec_cfg;

  /* Carrega parametros padrao */
  afec_get_config_defaults(&afec_cfg);

  /* Configura AFEC */
  afec_init(afec, &afec_cfg);

  /* Configura trigger por software */
  afec_set_trigger(afec, AFEC_TRIG_SW);

  /*** Configuracao específica do canal AFEC ***/
  struct afec_ch_config afec_ch_cfg;
  afec_ch_get_config_defaults(&afec_ch_cfg);
  afec_ch_cfg.gain = AFEC_GAINVALUE_0;
  afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

  /*
  * Calibracao:
  * Because the internal ADC offset is 0x200, it should cancel it and shift
  down to 0.
  */
  afec_channel_set_analog_offset(afec, afec_channel, 0x200);

  /***  Configura sensor de temperatura ***/
  struct afec_temp_sensor_config afec_temp_sensor_cfg;

  afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
  afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

  /* configura IRQ */
  afec_set_callback(afec, afec_channel, callback, 1);
  NVIC_SetPriority(afec_id, 4);
  NVIC_EnableIRQ(afec_id);
}









void sw_callback(void){

	if (flag_sw == 0){
		flag_sw = 1;
	}else{
		flag_sw = 0;
		button1 = '5';
		xQueueSendFromISR(xQueue,button1,NULL);
		}
	
	
}

void dt_callback(void){
	// if fall edge print a test string
	if (flag_dt == 0){
		//printf("dt_callback \r \n");
		flag_dt = 1;
		if (flag_clk == 1){
			printf("giro horario \r \n");
			button1 = '6';
			xQueueSendFromISR(xQueue,button1,NULL);
		}
	}
	else{
		//printf("dt_callbackSubiu \r \n");
		flag_dt = 0;
	}	

	
}

void clk_callback(void){
	
	// se for borda de descida

	if (flag_clk == 0){
		//printf("clk_callback \r \n");
		flag_clk = 1;
		if (flag_dt == 1){
			printf("giro anti-horario \r \n");
			button1 = '7';
			xQueueSendFromISR(xQueue,button1,NULL);
		}
	}
	else{
		//printf("clk_callbackSubiu \r \n");
		flag_clk = 0;
	}
	
	
	
}
void but1_callback(void){
	button1='1';
	xQueueSendFromISR(xQueue,button1,NULL);
}

void but2_callback(void){
	button1='2';
	xQueueSendFromISR(xQueue,button1,NULL);
}

void but3_callback(void){
	button1='3';
	xQueueSendFromISR(xQueue,button1,NULL);
}
void but4_callback(void){
	button1='4';
	xQueueSendFromISR(xQueue,button1,NULL);
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

	button1 = '0';
	char eof = 'X';

	// Task não deve retornar.
	while(1) {
		/*
		if(pio_get(BUT_PIO, PIO_INPUT, BUT_IDX_MASK) == 0) {
			button1 = '1';
		} 
		else if(pio_get(BUTAMARELO_PIO,PIO_INPUT,BUTAMARELO_IDX_MASK)==0){
			button1='2';
			}
		else if(pio_get(BUTAZUL_PIO,PIO_INPUT,BUTAZUL_IDX_MASK)==0){
			button1='3';
			}
		else if(pio_get(BUTVERMELHO_PIO,PIO_INPUT,BUTVERMELHO_IDX_MASK)==0){
			button1='4';
			}
		else {
			button1 = '0';
		}
		*/
		// recebe status botão via fila
		// desabilite isto se não estiver usando fila
		if(xQueueReceive(xQueue, &button1, 0) == pdTRUE) {
			printf("Botão: %c \n", button1);
		}
		
		
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

	// incia a queue
	// queue
	xQueue = xQueueCreate(100, sizeof(float));

	xQueueADC = xQueueCreate(100, sizeof(adcData));
  if (xQueueADC == NULL)
    printf("falha em criar a queue xQueueADC \n");
  xQueueProc = xQueueCreate(100, sizeof(adcData));
  if (xQueueProc == NULL)
    printf("falha em criar a queue xQueueProc \n");

  if (xTaskCreate(task_adc, "ADC", TASK_ADC_STACK_SIZE, NULL,
                  TASK_ADC_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create test ADC task\r\n");
  }
  if (xTaskCreate(task_proc, "PROC", TASK_PROC_STACK_SIZE, NULL,
                  TASK_PROC_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create test PROC task\r\n");
  }


	if (xQueue == NULL)
	printf("falha em criar a queue xQueue \n");

	/* Create task to make led blink */
	xTaskCreate(task_bluetooth, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL,	TASK_BLUETOOTH_STACK_PRIORITY, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
