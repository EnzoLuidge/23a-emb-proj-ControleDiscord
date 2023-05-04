#include <asf.h>
#include "conf_board.h"
#include <string.h>

// LEDs
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

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
volatile int inteiro;

// usart (bluetooth ou serial)
// Descomente para enviar dados
// pela serial debug

#define USART_COM_ID ID_USART1
#define USART_COM USART1

#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PD30

#define TASK_ADC_STACK_SIZE (1024 * 10 / sizeof(portSTACK_TYPE))
#define TASK_ADC_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_PROC_STACK_SIZE (1024 * 10 / sizeof(portSTACK_TYPE))
#define TASK_PROC_STACK_PRIORITY (tskIDLE_PRIORITY)

TimerHandle_t xTimer;

QueueHandle_t xQueueADC;
QueueHandle_t xQueueProc;

typedef struct {
  uint value;
} adcData;

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback);
volatile char flag_mute = 0;

//#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
#define USART_COM USART1
#define USART_COM_ID ID_USART1
#else
#define USART_COM USART0
#define USART_COM_ID ID_USART0
#endif

#define TASK_BLUETOOTH_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_BLUETOOTH_STACK_PRIORITY        (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

QueueHandle_t xQueue;
                                                
volatile flag_sw = 0;
volatile flag_dt = 0;
volatile flag_clk = 0;

static void AFEC_pot_callback(void) {
  adcData adc;
  adc.value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
  BaseType_t xHigherPriorityTaskWoken = pdTRUE;
  xQueueSendFromISR(xQueueProc, &adc, &xHigherPriorityTaskWoken);
}

void vTimerCallback(TimerHandle_t xTimer) {
  /* Selecina canal e inicializa conversão */
  afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
  afec_start_software_conversion(AFEC_POT);
}

static void task_proc(void *pvParameters) {
    // configura ADC e TC para controlar a leitura
  config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_callback);
  xTimer = xTimerCreate("Timer", 100, pdTRUE, (void *)0, vTimerCallback);
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
      if (count == 10) {
        adc.value = sum / 10;
        count = 0;
        sum = 0;
        xQueueSendFromISR(xQueueADC, &adc, &xHigherPriorityTaskWoken);
      }
    } else {
      // //printf("Nao chegou um novo dado em 1 segundo em proc");
    }
  }
}

static void task_adc(void *pvParameters) {
  // configura ADC e TC para controlar a leitura
  config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_callback);
  xTimer = xTimerCreate("Timer", 100, pdTRUE, (void *)0, vTimerCallback);
  xTimerStart(xTimer, 0);
  // variável para recever dados da fila
  adcData adc;
  adcData lastadc;
  // incializa o valor de lastadc para 0
  lastadc.value = 0;
  while (1) {
    if (xQueueReceive(xQueueADC, &(adc), 1000)) {
      //printf("ADC: %d \n", adc);
	  //printf("LastADC: %d \n", lastadc);
	  if (adc.value - 50 > lastadc.value && adc.value > 50) {
		//printf("Subindo \n");
		button1 = '6';
		// enviar para a fila
		xQueueSend(xQueue, &button1, 0);
	  } else if (adc.value + 50 < lastadc.value) {
		//printf("Descendo \n");
		button1 = '7';
		// enviar para a fila
		xQueueSend(xQueue, &button1, 0);
	  }
	  lastadc.value = adc.value;
    } else {
      //printf("Nao chegou um novo dado em 1 segundo");
    }
  }
}


static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback) {
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

void but1_callback(void){
	 if (flag_mute == 0) {
	 	pio_clear(LED_PIO, LED_IDX_MASK);
	 	flag_mute = 1;
	 } else {
	 	pio_set(LED_PIO, LED_IDX_MASK);
	 	flag_mute = 0;
	 }
	char button;
	button='1';
	xQueueSendFromISR(xQueue,&button,NULL);
}

void but2_callback(void){
	char button;
	button='2';
	xQueueSendFromISR(xQueue,&button,NULL);
}

void but3_callback(void){
	char button;
	button='3';
	xQueueSendFromISR(xQueue,&button,NULL);
}
void but4_callback(void){
	char button;
	button='4';
	xQueueSendFromISR(xQueue,&button,NULL);
}

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {
	}
}
extern void vApplicationIdleHook(void) {
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}
extern void vApplicationTickHook(void) { }
extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

void io_init(void) {
	// Ativa PIOs
	pio_configure(LED_PIO, PIO_OUTPUT_1, LED_IDX_MASK, PIO_DEFAULT);
	
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

	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_IDX_MASK, PIO_IT_RISE_EDGE, but1_callback);
	pio_enable_interrupt(BUT_PIO, BUT_IDX_MASK);
	pio_get_interrupt_status(BUT_PIO);
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);
	
	pio_handler_set(BUTVERMELHO_PIO, BUTVERMELHO_PIO_ID, BUTVERMELHO_IDX_MASK, PIO_IT_RISE_EDGE, but2_callback);
	pio_enable_interrupt(BUTVERMELHO_PIO, BUTVERMELHO_IDX_MASK);
	pio_get_interrupt_status(BUTVERMELHO_PIO);
	NVIC_EnableIRQ(BUTVERMELHO_PIO_ID);
	NVIC_SetPriority(BUTVERMELHO_PIO_ID, 4);
	
	pio_handler_set(BUTAMARELO_PIO, BUTAMARELO_PIO_ID, BUTAMARELO_IDX_MASK, PIO_IT_RISE_EDGE, but3_callback);
	pio_enable_interrupt(BUTAMARELO_PIO, BUTAMARELO_IDX_MASK);
	pio_get_interrupt_status(BUTAMARELO_PIO);
	NVIC_EnableIRQ(BUTAMARELO_PIO_ID);
	NVIC_SetPriority(BUTAMARELO_PIO_ID, 4);
	
	pio_handler_set(BUTAZUL_PIO, BUTAZUL_PIO_ID, BUTAZUL_IDX_MASK, PIO_IT_RISE_EDGE, but4_callback);
	pio_enable_interrupt(BUTAZUL_PIO, BUTAZUL_IDX_MASK);
	pio_get_interrupt_status(BUTAZUL_PIO);
	NVIC_EnableIRQ(BUTAZUL_PIO_ID);
	NVIC_SetPriority(BUTAZUL_PIO_ID, 4);
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
	/* Already the case in IAR's Normal DLIB default configuration: //printf()
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

		button1 = '0';

		if(xQueueReceive(xQueue, &button1, 0) == pdTRUE) {
			//printf("Botão: %c \n", button1);
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
