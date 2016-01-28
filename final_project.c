/*
  Steadily blinks an LED connected between Arduino Uno's IO9 and GND
*/

#include <avr/io.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#define BLINK_TASK_PRIORITY			(tskIDLE_PRIORITY + 1)
#define BLINK_LED				(_BV(PORTB1)) /* IO9 */
#define BUTTON					(_BV(PORTB2)) /* io10 */

static SemaphoreHandle_t mxpcr;
static SemaphoreHandle_t btnsh;

static int delay_value=255;// led look delay value

static QueueHandle_t qh;

// this semaphore protects the updating of gpio lines
static int mxpcr_init(void)
{
	mxpcr = xSemaphoreCreateMutex();
	return (mxpcr != NULL) ? 0 : -1;
}

static int mxpcr_lock(void)
{
	BaseType_t ret;
      
	ret = xSemaphoreTake(mxpcr, portMAX_DELAY);
	return (ret == pdPASS) ? 0 : -1;
}

static int mxpcr_unlock(void)
{
	BaseType_t ret;

	ret = xSemaphoreGive(mxpcr);
	return (ret == pdPASS) ? 0 : -1;
}
// button lock. this semaphore is set when the button is pressed. 
// it is designed to prevent the blink rate from being changed
static int btn_lock(void)
{
	BaseType_t ret;
	
	ret = xSemaphoreTake(btnsh, portMAX_DELAY);
	return (ret == pdPASS) ? 0 : -1;
}

static int btn_unlock(void)
{
	BaseType_t ret;

	ret = xSemaphoreGive(btnsh);
	return (ret == pdPASS) ? 0 : -1;
}
static const inline TickType_t ms_to_ticks(int ms)
{
	return ms / portTICK_PERIOD_MS;
}
// set the led and button port directions. LED is out, button is in
static void setup_main_led(void)
{
	mxpcr_lock();
	{
		DDRB |= BLINK_LED;		// make port output
		PORTB |= BLINK_LED;
		DDRB &= ~BUTTON;			// make port input
	}
	mxpcr_unlock();
}
// initialize the adc process
static void ADC_Init()
{
	
	mxpcr_lock();
	ADCSRA |= 1<<ADPS2;	// prescalar = 16
	ADMUX |= 1<<ADLAR;	// use ADCH to store upper 8 bits
	ADMUX |= 1<<REFS0;	// voltage reference
	ADCSRA |= 1<<ADEN;	// enable the adc
	//	ADCSRA |= 1<<ADIE;	// enable interrupts on the adc
	//	sei();				// enable interrupts
	ADCSRA |= 1<<ADSC;	// start the conversion
	mxpcr_unlock();
}
// toggle the LED on and off
static void toggle_main_led(void)
{
	mxpcr_lock();
	{
		PORTB ^= BLINK_LED;
	}
	mxpcr_unlock();
}
// blink task, the LED is toggled on and off after a delay value.
// the delay value is controlled by the potentiometer
static void blink_task(void *args)
{
	mxpcr_init();
	
	for (;;)
	{
		vTaskDelay(ms_to_ticks(delay_value));
		toggle_main_led();
	}
}
// get the next potentiometer value. The task get the adc value, then
// adds the value to a queue. THe queue is used to buffer updates
// to the LED blink rate
static void ADC_Task(void *args)
{
	int value;
	for (;;)
	{
		if (!(ADCSRA & _BV(ADSC))) {
			value=ADC;
			if (value>0)
				btn_lock();
				xQueueSend(qh, &value, 500);
				btn_unlock();
			ADCSRA |= 1<<ADSC;	// start the conversion
		}
		taskYIELD();
	}
}
// update the LED blick rate value from the queue
static void ADCR_Task(void *args)
{
	int *ptr_value=NULL;
	if (xQueueReceive(qh, ptr_value, 500)==pdPASS)
	{
		mxpcr_lock();
		delay_value=*ptr_value;
		mxpcr_unlock();
	}
	taskYIELD();
}
// button task. when the button is pressed, the pot values
// are prevented from getting posted to the queue. This 
// freezes the led blink rate
static void BUTTON_Task(void *args)
{
	btn_lock();
	while(bit_is_set(PINB, PB2))
		taskYIELD() ;
	btn_unlock();
	
}
void system_tick()
{
	
}
void vApplicationStackOverflowHook(void) 
{
	
}
int main(void)
{
	setup_main_led();
	ADC_Init();
	// create the button semaphore
	btnsh= xSemaphoreCreateMutex();
	qh=xQueueCreate(5, sizeof(int));
	// start the four tasks
	xTaskCreate(blink_task, "blink", configMINIMAL_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);
	xTaskCreate(ADC_Task, "ADC", configMINIMAL_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);
	xTaskCreate(ADCR_Task, "ADCR", configMINIMAL_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);
	xTaskCreate(BUTTON_Task, "button", configMINIMAL_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);
	
	vTaskStartScheduler(); /* never returns */

	return 0;
}

/* callback from FreeRTOS; can be disabled in config */
void vApplicationIdleHook (void)
{
}