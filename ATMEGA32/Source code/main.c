#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "common_macros.h"
#include "semphr.h"
#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

#define SENSOR_DRIVER_CHANNEL_ID PA1
#define SENSOR_PASSENGER_CHANNEL_ID PA2
#define SENSOR_MAX_VOLT_VALUE     1.5
#define SENSOR_MAX_TEMPERATURE    150
#define ADC_MAXIMUM_VALUE    1023
#define ADC_REF_VOLT_VALUE   5

#define DRIVER_PUSH_BUTTON PB3
#define PASSENGER_PUSH_BUTTON PB4

#define DRIVER_YELLOW_LED PD7
#define DRIVER_BLUE_LED PD6
#define DRIVER_GREEN_LED PD5

#define RED_LED PD4

#define PASSENGER_YELLOW_LED PC0
#define PASSENGER_BLUE_LED PC1
#define PASSENGER_GREEN_LED PC2

#define DRIVER_EVENT_BIT_BUTTON_PRESSED_OFF 	(1 << 0)
#define DRIVER_EVENT_BIT_BUTTON_PRESSED_LOW 	(1 << 1)
#define DRIVER_EVENT_BIT_BUTTON_PRESSED_MEDIUM 	(1 << 2)
#define DRIVER_EVENT_BIT_BUTTON_PRESSED_HIGHT 	(1 << 3)


#define PASSENGER_EVENT_BIT_BUTTON_PRESSED_OFF 		(1 << 4)
#define PASSENGER_EVENT_BIT_BUTTON_PRESSED_LOW 		(1 << 5)
#define PASSENGER_EVENT_BIT_BUTTON_PRESSED_MEDIUM 	(1 << 6)
#define PASSENGER_EVENT_BIT_BUTTON_PRESSED_HIGHT 	(1 << 7)

#define LOW_DESIRED_TEMP	25
#define MEDIUM_DESIRED_TEMP	30
#define HIGHT_DESIRED_TEMP	35


void ADC_Init(void);
void Hardware_init();
uint16_t ADC_Read(uint8_t channel);
uint8 LM35_getTemperature(uint8 channel);
void Buttons_init(void);
void Leds_init(void);

void DriverYellowLED_Off();
void DriverYellowLED_On();
void DriverBlueLED_On();
void DriverBlueLED_Off();
void DriverGreenLED_Off();
void DriverGreenLED_On();

void PassengerYellowLED_Off();
void PassengerYellowLED_On();
void PassengerBlueLED_On();
void PassengerBlueLED_Off();
void PassengerGreenLED_Off();
void PassengerGreenLED_On();

void RedLED_On();
void RedLED_Off();

void UART_sendChar(char data);
void UART_sendString(const char *str);
void UART_init(unsigned int ubrr) ;

int GetDriverButtonState();
int GetPassengerButtonState();

EventGroupHandle_t xEventGroup = NULL;
SemaphoreHandle_t Mutes;
int DriverSeatTemperature;
int PassengerSeatTemperature;


void PassengerButtonHandler(void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	int state;
	static int cnt=0;
	static int DriverButtonState=0;
    while (1)
    {

    	state = GetPassengerButtonState();
		if(state != DriverButtonState){
			DriverButtonState = state;
			if(DriverButtonState){
				cnt++;
				cnt %= 4;

				EventBits_t uxBits;
				uxBits = xEventGroupClearBits(
						xEventGroup,  /* The event group being updated. */
								0xff ); /* The bits being cleared. */

				PassengerGreenLED_Off();
				PassengerYellowLED_Off();
				PassengerBlueLED_Off();

				if(cnt==1)	xEventGroupSetBits(xEventGroup, PASSENGER_EVENT_BIT_BUTTON_PRESSED_LOW);
				else if(cnt==2) xEventGroupSetBits(xEventGroup, PASSENGER_EVENT_BIT_BUTTON_PRESSED_MEDIUM);
				else if(cnt==3)	xEventGroupSetBits(xEventGroup, PASSENGER_EVENT_BIT_BUTTON_PRESSED_HIGHT);
			}
		}
		vTaskDelayUntil(&xLastWakeTime, (100/portTICK_PERIOD_MS));
    }
}
void DriverButtonHandle(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int state;
    static int cnt=0;
    static int DriverButtonState=0;
    for(;;){
        state = GetDriverButtonState();
        if(state != DriverButtonState){
            DriverButtonState = state;
            if(DriverButtonState){
                cnt++;
                cnt %= 4;
                EventBits_t uxBits;
                uxBits = xEventGroupClearBits(
                		xEventGroup,  /* The event group being updated. */
                                0xff ); /* The bits being cleared. */
                DriverGreenLED_Off();
                DriverYellowLED_Off();
                DriverBlueLED_Off();

                if(cnt==1)	xEventGroupSetBits(xEventGroup, DRIVER_EVENT_BIT_BUTTON_PRESSED_LOW);
                else if(cnt==2)	xEventGroupSetBits(xEventGroup, DRIVER_EVENT_BIT_BUTTON_PRESSED_MEDIUM);
                else if(cnt==3)	xEventGroupSetBits(xEventGroup, DRIVER_EVENT_BIT_BUTTON_PRESSED_HIGHT);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, (100/portTICK_PERIOD_MS));
    }
}
void Temperature_Task(void *pvParameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	for(;;){
		DriverSeatTemperature=LM35_getTemperature(SENSOR_DRIVER_CHANNEL_ID);
		if(DriverSeatTemperature<5 || DriverSeatTemperature>40){
			UART_sendChar('a');
			RedLED_On();
		}else {
			RedLED_Off();
		}
		PassengerSeatTemperature =LM35_getTemperature(SENSOR_PASSENGER_CHANNEL_ID);
		if(PassengerSeatTemperature<5 || PassengerSeatTemperature>40){
			UART_sendChar('a');
			RedLED_On();
		}else {
			RedLED_Off();
		}

		vTaskDelayUntil(&xLastWakeTime, (100/portTICK_PERIOD_MS));
	}
}

void Heating_level_LOW(void *pvParameters){
    EventBits_t uxBits;
	for(;;){
		uxBits = xEventGroupWaitBits(
		                    xEventGroup,
							DRIVER_EVENT_BIT_BUTTON_PRESSED_LOW|PASSENGER_EVENT_BIT_BUTTON_PRESSED_LOW,
							pdTRUE,               // Clear the bits before returning.
		                    pdFALSE,
		                    portMAX_DELAY );
		if((uxBits & DRIVER_EVENT_BIT_BUTTON_PRESSED_LOW) && LOW_DESIRED_TEMP - DriverSeatTemperature >= 2){
			if(LOW_DESIRED_TEMP - DriverSeatTemperature<5)
				DriverYellowLED_On();
			else if(LOW_DESIRED_TEMP - DriverSeatTemperature<10)
				DriverBlueLED_On();
			else
				DriverGreenLED_On();
		}
		if((uxBits & PASSENGER_EVENT_BIT_BUTTON_PRESSED_LOW) && LOW_DESIRED_TEMP - PassengerSeatTemperature >= 2){
			if(LOW_DESIRED_TEMP - PassengerSeatTemperature<5)
				PassengerYellowLED_On();
			else if(LOW_DESIRED_TEMP - PassengerSeatTemperature<10)
				PassengerBlueLED_On();
			else
				PassengerGreenLED_On();

		}
	}
}
void Heating_level_MEDUIM(void *pvParameters){
    EventBits_t uxBits;
	for(;;){
		uxBits = xEventGroupWaitBits(
		                    xEventGroup,
							DRIVER_EVENT_BIT_BUTTON_PRESSED_MEDIUM | PASSENGER_EVENT_BIT_BUTTON_PRESSED_MEDIUM,
							pdTRUE,               // Clear the bits before returning.
		                    pdFALSE,
		                    portMAX_DELAY );
		if((uxBits & DRIVER_EVENT_BIT_BUTTON_PRESSED_MEDIUM) && MEDIUM_DESIRED_TEMP - DriverSeatTemperature >= 2){
			if(MEDIUM_DESIRED_TEMP - DriverSeatTemperature<5)
				DriverYellowLED_On();
			else if(MEDIUM_DESIRED_TEMP - DriverSeatTemperature<10)
				DriverBlueLED_On();
			else
				DriverGreenLED_On();
		}

		if((uxBits & PASSENGER_EVENT_BIT_BUTTON_PRESSED_MEDIUM) && MEDIUM_DESIRED_TEMP - PassengerSeatTemperature >= 2){
			if(MEDIUM_DESIRED_TEMP - PassengerSeatTemperature<5)
				PassengerYellowLED_On();
			else if(MEDIUM_DESIRED_TEMP - PassengerSeatTemperature<10)
				PassengerBlueLED_On();
			else
				PassengerGreenLED_On();
		}
	}
}
void Heating_level_HIGH(void *pvParameters){
    EventBits_t uxBits;
	for(;;){
		uxBits = xEventGroupWaitBits(
		                    xEventGroup,
							DRIVER_EVENT_BIT_BUTTON_PRESSED_HIGHT | PASSENGER_EVENT_BIT_BUTTON_PRESSED_HIGHT,
							pdTRUE,               // Clear the bits before returning.
		                    pdFALSE,
		                    portMAX_DELAY );
		if((uxBits & DRIVER_EVENT_BIT_BUTTON_PRESSED_HIGHT) && HIGHT_DESIRED_TEMP - DriverSeatTemperature >= 2){
			if(HIGHT_DESIRED_TEMP - DriverSeatTemperature<5)
				DriverYellowLED_On();
			else if(HIGHT_DESIRED_TEMP - DriverSeatTemperature<10)
				DriverBlueLED_On();
			else
				DriverGreenLED_On();
		}
		if((uxBits & PASSENGER_EVENT_BIT_BUTTON_PRESSED_HIGHT) && HIGHT_DESIRED_TEMP - PassengerSeatTemperature >= 2){
			if(HIGHT_DESIRED_TEMP - PassengerSeatTemperature<5)
				PassengerYellowLED_On();
			else if(HIGHT_DESIRED_TEMP - PassengerSeatTemperature<10)
				PassengerBlueLED_On();
			else
				PassengerGreenLED_On();
		}
	}
}



int main(void)
{
    Hardware_init();

    Mutes = xSemaphoreCreateMutex();
    xEventGroup = xEventGroupCreate();

    if(xEventGroup != NULL&&Mutes!=NULL) {

        xTaskCreate(PassengerButtonHandler, "LED Task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
        xTaskCreate(Heating_level_LOW, "vLED Task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
        xTaskCreate(DriverButtonHandle, "DriverButtonHandle Task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
        xTaskCreate(Heating_level_MEDUIM, "DriverButtonHandle Task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
        xTaskCreate(Heating_level_HIGH, "DriverButtonHandle Task", configMINIMAL_STACK_SIZE, NULL, 2	, NULL);
        xTaskCreate(Temperature_Task, "DriverButtonHandle Task", configMINIMAL_STACK_SIZE, NULL, 3, NULL);

        vTaskStartScheduler();
    }

    while (1) {}

    return 0;
}

void Hardware_init(){
    ADC_Init();
    Buttons_init();
    Leds_init();
    UART_init(MYUBRR);
}
int GetDriverButtonState(){
	return !!!(PINB&(1<<DRIVER_PUSH_BUTTON));
}
int GetPassengerButtonState(){
	return !!!(PINB&(1<<PASSENGER_PUSH_BUTTON));
}
void ADC_Init()
{
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

uint16_t ADC_Read(uint8_t channel)
{
    channel &= 0x07;
    ADMUX = (ADMUX & 0xF8) | channel;
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

uint8 LM35_getTemperature(uint8 channel)
{
    uint8 temp_value = 0;
    uint16 adc_value = 0;
    adc_value = ADC_Read(channel);
    temp_value = (uint8)(((uint32)adc_value*SENSOR_MAX_TEMPERATURE*ADC_REF_VOLT_VALUE)/(ADC_MAXIMUM_VALUE*SENSOR_MAX_VOLT_VALUE));
    return temp_value;
}

void Buttons_init(void){
    DDRB &=~((1<<DRIVER_PUSH_BUTTON)|(1<<PASSENGER_PUSH_BUTTON));
    PORTB |=(1<<DRIVER_PUSH_BUTTON)|(1<<PASSENGER_PUSH_BUTTON);
}

void Leds_init(void){
    DDRD|=(1<<DRIVER_YELLOW_LED)|(1<<DRIVER_BLUE_LED)|(1<<DRIVER_GREEN_LED)|(1<<RED_LED);
    DDRC |= (1 << PC0) | (1 << PC1) | (1 << PC2);

    PORTC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2));
}

void DriverYellowLED_On()
{
    PORTD |= (1 << DRIVER_YELLOW_LED);
}

void DriverYellowLED_Off()
{
    PORTD &= ~(1 << DRIVER_YELLOW_LED);
}

void DriverBlueLED_On()
{
    PORTD |= (1 << DRIVER_BLUE_LED);
}

void DriverBlueLED_Off()
{
    PORTD &= ~(1 << DRIVER_BLUE_LED);
}

void DriverGreenLED_On()
{
    PORTD |= (1 << DRIVER_GREEN_LED);
}

void DriverGreenLED_Off()
{
    PORTD &= ~(1 << DRIVER_GREEN_LED);
}
void RedLED_On(){
	PORTD |= (1 << RED_LED);
}
void RedLED_Off(){
	PORTD &= ~(1 << RED_LED);
}

void UART_init(unsigned int ubrr) {
    UBRRH = (unsigned char)(ubrr>>8);
    UBRRL = (unsigned char)ubrr;

    UCSRB = (1<<TXEN);

    UCSRC = (1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0);
}
void UART_sendChar(char data) {
    while (!( UCSRA & (1<<UDRE)));

    UDR = data;
}
void UART_sendString(const char *str) {
    while (*str) {
        UART_sendChar(*str++);
    }
}
void PassengerYellowLED_On(void) {
    PORTC |= (1 << PC0);  // Set PC0 high to turn on the Yellow LED
}

void PassengerYellowLED_Off(void) {
    PORTC &= ~(1 << PC0);  // Set PC0 low to turn off the Yellow LED
}

// Passenger Blue LED (connected to PC1)
void PassengerBlueLED_On(void) {
    PORTC |= (1 << PC1);  // Set PC1 high to turn on the Blue LED
}

void PassengerBlueLED_Off(void) {
    PORTC &= ~(1 << PC1);  // Set PC1 low to turn off the Blue LED
}

// Passenger Green LED (connected to PC2)
void PassengerGreenLED_On(void) {
    PORTC |= (1 << PC2);  // Set PC2 high to turn on the Green LED
}

void PassengerGreenLED_Off(void) {
    PORTC &= ~(1 << PC2);  // Set PC2 low to turn off the Green LED
}
