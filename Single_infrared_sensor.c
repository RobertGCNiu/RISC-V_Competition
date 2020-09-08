
/******************************************************************************
 *
 * Minimal Application for starting developping an freeRTOS program  
 *
 ******************************************************************************/

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Freedom metal includes. */
#include <metal/machine.h>
#include <metal/machine/platform.h>
#include <metal/plic_driver.h>
#include <metal/encoding.h>
#include <metal/stdatomic.h>
#include <metal/pwm1.h>
#include <metal/gpio1.h>
#include <metal/Sifive.h>

/* Car includes. */
#include "pwm.h"
#include "init1.h"

#define	left_monitor	p1
#define	right_monitor	p2

#define	car_motor_right_forward		p15
#define	car_motor_right_back		p16
#define	car_motor_left_forward		p14
#define	car_motor_left_back			p13

#define PLIC_NUM_INTERRUPTS 52


// Structures for registering different interrupt handlers
// for different parts of the application.
typedef void (*function_ptr_t) (void);
function_ptr_t g_ext_interrupt_handlers[PLIC_NUM_INTERRUPTS];
// Instance data for the PLIC.
plic_instance_t g_plic;
QueueHandle_t Message_Queue;

uint32_t GPIO_SET(uint32_t pin_num,uint32_t pin_val,uint32_t pin_model);
void smartcar_init(void);
void tracking_car_control(void);
uint8_t read_pin_val();
void PWM_run(int speed);
void PWM_left(int speed);
void PWM_right(int speed);
void PWM_back(int speed);
volatile void wait_ms(uint64_t ms);
void motor_stop();
void motor_back();
void motor_forward();



/*定义小车的运行程序和相关变量*/
//=====================================================================================
//=====================================================================================
volatile void wait_ms(uint64_t ms)//busy wait for the specified time
{
  static const uint64_t ms_tick = RTC_FREQ/1000;
  volatile uint64_t * mtime  = (uint64_t*) (METAL_RISCV_CLINT0_2000000_BASE_ADDRESS + METAL_RISCV_CLINT0_MTIME);
  uint64_t then = (ms_tick * ms) + *mtime;
  while(*mtime<then);
}


// which pin, value, input or output
uint32_t GPIO_SET(uint32_t pin_num,uint32_t pin_val,uint32_t pin_model)
{
    uint32_t input_val;
    if(pin_model==output)
     {
 	    GPIO_REG(METAL_SIFIVE_GPIO0_OUTPUT_EN) |= (0x01<<pin_num);
        if(pin_val==1)
		{
	     GPIO_REG(METAL_SIFIVE_GPIO0_OUTPUT_VAL) |=(0x01<<pin_num);
		}
        if(pin_val==0)
		{
	     GPIO_REG(METAL_SIFIVE_GPIO0_OUTPUT_VAL) &=~(0x01<<pin_num);
		}
    }
    if(pin_model==input)
    {
	GPIO_REG(METAL_SIFIVE_GPIO0_INPUT_EN) |= (0x01<<pin_num);


		input_val=GPIO_REG(METAL_SIFIVE_GPIO0_INPUT_VAL) &(0x01<<pin_num);
		if(input_val!=0)
		{
			input_val=1;
		}else
		{
			input_val=0;
		}
		return	input_val;
    }
 return 0;
}

void motor_stop()
{

		GPIO_SET(car_motor_left_back		,0,output); //left back stops
		GPIO_SET(car_motor_left_forward		,0,output); //left forward stops
		
		GPIO_SET(car_motor_right_back		,0,output); //right back stops
		GPIO_SET(car_motor_right_forward	,0,output); //right forward stops
}

void motor_forward()
{
	
	GPIO_SET(car_motor_left_back		,0,output); //left back stops
	GPIO_SET(car_motor_left_forward		,1,output); //left forward runs
	
	GPIO_SET(car_motor_right_back		,0,output); //right back stops
	GPIO_SET(car_motor_right_forward	,1,output); //right forward runs

}


void motor_back()
{
	
	GPIO_SET(car_motor_left_back		,1,output); //left back runs
	GPIO_SET(car_motor_left_forward		,0,output); //left forward stops

	GPIO_SET(car_motor_right_back		,1,output); //right back runs
	GPIO_SET(car_motor_right_forward	,0,output); //right forward stops

}


void PWM_run(int speed)
{// car goes forward at a specified speed
	for (int i = 1; i <= 150; i++)
	{
		if (i > speed)
		{ //Forward pins high
			GPIO_SET(car_motor_left_back, 0, output);
			GPIO_SET(car_motor_left_forward, 1, output);
			GPIO_SET(car_motor_right_back, 0, output);
			GPIO_SET(car_motor_right_forward, 1, output); 
		}
		// else if (i>0.7*speed)
		// { //Forward pins high
		// 	GPIO_SET(car_motor_left_back, 0, output);
		// 	GPIO_SET(car_motor_left_forward, 0, output);
		// 	GPIO_SET(car_motor_right_back, 0, output);
		// 	GPIO_SET(car_motor_right_forward, 1, output); // forward pins high
		// }
		else
		{
			//All pins low
			GPIO_SET(car_motor_left_back, 0, output);
			GPIO_SET(car_motor_left_forward, 0, output);

			GPIO_SET(car_motor_right_back, 0, output);
			GPIO_SET(car_motor_right_forward, 0, output); // all pins low
			delay(100);
		}
	}
}
void PWM_right(int speed)
{// the right wheel moves at a specified speed
	for (int i = 1; i <= 150; i++)
	{
		if (i > speed)
		{
			GPIO_SET(car_motor_right_back, 0, output);
			GPIO_SET(car_motor_right_forward, 0, output); 
			GPIO_SET(car_motor_left_back, 1, output);
			GPIO_SET(car_motor_left_forward, 0, output);
		}
		else
		{
			GPIO_SET(car_motor_right_back, 0, output);
			GPIO_SET(car_motor_right_forward, 0, output); // all pins low
			GPIO_SET(car_motor_left_back, 0, output);
			GPIO_SET(car_motor_left_forward, 0, output);
			delay(100);
		}
	}
}
void PWM_left(int speed)
{// the left wheel moves at a specified speed
	for (int i = 1; i <= 150; i++)
	{
		if (i > speed)
		{
			GPIO_SET(car_motor_left_back, 0, output);
			GPIO_SET(car_motor_left_forward, 0, output); 
			GPIO_SET(car_motor_right_back, 1, output);
			GPIO_SET(car_motor_right_forward, 0, output);
		}
		else
		{
			GPIO_SET(car_motor_left_back, 0, output);
			GPIO_SET(car_motor_left_forward, 0, output); // left pins low
			GPIO_SET(car_motor_right_back, 0, output);
			GPIO_SET(car_motor_right_forward, 0, output);
			delay(100);
		}
	}
}
void PWM_back(int speed)
{// car goes back at a specified speed
	for (int i = 1; i <= 150; i++)
	{
		if (i > speed)
		{
			GPIO_SET(car_motor_left_back, 1, output);
			GPIO_SET(car_motor_left_forward, 0, output);

			GPIO_SET(car_motor_right_back, 1, output);
			GPIO_SET(car_motor_right_forward, 0, output); // back pins high
		}
		// else if (i > 0.9 * speed)
		// { 
		// 	GPIO_SET(car_motor_left_back, 1, output);
		// 	GPIO_SET(car_motor_left_forward, 0, output);
		// 	GPIO_SET(car_motor_right_back, 0, output);
		// 	GPIO_SET(car_motor_right_forward, 0, output);
		// }
		else
		{
			GPIO_SET(car_motor_left_back, 0, output);
			GPIO_SET(car_motor_left_forward, 0, output);

			GPIO_SET(car_motor_right_back, 0, output);
			GPIO_SET(car_motor_right_forward, 0, output); // all pins low
			delay(100);
		}
	}
}

void smartcar_init()
{   // set the infrared sensor pins
	
	GPIO_SET(left_monitor	,0,input);
	GPIO_SET(right_monitor	,0,input);
}

uint8_t read_pin_val(){
	// return the input values of sensors
	uint32_t val;
	uint32_t caseA=0x01<<p2;
	uint32_t caseB=0x01<<p1;
	val=GPIO_REG(METAL_SIFIVE_GPIO0_INPUT_VAL); // read the input values of sensors

	if (val==0x00){  
		return 00;
	}
	else if (val==caseA){
		return 01;
	}
	else if (val==caseB){
		return 10;
	}
	else
		return 11;
}

void tracking_car_control()
{	
	uint8_t msg;
	int state;
	if(Message_Queue!=NULL) // Create queue successfully
		{	
			
			if(xQueueReceive(Message_Queue,&msg,portMAX_DELAY))
			{	
				write(STDOUT_FILENO,"Receiving successes!\n",strlen("Receiving successes!\n"));
				if(msg==00)		 // if on the road, go forward
				{
					PWM_run(0);
					state = 1;
				}
				if(msg==10)      // left out, turn right
				{	
					state = 2;
					PWM_left(0);
				}
				if(msg==01)		// right out, turn left
				{	
					state = 3;
					PWM_right(0);
				}
				if((msg==11)&&(state == 1)) // completely out and went forward initially, go backward
				{	
					PWM_back(0);	
				}
			}
		}
		else{
			write(STDOUT_FILENO,"No queue!\n",strlen("No queue!\n"));
		}
}

//FreeRTOS part
//============================================================================//
//============================================================================//

TaskHandle_t StartTask_Handler;
TaskHandle_t CarTask_Handler;
TaskHandle_t InfraredTask_Handler;
TaskHandle_t PrintTask_Handler;


static void Print_task()
{
	const char * const message1 = "FreeRTOS Version 10.3.1\n";
	const char * const message2 = "Hi, Sifive!\n";
	write(STDOUT_FILENO,message1,strlen(message1)); 
	write(STDOUT_FILENO,message2,strlen(message2));

	vTaskDelete(NULL);
}

static void Car_task()
{	UBaseType_t mark2;
 	while (1)
	{
		tracking_car_control();
		vTaskDelay(1);
	}
}

static void infrared_task()
{
	uint8_t result;
	BaseType_t err;

	while(1)
	{ 
		if(Message_Queue!=NULL) // create queue successfully
        {
			result=read_pin_val(); // get the result of sensors
			err=xQueueSend(Message_Queue,&result,0); 
			if(err==errQUEUE_FULL)   // if queue is full
				{
					write(STDOUT_FILENO,"Queue is full. Transmitting fails!\n",strlen("Queue is full. Transmitting fails!\n"));
				}
			else
			{
				write(STDOUT_FILENO,"Sending successes!\n",strlen("Sending successes!\n"));
			}
		}
		else
		{
			write(STDOUT_FILENO,"Creating Queue task fails!\n",strlen("Creating Queue task fails!\n"));
		}
		vTaskDelay(1);
	}
}
	
static void Start_Task(void)
{
	taskENTER_CRITICAL();
    Message_Queue=xQueueCreate(4,sizeof(uint8_t));

	xTaskCreate(Print_task,"Print_Task",200,NULL,5,(TaskHandle_t*  )&PrintTask_Handler);
	xTaskCreate(infrared_task,"infrared",300,NULL,3,(TaskHandle_t*  )&InfraredTask_Handler);
	xTaskCreate(Car_task,"Car_Task",512,NULL,4,(TaskHandle_t*  )&CarTask_Handler);
	
	vTaskDelete(NULL);
	taskEXIT_CRITICAL();
}

int main()
{
	PLIC_init(&g_plic,
		METAL_RISCV_PLIC0_C000000_BASE_ADDRESS,
 	    PLIC_NUM_INTERRUPTS,
		METAL_RISCV_PLIC0_C000000_RISCV_MAX_PRIORITY);

  	smartcar_init();
    xTaskCreate(Start_Task,"Start_Task",300,NULL,1,(TaskHandle_t*  )&StartTask_Handler);
	vTaskStartScheduler();
	return 0;
}
