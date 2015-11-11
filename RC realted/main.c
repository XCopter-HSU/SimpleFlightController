/**
 * Main file for RC-Receiver task testing
 * */


#include <stdio.h>
#include <system.h>
#include "includes.h"

#include <stdint.h>
#include "main.h"//contains all semaphores OS_EVENTs for external files // RC required
#include "RCreceiver.h" // RC required
#include "b_pwmdriver.h" // RC required

/* Definition of Task Stacks */
#define   TASK_STACKSIZE       2048
OS_STK    task1_stk[TASK_STACKSIZE];
OS_STK    task2_stk[TASK_STACKSIZE];

/* Definition of Task Priorities */

#define TASK1_PRIORITY      1
#define TASK2_PRIORITY      2

/* Definitio of OS_Queu*/
#define UART_Q_SIZE 16 // RC required
OS_EVENT* uartQsem; //message que for uart
void* uartQmessageTable[UART_Q_SIZE]; //uart queue table // RC required


/* Definition of Semaphores*/
OS_EVENT* rcTasksem; //message que for uart


/*global variables*/
INT8U dummy;
INT8U* err = &dummy;
uint16_t rcValue[8]; //contains rc commands will be updated by the receiver task // RC required


uint8_t contrain(uint8_t min, uint8_t max, uint8_t val)
{
	uint8_t res = val;
	res = res < min ? min : res;
	res = res > max ? max : res;
	return res;
}


/* Using the RC input signal, synchronized with the rc input*/
void task1(void* pdata)
{
	printf("Starting Flightcontrol task...\n");

	uint8_t pwmVal = 0;

	while (1)
	{
		OSSemPend(rcTasksem, 0, err);

		pwmVal = (rcValue[2] - 8800) * 0.03984375; // normalize channel data to Byte value
		pwmVal = contrain(8, 218, pwmVal);
		PWMDriver_setSignalWidth(pwmVal, PWM_1);
		PWMDriver_setSignalWidth(pwmVal, PWM_2);
		PWMDriver_setSignalWidth(pwmVal, PWM_3);
		PWMDriver_setSignalWidth(pwmVal, PWM_4);
		PWMDriver_setSignalWidth(pwmVal, PWM_5);
		PWMDriver_setSignalWidth(pwmVal, PWM_6);
		PWMDriver_setSignalWidth(pwmVal, PWM_7);
		PWMDriver_setSignalWidth(pwmVal, PWM_8);
//		printf("%d\n", pwmVal);

//		printf("SUMD-Frame done\n");
//		int i;
//		for(i = 0; i < SUMD_MAXCHAN; i++)
//		{
//			printf("%d\t", rcValue[i]);
//		}
//		printf("\n");
	}
}


/* Reading UART and parse a SUMD frame then opens a semaphore for other task */
void rcTask(void* pdata)
{
	int frameDone = 0; // RC required
	printf("Starting RC task...\n");
	while (1) // RC required, entspricht der grossen main task vom flightCtrl
	{
		frameDone = updateChannelsRC(); // RC required
		if (frameDone == 1) // RC required
		{
		  OSSemPost(rcTasksem);
		}
	}
}
/* The main function creates two task and starts multi-tasking */
int main(void)
{

	printf("Starting Program\n");
	uartQsem = OSQCreate((void*)&uartQmessageTable, UART_Q_SIZE); //create Message Que

	initRCreceiver(); // RC required
	PWMDriver_init(); // RC required

	rcTasksem = OSSemCreate(0);

  
	OSTaskCreateExt(task1,
                  NULL,
                  (void *)&task1_stk[TASK_STACKSIZE-1],
                  TASK1_PRIORITY,
                  TASK1_PRIORITY,
                  task1_stk,
                  TASK_STACKSIZE,
                  NULL,
                  0);
              
               
	OSTaskCreateExt(rcTask,
                  NULL,
                  (void *)&task2_stk[TASK_STACKSIZE-1],
                  TASK2_PRIORITY,
                  TASK2_PRIORITY,
                  task2_stk,
                  TASK_STACKSIZE,
                  NULL,
                  0);
  OSStart();
  return 0;
}
