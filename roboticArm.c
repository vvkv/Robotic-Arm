#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/qei.h"
#include "inc/hw_ints.h"

//input mechanisms for changing the input angle
#define PWM_FREQUENCY 55
//PID parameter definition
#define dt 0.00008
#define Kp 1 //1
#define Kd 0.02 //.01
#define Ki 0.005// 0.005
#define conversion 360/(6.3*2000);// 360/(6.3*2000) encoder ticks per 360 deg

float pre_error = 0;
float integral = 0;
volatile int32_t error;
float derivative;
float output;
float K;
float qeiDegPosition;
float qeiDegPosition2;


volatile int32_t encoderVal;
volatile uint32_t qeiPosition;
volatile uint32_t qeiPosition1;
volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile int32_t ui8Adjust;
volatile int32_t inputAngle;
volatile int32_t resetAngle = 0;
volatile int32_t pulseWidth;
int main(void)
	{
	ui8Adjust = 83;

	//input mechanisms for changing the input angle


	ROM_SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);


	ROM_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
	ROM_GPIOPinConfigure(GPIO_PE4_M1PWM2);
	//
	ROM_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);
	ROM_GPIOPinConfigure(GPIO_PE5_M1PWM3);
	//
	ui32PWMClock = SysCtlClockGet() / 64;
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ui32Load);
	//
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust * ui32Load / 1000);
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui8Adjust * ui32Load / 1000);

	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_1);

	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	ROM_GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
	ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);


	//PID

	//Encoder
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // Enable QEI Peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
	GPIOPinConfigure(GPIO_PC5_PHA1);
	GPIOPinConfigure(GPIO_PC6_PHB1);
	GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);
	//QEIDisable(QEI1_BASE); //ameer disabled this command.. useless?
	//QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER |QEI_INTINDEX); //ameer remove the disable of interrupts
	QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 50399);
	QEIEnable(QEI1_BASE);
	QEIPositionSet(QEI1_BASE, 25199);
	//ui8Adjust = 0;
	qeiPosition1 = 0;
   // inputAngle = 30;
    volatile int32_t newAngle = 90;
    volatile int32_t actualError;
    int32_t step1 = 0;
    int32_t step2 = 0;
    int32_t ANGLE = 0;
    volatile int32_t tester;
    resetAngle = 1;
    ANGLE = 90;//input your degrees here
    step1 = ANGLE*35;
	step2 = step1+25199;
	inputAngle=step2;
	while(1){
/*here we are reading the user input buttons on the Tiva, and incrementing...
  or decrementing based on the button pressed*/
	//if (resetAngle == 1){
	//	inputAngle = newAngle;
	//	resetAngle = 0;
	//	}
	// newAngle = 0;
	qeiPosition = QEIPositionGet(QEI1_BASE);
	//qeiDegPosition = (qeiP osition % 12599)*conversion; //this is for degree control.. not encoder control.
	error =  inputAngle - qeiPosition; //input angle is travel in what direction by how much
	//negative duty is clockwise rotation, since duty = current - desired position

	if (error > 20 || error < -20)// 0.5 -0.5 old values
	{	derivative = (error - pre_error)/dt;
		K = (Kp * error)  + (Ki * integral) + (Kd * derivative);

			if (K > 0)//check poaitive rotation
					//we are on the negative side of our target location, requiring a positive rotation
				{
						//shutting off the negative rotation PWM channel
				ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 4);
						//shutting off the negative rotation PWM channel

				if(K > 700)// old = 2... ~about half a turn away ... is it large enough? full power..
				{
					//ui8Adjust = 900;
					ui8Adjust = 40*K*conversion;//added the divby number
					pulseWidth = ui8Adjust * ui32Load / 1000;
					//pulseWidth = ui8Adjust * ui32Load / 100;
					ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, pulseWidth);
				}
				else
				{
					ui8Adjust = (40)*K*conversion;//added the divby number
					pulseWidth = ui8Adjust * ui32Load / 1000;
					ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, pulseWidth);
					integral += (error * dt);
				}

				//SysCtlDelay(100000);
				}
			else if (K < 0) // we are on the positive side of our target location, requiring a negative rotation
			{
				ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 4);
				if(K < -700)//old value = 1...is the error less than -400? full power
				{
					//ui8Adjust = 900;
					ui8Adjust = (-40)*K*conversion;//added the divby number
					pulseWidth = ui8Adjust * ui32Load / 1000;
					//pulseWidth = ui8Adjust * ui32Load / 100;
					ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, pulseWidth);
				}
				else
				{
					ui8Adjust = (-40)*K*conversion;//added 35 since 12599 -->0 is new scale, which is 35* larger than 0-360
					pulseWidth = ui8Adjust * ui32Load / 1000;
					ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, pulseWidth);
					integral += (error * dt);
				}
			}
			pre_error = error;
		}
		else  //we are at our target location, do not move in any direction!
		{
			pulseWidth = 1;
			ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, pulseWidth);
			ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, pulseWidth);

			if(ROM_GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00){
						ROM_SysCtlDelay(2000000);
						newAngle = 45;
						step1 = newAngle*35;
						step2 = step1;
						inputAngle += step2;
						resetAngle = 1;
					}
			if(ROM_GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)==0x00){
						ROM_SysCtlDelay(2000000);
						newAngle = -45;
						step1 = newAngle*35;
						step2 = step1;
						inputAngle += step2;
						resetAngle = 1;
					}
		}

		//SysCtlDelay(10);
	}
}