#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_rtc.h"
#include "em_lcd.h"
#include "em_timer.h"
#include "em_system.h"
#include "vddcheck.h"
#include "segmentlcd.h"
#include "bsp_trace.h"
#include "bsp.h"

#define RTC_FREQ    32768

/* Initial setup to 1:00 */
uint32_t minutes   = 0;
uint32_t seconds = 5;

//enable start
bool enable = false;

volatile uint32_t msTicks; /* counts 1ms timeTicks */

/* This flag enables/disables vboost on the LCD */
bool oldBoost = false;


/***************************************************************************//**
 * @brief GPIO Interrupt handler (PB9)
 *        Sets the hours
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  /* Acknowledge interrupt */
  GPIO_IntClear(1 << 9);

  /* Increase minutes */
  minutes = (minutes + 1) % 24;
}

/***************************************************************************//**
 * @brief GPIO Interrupt handler (PB10)
 *        Sets the minutes
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  /* Acknowledge interrupt */
  GPIO_IntClear(1 << 10);

  //set enable
  enable = !enable;
}

/***************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 ******************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}


/***************************************************************************//**
 * @brief Setup GPIO interrupt to set the time
 ******************************************************************************/
void gpioSetup(void)
{
  /* Enable GPIO in CMU */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure PB9 and PB10 as input */
  GPIO_PinModeSet(gpioPortB, 9, gpioModeInput, 0);
  GPIO_PinModeSet(gpioPortB, 10, gpioModeInput, 0);
  /* Configure PD15 as output */
  GPIO_PinModeSet(gpioPortD, 15, gpioModePushPull, 1);
  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 1);

  /* Set falling edge interrupt for both ports */
  GPIO_IntConfig(gpioPortB, 9, false, true, true);
  GPIO_IntConfig(gpioPortB, 10, false, true, true);

  /* Enable interrupt in core for even and odd gpio interrupts */
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);

  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

/***************************************************************************//**
 * @brief RTC Interrupt Handler.
 *        Updates minutes and seconds.
 ******************************************************************************/
void RTC_IRQHandler(void)
{
  /* Clear interrupt source */
  RTC_IntClear(RTC_IFC_COMP0);
  //if button PB1 was pressed start the timer
  if(enable){
	  //count down to 0:00
	  if(minutes != 0 || seconds != 0){
		  /* Decrease time by one minute */
		  if (seconds > 0) {
			  seconds--;
		  }
		  else{
			  seconds = 59;
			  minutes--;
		  }
	  }
	  //when time is up, lift the tea bag
	  else{
		  BSP_LedToggle(0);
		  BSP_LedToggle(1);
	  }
  }
}

/***************************************************************************//**
 * @brief Enables LFACLK and selects LFXO as clock source for RTC
 *        Sets up the RTC to generate an interrupt every minute.
 ******************************************************************************/
void rtcSetup(void)
{
  RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;

  /* Enable LE domain registers */
  CMU_ClockEnable(cmuClock_CORELE, true);

  /* Enable LFXO as LFACLK in CMU. This will also start LFXO */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

  /* Set a clock divisor of 32 to reduce power consumption. */
  CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_32);

  /* Enable RTC clock */
  CMU_ClockEnable(cmuClock_RTC, true);

  /* Initialize RTC */
  rtcInit.enable   = false;  /* Do not start RTC after initialization is complete. */
  rtcInit.debugRun = false;  /* Halt RTC when debugging. */
  rtcInit.comp0Top = true;   /* Wrap around on COMP0 match. */
  RTC_Init(&rtcInit);

  /* Interrupt every Second */
  RTC_CompareSet(0, ((RTC_FREQ / 32)) - 1);

  /* Enable interrupt */
  NVIC_EnableIRQ(RTC_IRQn);
  RTC_IntEnable(RTC_IEN_COMP0);

  /* Start Counter */
  RTC_Enable(true);
}

/***************************************************************************//**
 * @brief Check input voltage and enable vboost if it drops too low.
 ******************************************************************************/
void checkVoltage(void)
{
  bool vboost;

  /* Initialize voltage comparator */
  VDDCHECK_Init();

  /* Check if voltage is below 3V, if so use voltage boost */
  if (VDDCHECK_LowVoltage(2.9)) {
    vboost = true;
  } else {
    vboost = false;
  }

  /* Disable Voltage Comparator */
  VDDCHECK_Disable();

  if (vboost != oldBoost) {
    /* Reinitialize with new vboost setting */
    SegmentLCD_Init(vboost);
    /* Use Antenna symbol to signify enabling of vboost */
    SegmentLCD_Symbol(LCD_SYMBOL_ANT, vboost);
    oldBoost = vboost;
  }
}


/***************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 ******************************************************************************/
void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}

void tone(uint32_t delay, uint32_t duty_cycle,int duration){
	for(int i=0; i<duration;i++){
		GPIO_PinOutSet(gpioPortD,15);
		Delay(duty_cycle*delay);
		GPIO_PinOutClear(gpioPortD,15);
		Delay((1-duty_cycle)*delay);
	}
}


void mainLoop(void)
{

  /* Write Gecko and display, and light up the colon between minutes and seconds. */
  SegmentLCD_Symbol(LCD_SYMBOL_COL10, 1);
  SegmentLCD_Write("TEA-BOT");

  bool last_enable;

  while (1) {
	if(last_enable == false && enable == true){
		for(int i=0; i<10;i++){
			GPIO_PinOutSet(gpioPortD,3);
			Delay(2);
			GPIO_PinOutClear(gpioPortD,3);
			Delay(18);
		}
	}
	last_enable = enable;


    checkVoltage();
    SegmentLCD_Number(minutes * 100 + seconds);
    if(minutes == 0 && seconds == 0){
		for(int i=0; i<10;i++){
			GPIO_PinOutSet(gpioPortD,3);
			Delay(1.5);
			GPIO_PinOutClear(gpioPortD,3);
			Delay(18.5);
		}
		while(1){
			tone(1,0.75,500);
    		Delay(500);
    		if (!enable){
    			minutes = 1;
    			break;
    		}
    	}
    }
    EMU_EnterEM2(true);
  }
}

/***************************************************************************//**
 * @brief  Main function
 ******************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

  /* If first word of user data page is non-zero, enable Energy Profiler trace */
  BSP_TraceProfilerSetup();

  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) {
    while (1) ;
  }

  /* Ensure core frequency has been updated */
  SystemCoreClockUpdate();

  /* Init LCD with no voltage boost */
  SegmentLCD_Init(oldBoost);

  //Init LEDs
  BSP_LedsInit();
  BSP_LedSet(0);

  /* Setup RTC to generate an interrupt every minute */
  rtcSetup();

  /* Setup GPIO with interrupts to serve the pushbuttons */
  gpioSetup();

  /* Main function loop */
  mainLoop();

  return 0;
}
