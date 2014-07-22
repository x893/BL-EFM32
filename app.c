#include <stddef.h>

#include "demo.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_rtc.h"
#include "em_leuart.h"
#include "em_msc.h"

void ClockUpdate(void);
void RTC_Trigger(uint32_t msec, void(*funcPointer)(void));
uint16_t CRC_calc(uint8_t *start, uint8_t *end);

/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 *****************************************************************************/
volatile uint16_t delayTicks;
void SysTick_Handler(void)
{
	if (delayTicks)
		--delayTicks;
}

/**************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 *****************************************************************************/
void DelayStart(uint16_t dlyTicks)
{
	delayTicks = dlyTicks;
}

uint8_t DelayDone(void)
{
	return (delayTicks ? 0 : 1);
}

void Delay(uint16_t dlyTicks)
{
	DelayStart(dlyTicks);
	while (!DelayDone())
	{
	}
	// EMU_EnterEM1();
}

/**************************************************************************//**
 * @brief	Check host connected to RX pin
 *****************************************************************************/
uint8_t CheckHostConnect(void)
{
	uint8_t connected = 0;
	/* Enable GPIO for input with pull-down. RX is on D5 */
	GPIO_PinModeSet(HOST_RX, gpioModeInputPull, 0);
	Delay(10);
	if (GPIO_PinInGet(HOST_RX))
		connected = 1;
	GPIO_PinModeSet(HOST_RX, gpioModeDisabled, 0);
	return connected;
}

/**************************************************************************//**
 * @brief	Callback function for RTC
 *****************************************************************************/
volatile uint32_t Seconds;
void ClockUpdate(void)
{
	++Seconds;
	GPIO_PinOutToggle(LED1);
}

/**************************************************************************//**
 * @brief RTC Interrupt Handler, does nothing but clear the flag.
 *        The interrupt table is in assembly startup file startup_efm32.s
 *****************************************************************************/
void(*cb)(void) = NULL;
void RTC_IRQHandler(void)
{
	RTC->IFC = RTC_IFC_COMP0;	/* Clear interrupt source */
	if (cb != NULL)				/* Trigger callback */
		cb();
}

void RTC_Reset(void)
{
	/* Clear pending interrupts */
	RTC->IEN = 0;
	RTC->IFC = RTC_IFC_COMP0;
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
}

/**************************************************************************//**
 * @brief RTC trigger enable
 * @param msec Enable trigger in msec
 * @param fp Call this function if not NULL
 *****************************************************************************/
void RTC_Trigger(uint32_t msec, void(*funcPointer)(void))
{
	RTC_Reset();

	cb = funcPointer;			/* Register callback */

	RTC->COMP0 = (RTC_FREQ * msec) / 1000;	/* Calculate trigger value in ticks based on 32768Hz clock */
	RTC->CTRL = ~(RTC_CTRL_EN);	/* Restart counter */
	while (RTC->SYNCBUSY);		/* Wait until all registers are updated */

	NVIC_EnableIRQ(RTC_IRQn);	/* Enable interrupt */
	RTC->IEN = RTC_IEN_COMP0;

	/* Start Counter */
	RTC->CTRL = RTC_CTRL_EN | RTC_CTRL_COMP0TOP;	// | RTC_CTRL_DEBUGRUN
	while (RTC->SYNCBUSY);
}

void SendByte(uint8_t ch)
{
	LEUART_Tx(LEUART0, ch);
	while (!(LEUART0->STATUS & LEUART_STATUS_TXC))
	{
	}
}

void SendBytes(const uint8_t * bytes, uint8_t count)
{
	do
	{
		SendByte(*bytes++);
	} while (count-- != 0);
}

uint8_t RecvByte(uint8_t * ch)
{
	DelayStart(200);
	do
	{
		if (LEUART0->STATUS & LEUART_STATUS_RXDATAV)
		{
			*ch = (uint8_t)(LEUART0->RXDATA);
			return 1;
		}
	} while (!DelayDone());
	return 0;
}

/* Defining the LEUARTx initialization data */
const LEUART_Init_TypeDef leuartInit =
{
	leuartEnable,		/* Activate data reception on LEUn_RX pin. */
	RTC_FREQ,			/* Inherit the clock frequenzy from the LEUART clock source */
	9600,				/* Baudrate = 9600 bps */
	leuartDatabits8,	/* Each LEUART frame containes 8 databits */
	leuartEvenParity,	/* No parity bits in use */
	leuartStopbits1		/* Setting the number of stop bits in a frame to 2 bitperiods */
};

/**************************************************************************//**
 * @brief	Initialize LEUART
 *****************************************************************************/
void InitLEUART(void)
{
	CMU_TypeDef * cmu = CMU;
	LEUART_TypeDef * leu = LEUART0;

	GPIO_PinModeSet(HOST_RX, gpioModeInputPull, 1);
	GPIO_PinModeSet(HOST_TX, gpioModePushPull, 1);

	cmu->LFCLKSEL = (cmu->LFCLKSEL & ~(_CMU_LFCLKSEL_LFB_MASK)) | CMU_LFCLKSEL_LFB_LFXO;
	cmu->LFBCLKEN0 |= CMU_LFBCLKEN0_LEUART0;

	LEUART_Init(leu, &leuartInit);
	leu->ROUTE = LEUART_ROUTE_RXPEN | LEUART_ROUTE_TXPEN | LEUART_ROUTE_LOCATION_LOC0;

	Delay(500);
}

/**************************************************************************//**
 * @brief	Receive hex chars
 *****************************************************************************/
uint8_t PackCRC;
uint8_t RecvUx(uint32_t * value, uint8_t count)
{
	uint8_t ch;
	*value = 0;
	while (count-- != 0)
	{
		if (!RecvByte(&ch))
			return 0;
		PackCRC ^= ch;
		*value = (*value << 8) | ch;
	}
	return 1;
}

uint8_t RecvCRC(void)
{
	uint8_t ch;
	if (RecvByte(&ch))
	{
		PackCRC ^= ch;
		if (PackCRC == 0)
			return 1;
	}
	return 0;
}

uint8_t RecieveU8(uint8_t * value)
{
	uint32_t data;
	if (RecvUx(&data, 1))
	{
		*value = (uint8_t)data;
		return 1;
	}
	return 0;
}

uint8_t RecieveU16(uint16_t * value)
{
	uint32_t data;
	if (RecvUx(&data, 2))
	{
		*value = (uint16_t)data;
		return 1;
	}
	return 0;
}

uint8_t RecieveU32(uint32_t * value)
{
	uint32_t data;
	if (RecvUx(&data, 4))
	{
		*value = data;
		return 1;
	}
	return 0;
}

void LED1_OFF(void)
{
	GPIO_PinModeSet(LED1, gpioModeDisabled, 0);
}
void LED1_ON(void)
{
	GPIO_PinModeSet(LED1, gpioModeWiredAnd, 0);
}
void LED2_OFF(void)
{
	GPIO_PinModeSet(LED2, gpioModeDisabled, 0);
}
void LED2_ON(void)
{
	GPIO_PinModeSet(LED2, gpioModeWiredAnd, 0);
}

/**************************************************************************//**
 * @brief
 *   This function calculates the CRC-16-CCIT checksum of a memory range.
 *
 * @note
 *   This implementation uses an initial value of 0, while some implementations
 *   of CRC-16-CCIT uses an initial value of 0xFFFF. If you wish to
 *   precalculate the CRC before uploading the binary to the bootloader you
 *   can use this function. However, keep in mind that the 'v' and 'c' commands
 *   computes the crc of the entire flash, so any bytes not used by your
 *   application will have the value 0xFF.
 *
 * @param start
 *   Pointer to the start of the memory block
 *
 * @param end
 *   Pointer to the end of the block. This byte is not included in the computed
 *   CRC.
 *
 * @return
 *   The computed CRC value.
 *****************************************************************************/
uint16_t CRC_calc(uint8_t *start, uint8_t *end)
{
	register uint16_t crc = 0x0;
	while (start < end)
	{
		crc = (crc >> 8) | (crc << 8);
		crc ^= *start++;
		crc ^= (crc & 0xFF) >> 4;
		crc ^= crc << 12;
		crc ^= (crc & 0xFF) << 5;
	}
	return crc;
}

/**************************************************************************//**
 * @brief	Main function
 *****************************************************************************/
int main(void)
{
	uint8_t host_connect;
	/* Chip errata */
	// CHIP_Init();
	{
		CMU_TypeDef * cmu = CMU;
		uint32_t timeout;

		cmu->HFRCOCTRL = (cmu->HFRCOCTRL
			& ~(_CMU_HFRCOCTRL_BAND_MASK | _CMU_HFRCOCTRL_TUNING_MASK))
			| (cmuHFRCOBand_1MHz << _CMU_HFRCOCTRL_BAND_SHIFT)
			| (((DEVINFO->HFRCOCAL0 & _DEVINFO_HFRCOCAL0_BAND1_MASK) >> _DEVINFO_HFRCOCAL0_BAND1_SHIFT) << _CMU_HFRCOCTRL_TUNING_SHIFT);
		SystemCoreClock = 1000000;
		MSC->READCTRL = (MSC->READCTRL & ~_MSC_READCTRL_MODE_MASK) | MSC_READCTRL_MODE_WS0;

		cmu->HFCORECLKEN0 |= CMU_HFCORECLKEN0_LE;
		cmu->HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK4 | CMU_HFPERCLKDIV_HFPERCLKEN;
		cmu->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;
		GPIO_DriveModeSet(LEDx_PORT, gpioDriveModeHigh);

		cmu->CTRL = (cmu->CTRL & ~_CMU_CTRL_LFXOBOOST_MASK) | CMU_CTRL_LFXOBOOST_70PCENT;
		cmu->OSCENCMD = CMU_OSCENCMD_LFXOEN;
		timeout = 120000;
		while (!(cmu->STATUS & CMU_STATUS_LFXORDY) && timeout-- != 0)
			__NOP();

		if (cmu->STATUS & CMU_STATUS_LFXORDY)
		{
			cmu->LFCLKSEL = (cmu->LFCLKSEL & ~_CMU_LFCLKSEL_LFA_MASK) | CMU_LFCLKSEL_LFA_LFXO;
			cmu->OSCENCMD = CMU_OSCENCMD_LFRCODIS;
		}
		else
		{	// LFXO not start, use internal low freq. oscilator
			cmu->OSCENCMD = (CMU_OSCENCMD_LFXODIS | CMU_OSCENCMD_LFRCOEN);
			timeout = 120000;
			while (!(cmu->STATUS & CMU_STATUS_LFRCORDY) && timeout-- != 0)
				__NOP();
			cmu->LFCLKSEL = (cmu->LFCLKSEL & ~_CMU_LFCLKSEL_LFA_MASK) | CMU_LFCLKSEL_LFA_LFRCO;
			LED2_ON();
		}

		cmu->LFACLKEN0 |= CMU_LFACLKEN0_RTC;

		LED1_ON();
		if (cmu->STATUS & CMU_STATUS_LFXORDY)
			LED2_OFF();
		else
			LED2_ON();
	}

	Seconds = 0;
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		LED1_OFF();
		while (1)
		{
			EMU_EnterEM2(false);
		}
	}

	RTC_Trigger(5000, ClockUpdate);

	host_connect = CheckHostConnect();
	if (host_connect)
		InitLEUART();

	while (1)
	{
	}
}

/**************************************************************************
 * @brief	Fatal error handler
 *************************************************************************/
__declspec(noreturn) void Fatal_Handler(void);
__attribute((alias("Fatal_Handler"))) void NMI_Handler(void);
__attribute((alias("Fatal_Handler"))) void HardFault_Handler(void);
__attribute((alias("Fatal_Handler"))) void MemManage_Handler(void);
__attribute((alias("Fatal_Handler"))) void BusFault_Handler(void);
__attribute((alias("Fatal_Handler"))) void UsageFault_Handler(void);
__attribute((alias("Fatal_Handler"))) void SVC_Handler(void);
__attribute((alias("Fatal_Handler"))) void DebugMon_Handler(void);
__attribute((alias("Fatal_Handler"))) void PendSV_Handler(void);

void Fatal_Handler(void)
{
	volatile uint32_t delay;
	__disable_irq();
	while (1)
	{
		LED1_ON();
		LED2_ON();
		for (delay = 250000; delay != 0; --delay)
			__NOP();
		LED1_OFF();
		LED2_OFF();
		for (delay = 1000000; delay != 0; --delay)
			__NOP();
	}
}
