#include <stddef.h>
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_rtc.h"
#include "em_gpio.h"
#include "em_leuart.h"
#include "em_msc.h"
#include "demo.h"

#define BOOTLOADER_SIZE	0x800

#define BL_VERSION		0x20
#define BP_ACK			0x79
#define BP_NACK			0x1F

#define GET_CMD			0x00
#define GET_VERSION_CMD	0x01
#define GET_ID_CMD		0x02
#define READ_CMD		0x11
#define GO_CMD			0x21
#define WRITE_CMD		0x31
#define ERASE_CMD		0x43
#define ERASE_EX_CMD	0x44

void ClockUpdate(void);
void RTC_Trigger(uint32_t msec, void (*funcPointer)(void));
uint16_t CRC_calc(uint8_t *start, uint8_t *end);

void LED1_OFF(void)
{
	GPIO_PinModeSet(LED1, gpioModeDisabled, 0);
}
void LED1_ON(void)
{
	GPIO_PinModeSet(LED1, gpioModeWiredAndDrive, 0);
}
void LED2_OFF(void)
{
	GPIO_PinModeSet(LED2, gpioModeDisabled, 0);
}
void LED2_ON(void)
{
	GPIO_PinModeSet(LED2, gpioModeWiredAndDrive, 0);
}

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
		EMU_EnterEM1();
	}
}

/**************************************************************************//**
 * @brief	Check host connected to RX pin
 *****************************************************************************/
uint8_t CheckHostConnect(void)
{
	uint8_t connected = 0;
	/* Enable GPIO for input with pull-down. RX is on D5 */
	GPIO_PinModeSet(HOST_RX, gpioModeInputPull, 0);
	Delay(5);
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
	if (Seconds & 1)
		LED1_ON();
	else
		LED1_OFF();
}

/**************************************************************************//**
 * @brief RTC Interrupt Handler, does nothing but clear the flag.
 *        The interrupt table is in assembly startup file startup_efm32.s
 *****************************************************************************/
void (*cb)(void) = NULL;
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
void RTC_Trigger(uint32_t msec, void (*funcPointer)(void))
{
	RTC_Reset();

	cb = funcPointer;			/* Register callback */

	RTC->COMP0 = (RTC_FREQ * msec) / 1000;	/* Calculate trigger value in ticks based on 32768Hz clock */
	RTC->CTRL = ~(RTC_CTRL_EN);	/* Restart counter */
	while (RTC->SYNCBUSY) ;		/* Wait until all registers are updated */

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
	{ }
}

void SendBytes(const uint8_t * bytes, uint8_t count)
{
	do
	{
		SendByte(*bytes++);
	} while (count-- != 0);
}

void SendLenBytes(const uint8_t *bytes, uint8_t count)
{
	SendByte(BP_ACK);
	SendByte(count);
	SendBytes(bytes, count);
	SendByte(BP_ACK);
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

uint8_t MemBuffer[2048];

const uint8_t BP_GET[] = {BL_VERSION, 0x00, 0x01, 0x02, 0x11, 0x21, 0x31, 0x44}; //, 0x63, 0x73, 0x82, 0x92};
const uint8_t BP_GV[] = {BL_VERSION, 0x00, 0x00};
const uint8_t BP_GID[] = {0x05, 0x20};

const uint8_t BP_OPTIONS[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// ?
	0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF		// Write protection (bit per page)
	};

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
		crc  = (crc >> 8) | (crc << 8);
		crc ^= *start++;
		crc ^= (crc & 0xFF) >> 4;
		crc ^= crc << 12;
		crc ^= (crc & 0xFF) << 5;
	}
	return crc;
}

#if defined ( __CC_ARM   )
__ASM
void JumpTo(uint32_t sp, uint32_t pc)
{
	msr msp, r0
	msr psp, r0
	bx	r1
}
#else
__ramfunc __noreturn void BOOT_jump(uint32_t sp, uint32_t pc)
{
  (void) sp;
  (void) pc;
  /* Set new MSP, PSP based on SP (r0)*/
  __asm("msr msp, r0");
  __asm("msr psp, r0");

  /* Jump to PC (r1)*/
  __asm("mov pc, r1");
}
#endif

void ResetAll(void)
{
	CMU_TypeDef * cmu = CMU;

	__disable_irq();

	MSC_Deinit();
	LED1_OFF();
	LED2_OFF();

	/* Clear all interrupts set. */
	NVIC->ICER[0] = 0xFFFFFFFF;
	NVIC->ICER[1] = 0xFFFFFFFF;
	RTC->IEN   = _RTC_IEN_RESETVALUE;
	RTC->CTRL  = _RTC_CTRL_RESETVALUE;
	SysTick->CTRL  = 0;

	/* Disable RTC clock */
	cmu->LFACLKEN0 = _CMU_LFACLKEN0_RESETVALUE;
	cmu->LFCLKSEL  = _CMU_LFCLKSEL_RESETVALUE;

	/* Disable LFRCO/LFXO */
	cmu->OSCENCMD = CMU_OSCENCMD_LFRCODIS | CMU_OSCENCMD_LFXODIS;

	/* Disable LE interface */
	cmu->HFCORECLKEN0 = _CMU_HFCORECLKEN0_RESETVALUE;
}

void JumpToApplication(uint32_t address)
{
	uint32_t pc, sp;

	ResetAll();

	SCB->VTOR = (uint32_t)(FLASH_BASE + BOOTLOADER_SIZE);

	// Read new SP and PC from vector table
	sp = *((uint32_t *) (FLASH_BASE + BOOTLOADER_SIZE));
	if (address == 0xFFFFFFFF)
		pc = *((uint32_t *) (FLASH_BASE + BOOTLOADER_SIZE + 4));
	else
		pc = address;
	JumpTo(sp, pc);
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
	CMU_TypeDef * cmu = CMU;

	__disable_irq();
	__set_MSP(*(volatile uint32_t *)FLASH_BASE);

	ResetAll();
	cmu->HFRCOCTRL = (cmu->HFRCOCTRL & ~(_CMU_HFRCOCTRL_BAND_MASK | _CMU_HFRCOCTRL_TUNING_MASK))
					| (cmuHFRCOBand_1MHz << _CMU_HFRCOCTRL_BAND_SHIFT)
					| (((DEVINFO->HFRCOCAL0 & _DEVINFO_HFRCOCAL0_BAND1_MASK) >> _DEVINFO_HFRCOCAL0_BAND1_SHIFT) << _CMU_HFRCOCTRL_TUNING_SHIFT);
	MSC->READCTRL = (MSC->READCTRL & ~_MSC_READCTRL_MODE_MASK) | MSC_READCTRL_MODE_WS0;
	cmu->HFPERCLKDIV   = CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK4 | CMU_HFPERCLKDIV_HFPERCLKEN;
	cmu->HFPERCLKEN0  |= CMU_HFPERCLKEN0_GPIO;
	GPIO_DriveModeSet(LEDx_PORT, gpioDriveModeHigh);

	LED1_ON();
	LED2_ON();
	for (delay = 250000; delay != 0; --delay)
		__NOP();
	LED1_OFF();
	LED2_OFF();
	for (delay = 1000000; delay != 0; --delay)
		__NOP();
	NVIC_SystemReset();
	while (1)
	{ }
}

/**************************************************************************
 * @brief	Main function
 *************************************************************************/
int main(void)
{
	uint32_t seconds;
	uint32_t address;
	uint16_t count16;
	uint8_t count;
	uint8_t boot_ok;
	uint8_t boot_retry   = 0;
	uint8_t host_connect = 0;
	uint8_t ch;
	uint8_t cmd;

	/* Chip errata */
	// CHIP_Init();
	{
		CMU_TypeDef * cmu = CMU;
		uint32_t timeout;

		cmu->HFRCOCTRL = (cmu->HFRCOCTRL & ~(_CMU_HFRCOCTRL_BAND_MASK | _CMU_HFRCOCTRL_TUNING_MASK))
						| (cmuHFRCOBand_1MHz << _CMU_HFRCOCTRL_BAND_SHIFT)
						| (((DEVINFO->HFRCOCAL0 & _DEVINFO_HFRCOCAL0_BAND1_MASK) >> _DEVINFO_HFRCOCAL0_BAND1_SHIFT) << _CMU_HFRCOCTRL_TUNING_SHIFT);
		MSC->READCTRL = (MSC->READCTRL & ~_MSC_READCTRL_MODE_MASK) | MSC_READCTRL_MODE_WS0;
		SystemCoreClock =  1000000;

		cmu->CTRL     = (cmu->CTRL & ~_CMU_CTRL_LFXOBOOST_MASK) | CMU_CTRL_LFXOBOOST_70PCENT;
		cmu->OSCENCMD = CMU_OSCENCMD_LFXOEN;
		timeout = 120000;
		while (!(cmu->STATUS & CMU_STATUS_LFXORDY) && timeout-- != 0)
			__NOP();

		cmu->HFPERCLKDIV   = CMU_HFPERCLKDIV_HFPERCLKDIV_HFCLK4 | CMU_HFPERCLKDIV_HFPERCLKEN;
		cmu->HFPERCLKEN0  |= CMU_HFPERCLKEN0_GPIO;
		cmu->HFCORECLKEN0 |= CMU_HFCORECLKEN0_LE;

		GPIO_DriveModeSet(LEDx_PORT, gpioDriveModeHigh);
		
		if (cmu->STATUS & CMU_STATUS_LFXORDY)
		{
			cmu->LFCLKSEL = (cmu->LFCLKSEL & ~_CMU_LFCLKSEL_LFA_MASK) | CMU_LFCLKSEL_LFA_LFXO;
			cmu->OSCENCMD = CMU_OSCENCMD_LFRCODIS;
		}
		else
		{	// LFXO not start, use internal low freq. oscilator
			cmu->OSCENCMD = (CMU_OSCENCMD_LFXODIS | CMU_OSCENCMD_LFRCOEN);
			timeout = 200000;
			while (!(cmu->STATUS & CMU_STATUS_LFRCORDY) && timeout-- != 0)
				__NOP();
			cmu->LFCLKSEL = (cmu->LFCLKSEL & ~_CMU_LFCLKSEL_LFA_MASK) | CMU_LFCLKSEL_LFA_LFRCO;
			LED2_ON();
		}

		cmu->LFACLKEN0 |= CMU_LFACLKEN0_RTC;

		LED1_OFF();
		if (cmu->STATUS & CMU_STATUS_LFXORDY)
			LED2_OFF();
		else
			LED2_ON();
	}

	Seconds = 0;
	if (SysTick_Config(SystemCoreClock / 1000))
		Fatal_Handler();

	/*
	boot_ok = 0;
	// Check for boot new version valid (in upper flash)
	if (boot_ok)
	{
		// Copy new application to work area
	}
	*/

	host_connect = CheckHostConnect();

	boot_ok = 0;
	if (boot_ok)
	{
		if (host_connect)
			boot_retry = 10;
		else
			JumpToApplication(0xFFFFFFFF);
	}

	RTC_Trigger(1000, ClockUpdate);

	if (host_connect)
		InitLEUART();

	seconds = Seconds;
	MSC_Init();

	while (1)
	{
		if (seconds != Seconds)
		{
			seconds = Seconds;
			if (boot_retry != 0)
			{
				if (boot_retry == 1)
					JumpToApplication(0xFFFFFFFF);
				--boot_retry;
			}
			if (!host_connect && (host_connect = CheckHostConnect()) != 0)
				InitLEUART();
		}

		if (!host_connect)
		{
			EMU_EnterEM2(false);
			continue;
		}

		PackCRC = 0xFF;
		if (!RecieveU8(&cmd))
			continue;

		if (cmd == BP_ACK || cmd == BP_NACK)
			continue;
		if (cmd == 0x7F)
		{
			SendByte(BP_ACK);
			continue;
		}

		if (RecvCRC())
		{
			switch (cmd)
			{
				case GET_CMD:
					SendLenBytes(BP_GET, sizeof(BP_GET)-1);
					continue;
				case GET_VERSION_CMD:
					SendByte(BP_ACK);
					SendBytes(BP_GV, sizeof(BP_GV) - 1);
					SendByte(BP_ACK);
					continue;
				case GET_ID_CMD:
					SendLenBytes(BP_GID, sizeof(BP_GID)-1);
					continue;
				case GO_CMD:
					SendByte(BP_ACK);
					PackCRC = 0;
					if (RecieveU32(&address) && RecvCRC())
					{
						SendByte(BP_ACK);
#if (FLASH_BASE > 0)
						if (address < (FLASH_BASE + FLASH_SIZE) && address >= FLASH_BASE)
#else
						if (address < (FLASH_BASE + FLASH_SIZE))
#endif
							JumpToApplication(address);
					}					
					continue;

				case ERASE_EX_CMD:
					SendByte(BP_ACK);
					PackCRC = 0;
					if (RecieveU16(&count16))
					{
						if (count16 >= 0xFFF0)
						{	// Special Erase
							if (RecieveU8(&ch))
							{
								SendByte(BP_ACK);
								continue;
							}
						}
						else
						{
							uint16_t data;
							uint16_t * pages = (uint16_t *)(&MemBuffer[0]);
							*pages++ = count16;
							cmd = 1;
							do
							{
								if (!RecieveU16(&data))
								{
									cmd = 0;
									break;
								}
								*pages++ = data;
							} while (count16-- != 0);

							if (cmd && RecvCRC())
							{
								pages = (uint16_t *)(&MemBuffer[0]);
								count16 = *pages++;
								cmd = 1;
								__disable_irq();
								do
								{
									address = (((uint32_t)*pages++) * FLASH_PAGE_SIZE);
									if (mscReturnOk != MSC_ErasePage((uint32_t *)address))
									{
										cmd = 0;
										break;
									}
								} while (count16-- != 0);
								__enable_irq();
								if (cmd)
								{
									SendByte(BP_ACK);
									continue;
								}
							}
						}
					}
					break;
				case ERASE_CMD:
					SendByte(BP_ACK);
					PackCRC = 0;
					if (RecieveU8(&count))
					{
						if (count == 0xFF)
						{
							if (RecvByte(&ch) && ch == 0x00)
							{
								// Mass Erase
								SendByte(BP_ACK);
								continue;
							}
						}
						else if (count < ((FLASH_SIZE / FLASH_PAGE_SIZE) - 1))
						{
							uint8_t * pages = &MemBuffer[0];
							*pages++ = count;
							cmd = 1;
							do
							{
								if (!RecieveU8(&ch))
								{
									cmd = 0;
									break;
								}
								*pages++ = ch;
							} while (count-- != 0);
							if (cmd && RecvCRC())
							{
								pages = &MemBuffer[0];
								count = *pages++;
								cmd = 1;
								__disable_irq();
								do
								{
									address = (((uint32_t)(*pages++)) * FLASH_PAGE_SIZE);
									if (mscReturnOk != MSC_ErasePage((uint32_t *)address))
									{
										cmd = 0;
										break;
									}
								} while (count-- != 0);
								__enable_irq();
								if (cmd)
								{
									SendByte(BP_ACK);
									continue;
								}
							}
						}
					}
					break;
				case WRITE_CMD:
					SendByte(BP_ACK);
					PackCRC = 0;
					if (RecieveU32(&address) && RecvCRC())
					{
						SendByte(BP_ACK);
						PackCRC = 0;
						if (RecieveU8(&count))
						{
							uint8_t * dst = &MemBuffer[0];
							*dst++ = count;
							cmd = 1;
							do
							{
								if (!RecieveU8(&ch))
								{
									cmd = 0;
									break;
								}
								*dst++ = ch;
							} while (count-- != 0);
							if (cmd && RecvCRC())
							{
								dst = &MemBuffer[0];
								count16 = (uint16_t)(*dst++) + 1;
								if ((count16 & 0x03) == 0
								// &&	address >= FLASH_BASE
								&&	address < (FLASH_BASE + FLASH_SIZE)
								&&	(address + count16) < (FLASH_BASE + FLASH_SIZE)
									)
								{
									cmd = 1;
									__disable_irq();
									if (mscReturnOk != MSC_WriteWord((uint32_t *)address, dst, count16))
										cmd = 0;
									__enable_irq();
									if (cmd)
									{
										SendByte(BP_ACK);
										continue;
									}
								}
							}
						}
					}
					break;
				case READ_CMD:
					SendByte(BP_ACK);
					PackCRC = 0;
					if (RecieveU32(&address) && RecvCRC())
					{
						SendByte(BP_ACK);
						PackCRC = 0xFF;
						if (RecieveU8(&count) && RecvCRC())
						{
							SendByte(BP_ACK);
							if (address == 0x1FFFF7E0 && count == 1)
							{
								SendByte((uint8_t)((FLASH_SIZE / 1024) >> 0));
								SendByte((uint8_t)((FLASH_SIZE / 1024) >> 8));
								continue;
							}
							else if (address == 0x1FFFF7D6 && count == 0x01)
							{
								SendByte(BL_VERSION);
								SendByte(0x0);
								continue;
							}
							else if (address == 0x1FFFF800 && count == 0x0F)
							{
								SendBytes(BP_OPTIONS, count);
								continue;
							}
							else if ((address + (uint32_t)count + 1) < (FLASH_BASE + FLASH_SIZE)
							//	&&	address >= FLASH_BASE
									)
							{
								SendBytes((uint8_t *)address, count);
								continue;
							}
							else
							{
								do
								{
									SendByte(0xFF);
								} while (count-- != 0);
								continue;
							}
						}
					}
					break;
			}
		}
		SendByte(BP_NACK);
	}
}
