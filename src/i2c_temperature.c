#ifndef TEST
#include "board.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
/* I2C master handle and memory for ROM API */
static I2C_HANDLE_T *i2cHandleMaster;

/* Use a buffer size larger than the expected return value of
 i2c_get_mem_size() for the static I2C handle type */
static uint32_t i2cMasterHandleMEM[0x20];

/* 100kbps I2C bit-rate */
#define I2C_BITRATE             (100000)

/** 7-bit and 10-bit I2C addresses */
#define I2C_ADDR_7BIT_1     (0x48 << 1) //thermal device
#define tempNumber 10   //how much temperature results behave

static volatile int intErrCode;

/* SysTick rate in Hz */
#define TICKRATE_HZ (10)
/* LED NUMBERS */
#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2
/* Current state for LED control via I2C cases */
static volatile int state = 0;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Initializes pin muxing for I2C interface - note that SystemInit() may
 already setup your pin muxing at system startup */
static void Init_I2C_PinMux(void) {
	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

	/* Connect the I2C_SDA and I2C_SCL signals to port pins(P0.10, P0.11) */
	Chip_SWM_MovablePinAssign(SWM_I2C_SDA_IO, 10);
	Chip_SWM_MovablePinAssign(SWM_I2C_SCL_IO, 11);

	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
}

/* Turn on LED to indicate an error */
static void errorI2C(void) {
	Board_LED_Set(0, true);
	while (1) {
	}
}

/* Setup I2C handle and parameters */
static void setupI2CMaster() {
	/* Enable I2C clock and reset I2C peripheral - the boot ROM does not
	 do this */
	Chip_I2C_Init(LPC_I2C);

	/* Perform a sanity check on the storage allocation */
	if (LPC_I2CD_API->i2c_get_mem_size() > sizeof(i2cMasterHandleMEM)) {
		/* Example only: this should never happen and probably isn't needed for
		 most I2C code. */
		errorI2C();
	}

	/* Setup the I2C handle */
	i2cHandleMaster = LPC_I2CD_API->i2c_setup(LPC_I2C_BASE, i2cMasterHandleMEM);
	if (i2cHandleMaster == NULL) {
		errorI2C();
	}

	/* Set I2C bitrate */
	if (LPC_I2CD_API->i2c_set_bitrate(i2cHandleMaster,
			Chip_Clock_GetSystemClockRate(),
			I2C_BITRATE) != LPC_OK) {
		errorI2C();
	}
}

/* I2C interrupt callback, called on completion of I2C operation when in
 interrupt mode. Called in interrupt context. */
static void cbI2CComplete(uint32_t err_code, uint32_t n) {
	intErrCode = (int) err_code;
}

/* Master transmit in interrupt mode */
static void sendI2CMaster(uint16_t AddressI2C, bool ledStateOut,
		uint8_t regAddr, uint8_t messageByte) {
	uint8_t SendData[10];
	I2C_PARAM_T param;
	I2C_RESULT_T result;
	ErrorCode_t error_code;
	int index = 0;

	/* 7-bit address */
	SendData[index++] = (uint8_t) AddressI2C;
	/* I2C device regAddr */
	SendData[index++] = regAddr;
	/* I2C device regVal */
	SendData[index++] = messageByte;

	/* Setup I2C parameters for number of bytes with stop - appears as follows on bus:
	 Start - address7 or address10upper - ack
	 (10 bits addressing only) address10lower - ack
	 value 1 - ack
	 value 2 - ack - stop */
	param.num_bytes_send = index;
	param.buffer_ptr_send = &SendData[0];
	param.num_bytes_rec = 0;
	param.stop_flag = 1;
	param.func_pt = cbI2CComplete;

	/* Set timeout (much) greater than the transfer length */
	LPC_I2CD_API->i2c_set_timeout(i2cHandleMaster, 100000);

	/* Do master write transfer */
	intErrCode = -1;

	/* Function is non-blocking, returned error should be LPC_OK, but isn't checked here */
	error_code = LPC_I2CD_API->i2c_master_transmit_intr(i2cHandleMaster, &param,
			&result);

	/* Sleep until transfer is complete, but allow IRQ to wake system
	 to handle I2C IRQ */
	while (intErrCode == -1) {
		__WFI();
	}

	/* Cast saved error code from callback */
	error_code = (ErrorCode_t) intErrCode;

	/* Completed without erors? */
	if (error_code != LPC_OK) {
		errorI2C();
	}

	/* Note results are only valid when there are no errors */
}

/* Master receive in interrupt mode */
static void readI2CMaster(uint16_t AddressI2C, bool * ledStateIn,
		uint8_t * temp) {
	uint8_t recvData[10] = { 0 };
	I2C_PARAM_T param;
	I2C_RESULT_T result;
	ErrorCode_t error_code;
	int index = 0;

	/* 7-bit address */
	recvData[index++] = (uint8_t) AddressI2C;

	/* Setup I2C paameters for number of bytes with stop - appears as follows on bus:
	 Start - address7 or address10upper - ack
	 (10 bits addressing only) address10lower - ack
	 value 1 (read) - ack
	 value 2 read) - ack - stop */
	param.num_bytes_rec = 1;
	param.buffer_ptr_rec = &recvData[0];
	param.stop_flag = 1;
	param.func_pt = cbI2CComplete;

	/* Set timeout (much) greater than the transfer length */
	LPC_I2CD_API->i2c_set_timeout(i2cHandleMaster, 100000);

	/* Do master read transfer */
	intErrCode = -1;

	/* Function is non-blocking, returned error should be LPC_OK, but isn't checked here */
	error_code = LPC_I2CD_API->i2c_master_receive_intr(i2cHandleMaster, &param,
			&result);

	/* Sleep until transfer is complete, but allow IRQ to wake system
	 to handle I2C IRQ */
	while (intErrCode == -1) {
		__WFI();
	}

	/* Completed without erors? */
	if (error_code != LPC_OK) {
		errorI2C();
	}

	/* Note results are only valid when there are no errors */
	*temp = recvData[1];
	*ledStateIn = (bool) recvData[1];
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	I2C interrupt handler
 * @return	Nothing
 */
void I2C_IRQHandler(void) {
	/* Call I2C ISR function in ROM with the I2C handle */
	LPC_I2CD_API->i2c_isr_handler(i2cHandleMaster);
}

/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void SysTick_Handler(void) {
	static int ticks = 0;
	ticks++;
	if (ticks > TICKRATE_HZ) {
		ticks = 0;
		state ^= 1; /* Toggle the state of bit 1 */
	}
}

int main(void) {
	bool ledState = false;
	int lastState = -1;

	/* Generic Initialization */
	SystemCoreClockUpdate();
	Board_Init();

	/* Turn clock to switch matrix back off to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

//	Board_LED_Set(0, false);

	/* Setup I2C pin muxing */
	Init_I2C_PinMux();

	/* Allocate I2C handle, setup I2C rate, and initialize I2C
	 clocking */
	setupI2CMaster();

	/* Enable the interrupt for the I2C */
	NVIC_EnableIRQ(I2C_IRQn);

	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);
	sendI2CMaster(I2C_ADDR_7BIT_1, -1, 0x01, 0);

	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, 15);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, 16);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, 17);

	uint8_t temp;  //temperature
	uint8_t tempTab[tempNumber] = { 0 };   //array with temperatures
	int i = 0;
	int measurementCounter = 1;
	double average = 0;   //average
	double *averagePointer;
	averagePointer = &average;
	uint8_t measurementNumber = 0;

	while (1) {

		/* Sleep until a state change occurs in SysTick */
		while (lastState == state) {
			__WFI();
		}

		/* Handle states */
		switch (state) {
		case 0:
			/* Set LED state on slave device */
			sendI2CMaster(I2C_ADDR_7BIT_1, ledState, 0x00, 0);
			break;

		case 1:
		default:
			readI2CMaster(I2C_ADDR_7BIT_1, &ledState, &temp);

			/*if more measurements than the size of tempTab we must sign it*/
			if (i > tempNumber) {
				i = 0;
			}

			/*put value of temperature to tempTab*/
			tempTab[i] = temp;

			DEBUGOUT("temp= %d\n\n", temp);

			for (int x = 0; x < tempNumber; x++) {
				DEBUGOUT("Temptab[%d]= %d\n", x, tempTab[x]);
			}

			if (measurementCounter > tempNumber) {
				measurementCounter = tempNumber;
			}

			/*get average of tempTab*/
			getAverage(tempTab, measurementCounter, averagePointer);

			/*setting the weight bit on GPIO*/
			uint8_t bit2 = Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0, 17);
			uint8_t bit1 = Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0, 16);
			uint8_t bit0 = Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0, 15);

			/*get value of measurements that user want to calculate the average */
			measurementNumber = getMeasurementNumber(bit2,bit1,bit0);

			DEBUGOUT("ile pomiarow do sredniej= %d\n", measurementNumber);

			/*compare  tempTab average and user average*/
			getUserAverage(tempTab, averagePointer, measurementNumber,
					measurementCounter);

			i++;
			measurementCounter++;

			break;
		}

		lastState = state;
	}
}
#endif   //TEST
