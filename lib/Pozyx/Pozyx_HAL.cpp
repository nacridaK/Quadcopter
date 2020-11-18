#include "Pozyx.h"

I2C_HandleTypeDef *hi2c;
uint32_t Timeout;

bool PozyxClass::waitForFlag(uint8_t interrupt_flag, int timeout_ms, uint8_t* interrupt)
{
	long timer = HAL_GetTick();
	int status;

	// stay in this loop until the event interrupt flag is set or until the the timer runs out
	while (HAL_GetTick() - timer < timeout_ms)
	{
		// in polling mode, we insert a small delay such that we don't swamp the i2c bus
		if (_mode == MODE_POLLING)
			HAL_Delay(1);
		if ((_interrupt == 1) || (_mode == MODE_POLLING))
		{
			_interrupt = 0;
			// Read out the interrupt status register. After reading from this register, pozyx automatically clears the interrupt flags.
			uint8_t interrupt_status = 0;
			status = regRead(POZYX_INT_STATUS, &interrupt_status, 1);
			if ((interrupt_status & interrupt_flag) && status == POZYX_SUCCESS)
			{
				// one of the interrupts we were waiting for arrived!
				if (interrupt != NULL)
					*interrupt = interrupt_status;
				return true;
			}
		}
	}
	// too bad, pozyx didn't respond
	// 1) pozyx can select from two pins to generate interrupts, make sure the correct pin is connected with the attachInterrupt() function.
	// 2) make sure the interrupt we are waiting for is enabled in the POZYX_INT_MASK register)
	return false;
}

bool PozyxClass::waitForFlag_safe(uint8_t interrupt_flag, int timeout_ms, uint8_t* interrupt)
{
	int tmp = _mode;
	_mode = MODE_POLLING;
	bool result = waitForFlag(interrupt_flag, timeout_ms, interrupt);
	_mode = tmp;
	return result;
}

int PozyxClass::begin(int mode, int interrupts, int interrupt_pin)
{
	int status = POZYX_SUCCESS;

	// check if the mode parameter is valid
	if ((mode != MODE_POLLING) && (mode != MODE_INTERRUPT))
		return POZYX_FAILURE;

	// check if the pin is valid
	if ((interrupt_pin != 0) && (interrupt_pin != 1))
		return POZYX_FAILURE;

	// wait a bit until the pozyx board is up and running
	HAL_Delay(250);

	_mode = mode;

	uint8_t whoami, selftest;
	uint8_t regs[3];
	regs[2] = 0x12;

	// we read out the first 3 register values: who_am_i, firmware_version and harware version, respectively.
	if (regRead(POZYX_WHO_AM_I, regs, 3) == POZYX_FAILURE)
		return POZYX_FAILURE;
	whoami = regs[0];
	_fw_version = regs[1];
	_hw_version = regs[2];

	// verify if the whoami is correct
	if (whoami != 0x43)
		// possibly the pozyx is not connected right. Also make sure the jumper of the boot pins is present.
		status = POZYX_FAILURE;

	// readout the selftest registers to validate the proper functioning of pozyx
	if (regRead(POZYX_ST_RESULT, &selftest, 1) == POZYX_FAILURE)
		return POZYX_FAILURE;

	if ((_hw_version & POZYX_TYPE) == POZYX_TAG)
	{
		// check if the uwb, pressure sensor, accelerometer, magnetometer and gyroscope are working
		if (selftest != 0b00111111)
			status = POZYX_FAILURE;
	}
	else if ((_hw_version & POZYX_TYPE) == POZYX_ANCHOR)
	{
		// check if the uwb transceiver and pressure sensor are working
		if (selftest != 0b00110000)
			status = POZYX_FAILURE;
		return status;
	}

	if (_mode == MODE_INTERRUPT)
	{
		//    // set the function that must be called upon an interrupt
		//    // put your main code here, to run repeatedly:
		//#if defined(__SAMD21G18A__) || defined(__ATSAMD21G18A__)
		//    // Arduino Tian
		//    int tian_interrupt_pin = interrupt_pin;
		//    attachInterrupt(interrupt_pin+2, IRQ, RISING);
		//#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
		//    // Arduino UNO, Mega
		//    attachInterrupt(interrupt_pin, IRQ, RISING);
		//#else
		//    Serial.println("This is not a board supported by Pozyx, interrupts may not work");
		//    attachInterrupt(interrupt_pin, IRQ, RISING);
		//#endif

		//    // use interrupt as provided and initiate the interrupt mask
		//    uint8_t int_mask = interrupts;
		//    configInterruptPin(5+interrupt_pin, PIN_MODE_PUSHPULL, PIN_ACTIVE_LOW, 0);

		//    if (regWrite(POZYX_INT_MASK, &int_mask, 1) == POZYX_FAILURE)
		//			return POZYX_FAILURE;
	}

	// all done
	HAL_Delay(POZYX_DELAY_LOCAL_WRITE);
	return status;
}

/**
  * Reads a number of bytes from the specified pozyx register address using I2C
  */
int PozyxClass::regRead(uint8_t reg_address, uint8_t *pData, int size)
{
	if(!IS_REG_READABLE(reg_address))
    return POZYX_FAILURE;
	//return !HAL_I2C_Mem_Read(hi2c, POZYX_I2C_ADDRESS << 1, reg_address, I2C_MEMADD_SIZE_8BIT, pData, size, timeout);
	switch (HAL_I2C_Mem_Read(hi2c, POZYX_I2C_ADDRESS << 1, reg_address, I2C_MEMADD_SIZE_8BIT, pData, size, timeout))
	{
		case HAL_OK:
			return 1;
		default:
			return 0;
	}
}

/**
  * Writes a number of bytes to the specified pozyx register address using I2C
  */
int PozyxClass::regWrite(uint8_t reg_address, uint8_t *pData, int size)
{
	if(!IS_REG_WRITABLE(reg_address))
		return POZYX_FAILURE;
	return !HAL_I2C_Mem_Write(hi2c, POZYX_I2C_ADDRESS << 1, reg_address, I2C_MEMADD_SIZE_8BIT, pData, size, timeout);
}
	
/**
  * Call a register function using i2c with given parameters, the data from the function is stored in pData
  */
int PozyxClass::regFunction(uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size)
{
	if(!IS_FUNCTIONCALL(reg_address))
		return POZYX_FAILURE;
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, POZYX_I2C_ADDRESS << 1, reg_address, I2C_MEMADD_SIZE_8BIT, params, param_size, timeout);
	if (status != HAL_OK)
		return POZYX_FAILURE;
  uint8_t read_data[size + 1];
	status = HAL_I2C_Master_Receive(hi2c, POZYX_I2C_ADDRESS << 1, read_data, size + 1, timeout);
	if (status != HAL_OK)
		return POZYX_FAILURE;
	memcpy(pData, read_data + 1, size);
	// the first byte that a function returns is always it's success indicator, so we simply pass this through
	return read_data[0];
}
	
int PozyxClass::writeTXBufferData(uint8_t data[], int size, int offset)
{
  if (offset + size > MAX_BUF_SIZE)
		return POZYX_FAILURE;

  int status = 1;
  uint8_t params[size + 1];
	
  // read out the received data.
  params[0] = offset;      // the offset
  memcpy(params + 1, data, size);
  status &= regFunction(POZYX_TX_DATA, params, size + 1, NULL, 0);

  return status;
}

int PozyxClass::readRXBufferData(uint8_t* pData, int size)
{
  if (size > MAX_BUF_SIZE)
		POZYX_FAILURE;
	
  int status;
  uint8_t params[] = {0, (uint8_t)size};

  // read out the received data.
	status = regFunction(POZYX_RX_DATA, params, 2, pData + params[0], params[1]);

  return status;
}

int PozyxClass::getDeviceIds(uint16_t devices[],int size, uint16_t remote_id)
{
  assert(size > 0);
  assert(size <= 20);

  // verify that the device list has at least the requested number of devices
  uint8_t list_size = 0;
  int status = getDeviceListSize(&list_size, remote_id);
  if(status != POZYX_SUCCESS || list_size < size)
    return POZYX_FAILURE;

  uint8_t params[2] = { 0, (uint8_t)size };

  return useFunction(POZYX_DEVICES_GETIDS, params, 2, (uint8_t *) devices, size * 2, remote_id);
}