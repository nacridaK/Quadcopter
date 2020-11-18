/**
* Pozyx_core.cpp
* --------------
* This file contains the defintion of the core POZYX functions and variables
*
*/

#include "Pozyx.h"

#if defined(__SAM3X8E__)
// Arduino Due
#define Wire Wire1
#endif


extern "C" {
  #include "Pozyx_definitions.h"
}

/**
 * The interrupt handler for the pozyx device: keeping it uber short!
 */
void PozyxClass::IRQ()
{
  _interrupt = 1;
}

/**
 * Wirelessly write a number of bytes to a specified register address on a remote Pozyx device using UWB.
 */
int PozyxClass::remoteRegWrite(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size)
{
  // some checks
  if(!IS_REG_WRITABLE(reg_address))      return POZYX_FAILURE;    // the register is not writable
  if(size > MAX_BUF_SIZE-1)              return POZYX_FAILURE;    // trying to write too much data

  int status = 0;

  // first prepare the packet to send
  uint8_t tmp_data[size+1];
  tmp_data[0] = 0;
  tmp_data[1] = reg_address;              // the first byte is the register address we want to start writing to.
  memcpy(tmp_data+2, pData, size);         // the remaining bytes are the data bytes to be written starting at the register address.
  status = regFunction(POZYX_TX_DATA, (uint8_t *)&tmp_data, size+2, NULL, 0);

  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
    return status;

  // send the packet
  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x04;    // flag to indicate a register write

  uint8_t int_status = 0;
  regRead(POZYX_INT_STATUS, &int_status, 1);      // first clear out the interrupt status register by reading from it
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);

  if (waitForFlag_safe(POZYX_INT_STATUS_FUNC | POZYX_INT_STATUS_ERR, 100, &int_status)){
    if((int_status & POZYX_INT_STATUS_ERR) == POZYX_INT_STATUS_ERR)
    {
      // An error occured during positioning.
      // Please read out the register POZYX_ERRORCODE to obtain more information about the error
      return POZYX_FAILURE;
    }else{
      return POZYX_SUCCESS;
    }
  }else{
    return POZYX_TIMEOUT;
  }

  return status;
}

/**
 * Wirelessly read a number of bytes from a specified register address on a remote Pozyx device using UWB.
 */
int PozyxClass::remoteRegRead(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size)
{
  // some checks
  if(!IS_REG_READABLE(reg_address))      return POZYX_FAILURE;        // the register is not readable
  if(size > MAX_BUF_SIZE)                return POZYX_FAILURE;        // trying to read too much data
  if(destination == 0)                   return POZYX_FAILURE;        // remote read not allowed in broadcast mode

  int status = 0;

  // first prepare the packet to send
  uint8_t tmp_data[3];
  tmp_data[0] = 0;                  // the offset in the TX buffer
  tmp_data[1] = reg_address;        // the first byte is the register address we want to start reading from
  tmp_data[2] = size;               // the number of bytes to read starting from the register address
  status = regFunction(POZYX_TX_DATA, (uint8_t *)&tmp_data, 3, NULL, 0);

  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
    return status;

  // send the packet
  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x02;    // flag to indicate a register read

  uint8_t int_status = 0;
  regRead(POZYX_INT_STATUS, &int_status, 1);      // first clear out the interrupt status register by reading from it
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);

  // stop if POZYX_TX_SEND returned an error.
  if(status == POZYX_FAILURE)
    return status;

  // wait up to x ms to receive a response
  if(waitForFlag_safe(POZYX_INT_STATUS_FUNC | POZYX_INT_STATUS_ERR, 1000, &int_status))
  {
    if((int_status & POZYX_INT_STATUS_ERR) == POZYX_INT_STATUS_ERR)
    {
      // An error occured during positioning.
      // Please read out the register POZYX_ERRORCODE to obtain more information about the error
      return POZYX_FAILURE;
    }else{
      // we received a response, now get some information about the response
      uint8_t rx_info[3]= {0,0,0};
      regRead(POZYX_RX_NETWORK_ID, rx_info, 3);
      uint16_t remote_network_id = rx_info[0] + ((uint16_t)rx_info[1]<<8);
      uint8_t data_len = rx_info[2];

      if( remote_network_id == destination && data_len == size)
      {
        status = readRXBufferData(pData, size);
        return status;
      }else{
        return POZYX_FAILURE;
      }
    }

  }else{
    // timeout
    return POZYX_TIMEOUT;
  }
}

/*
 * Wirelessly call a register function with given parameters on a remote Pozyx device using UWB, the data from the function is stored in pData
 */
int PozyxClass::remoteRegFunction(uint16_t destination, uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size)
{
  // some checks
  if(!IS_FUNCTIONCALL(reg_address))      return POZYX_FAILURE;        // the register is not a function

  int status = 0;

  // first prepare the packet to send
  uint8_t tmp_data[param_size+2];
  tmp_data[0] = 0;
  tmp_data[1] = reg_address;                // the first byte is the function register address we want to call.
  memcpy(tmp_data+2, params, param_size);   // the remaining bytes are the parameter bytes for the function.
  status = regFunction(POZYX_TX_DATA, tmp_data, param_size+2, NULL, 0);

  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
  {
    return status;
  }

  // send the packet
  uint8_t tx_params[3];
  tx_params[0] = (uint8_t)destination;
  tx_params[1] = (uint8_t)(destination>>8);
  tx_params[2] = 0x08;    // flag to indicate a register function call
  uint8_t int_status = 0;
  regRead(POZYX_INT_STATUS, &int_status, 1);      // first clear out the interrupt status register by reading from it
  status = regFunction(POZYX_TX_SEND, tx_params, 3, NULL, 0);

  // stop if POZYX_TX_SEND returned an error.
  if(status == POZYX_FAILURE){
    return status;
  }

  // wait up to x ms to receive a response
  if(waitForFlag_safe(POZYX_INT_STATUS_FUNC | POZYX_INT_STATUS_ERR, 1000, &int_status))
  {
    if((int_status & POZYX_INT_STATUS_ERR) == POZYX_INT_STATUS_ERR)
    {
      return POZYX_FAILURE;
    }else
    {
      // we received a response, now get some information about the response
      uint8_t rx_info[3];
      regRead(POZYX_RX_NETWORK_ID, rx_info, 3);
      uint16_t remote_network_id = rx_info[0] + ((uint16_t)rx_info[1]<<8);
      uint8_t data_len = rx_info[2];

      if( remote_network_id == destination && data_len == size+1)
      {
        uint8_t return_data[size+1];

        status = readRXBufferData(return_data, size+1);

        if(status == POZYX_FAILURE){
          // debug information
          return status;
        }

        memcpy(pData, return_data+1, size);

        return return_data[0];
      }else{
        return POZYX_FAILURE;
      }
    }

  }else{
    // timeout
    return POZYX_TIMEOUT;
  }
}

int PozyxClass::sendTXBufferData(uint16_t destination)
{
  int status;

  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x06;
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);
  HAL_Delay(POZYX_DELAY_LOCAL_FUNCTION);

  return status;
}


/*
 * This function sends some data bytes to the destination
 */
int PozyxClass::sendData(uint16_t destination, uint8_t *pData, int size)
{
  if(size > MAX_BUF_SIZE)          return POZYX_FAILURE;        // trying to send too much data

  uint8_t status = 0;

  uint8_t tmp_data[size+1];
  tmp_data[0] = 0;                        // the first byte is the offset byte.
  memcpy(tmp_data+1, pData, size);

  // set the TX buffer
  status = regFunction(POZYX_TX_DATA, tmp_data, size+1, NULL, 0);

  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
    return status;

  // send the packet
  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x06;    // flag to indicate we're just sending data
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);

  return status;
}