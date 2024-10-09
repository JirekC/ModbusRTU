/**
 * @file    mod_master_rtu.h
 * @mainpage
 * @brief   Simple implementation of Modbus RTU master stack. Does not need any RTOS only @ref ModMasterCheck()
 *          has to be periodically called until Modbus transaction (operation) is done.
 * @note    FreeRTOS included here only to define @ref ModbusMasterInternal group macros. Use any other time-reading API.
 *
Example of usage (STM32 HAL used):
@verbatim
modMasterStack_t mstack; // modbus stack

...

// callbacks from ISR / HW driver to ModMaster

void U2TxCallback(UART_HandleTypeDef *huart)
{
    ModMasterTxDoneCallback(&mstack);
}

void U2RxCallback(UART_HandleTypeDef *huart)
{
    uint32_t error = HAL_UART_GetError(huart);
    uint16_t len = 257 - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    if (error & HAL_UART_ERROR_RTO) {
        // RTO = complete message received
        ModMasterRxDoneCallback(&mstack, (uint8_t*)mstack.message, len);
    } else {
        // other error
        // OR no error, but full buffer (257 B) was received = overflow
        ModMasterRxErrorCallback(&mstack);
    }
    HAL_UART_AbortReceive(huart); // disable interrupts !
}

// callbacks from ModMaster

int16_t UartSendCb(const uint8_t* data, uint16_t length)
{
    SWITCH_TX;
    HAL_UART_Transmit_DMA(&huart2, data, length);
    return 0;
}

int16_t UartReceiveCb(void)
{
    HAL_UART_AbortReceive(&huart2);
    SWITCH_RX;
    // small hack - we can use message array directly by DMA
    // rx done callback will compare message printer with data param
    // and will avoid copy if pointers are same
    HAL_UART_Receive_DMA(&huart2, (uint8_t*)mstack.message, 257);
    return 0;
}

// initialisation procedure

void ModbusUARTInit(void)
{
    memset(&mstack, 0, sizeof(mstack));

    // register callbacks
    mstack.pfSend = &UartSendCb;
    mstack.pfReceive = &UartReceiveCb;

    ModMasterInit(&mstack);

    // HW init + register ISR callbacks
    HAL_UART_Init(&huart2);
    HAL_UART_RegisterCallback(&huart2, HAL_UART_RX_COMPLETE_CB_ID, U2RxCallback);
    HAL_UART_RegisterCallback(&huart2, HAL_UART_ERROR_CB_ID, U2RxCallback);
    HAL_UART_RegisterCallback(&huart2, HAL_UART_TX_COMPLETE_CB_ID, U2TxCallback);
    HAL_UART_EnableReceiverTimeout(&huart2);
    HAL_UART_ReceiverTimeout_Config(&huart2, 35); // 35 baud-blocks ~= 3.5 byte-times
    SWITCH_RX;
}

...

int main(void)
{
    // HW init

    ...

    ModbusUARTInit();

    ...

    // Stupid example - infinite loop of writing regs
    for(;;)
    {
        uint16_t regs[2] = {0xffff, 0xffff};
        ModMasterWriteRegs(&mstack, 1, 0xD2, 2, regs);
        uint8_t e;
        modMasterState_t s;
        while (!ModMasterCheck(&mstack, &s, &e)); // keep calling until operation finishes
        (void)e;
        if (s == eMOD_M_STATE_PROCESSED) {
          // TODO: consume response data
        }
    }
}
@endverbatim
 */

#ifndef SYSTEM_MOD_MASTER_RTU_H
#define SYSTEM_MOD_MASTER_RTU_H

#include <stdint.h>

/**
 * @defgroup TimeAPI Time API includes
 * It is here only for @ref ModbusMasterInternal. Feel free to change to any other API.
 * @{
 */
#include "FreeRTOS.h"
#include "task.h"
/** @} */

/**
 * @defgroup ModbusErrors Modbus error-report codes
 * @{
 */
#define MODBUS_ERR_ILLEGAL_OPCODE   0x01
#define MODBUS_ERR_ILLEGAL_ADDRESS  0x02
#define MODBUS_ERR_ILLEGAL_VALUE    0x03
#define MODBUS_ERR_DEVICE_FAULT     0x04
/** @} */

/**
 * @defgroup ModbusMasterInternal
 * @{
 */
#define MODBUS_RX_TIMEOUT           100     ///< in milliseconds
#define MODBUS_TIME_T               TickType_t
#define MODBUS_GET_TIME_MS          (xTaskGetTickCount() * (1000/configTICK_RATE_HZ))
#define MODBUS_GET_TIME_ISR_MS      (xTaskGetTickCountFromISR() * (1000/configTICK_RATE_HZ))
/** @} */

/** MODBUS engine status flags **/
typedef enum
{
    eMOD_M_STATE_STANDBY,
    eMOD_M_STATE_TRANSMITTING,
    eMOD_M_STATE_WAITING_ANSWER,
    eMOD_M_STATE_RECEIVED,
    eMOD_M_STATE_PROCESSING,
    eMOD_M_STATE_TIMED_OUT,
    eMOD_M_STATE_CORRUPTED,
    eMOD_M_STATE_ERR_REPORTED,
    eMOD_M_STATE_PROCESSED,
    eMOD_M_STATE_HW_ERROR
} modMasterState_t;

/**
 * @defgroup ModbusMasterCb Modbus master callbacks
 * @{
 */
/**
 * @brief   User will pass pointer to function that sends @b data which are @b length bytes long over the UART.
 * @note    After successfull Tx, user has to call @ref ModMasterTxDoneCallback().
 * @return  0 if everything OK, negative value in case of failure.
 */
typedef int16_t (*pfModMSend_t)(const uint8_t* data, uint16_t length);

/**
 * @brief   User will pass pointer to function that starts receiving data from UART.
 * @note    After successful receive of answer, user have to call @ref ModMasterRxDoneCallback().
 *          If error occurred @ref ModMasterRxErrorCallback() has to be called.
 * @warning This function is called by @ref ModMasterTxDoneCallback(), so can be called from ISR.
 * @return  0 if everything OK, negative value in case of failure.
 */
typedef int16_t (*pfModMReceive_t)(void);
/** @} */

/**
 * @brief   Modbus master stack structure. Pass pointer to this tructure to each ModMaster function.
 *          More than one stack can exist in the system at one time.
 */
typedef struct
{
    void*                       userContent;    ///< user defined pointer, can by used to pass anything
    modMasterState_t volatile   status;         ///< status of MODBUS engine

    pfModMSend_t                pfSend;         ///< send command function
    pfModMReceive_t             pfReceive;      ///< setup receiver function
    
    MODBUS_TIME_T               rxStartTime;    ///< absolute time, when Rx starts - used by timeout calculation
    uint8_t                     slaveAddr;      ///< address of slave device for command on the fly
    uint8_t                     opCode;         ///< operation code of command on the fly
    uint16_t                    firstReg;       ///< first register of R/W operation
    uint16_t                    numRegs;        ///< number of registers to read or write
    uint16_t*                   registers;      ///< user defined storage for rx or tx register-values
    uint16_t volatile           messageLast;    ///< total length of MODBUS message - 1
    uint8_t                     message[257];   ///< the message
} modMasterStack_t;

/**
 * @brief           Initializes Modbus-RTU master stack
 * @warning         mstack structure must have valid callback-function pointers BEFORE calling this fnc
 * @param mstack    Pointer to modbus stack structure
 * @return int16_t  O if OK, -1 if params are wrong
 */
int16_t ModMasterInit(modMasterStack_t* mstack);

/**
 * @brief               Initialize "read holding registers" operation of slave device
 * 
 * @param mstack        Pointer to modbus stack structure
 * @param modAddress    Slave device address
 * @param first         Address of first register
 * @param num           Number of registers to read
 * @param regs          Storage, it's caller responsibility to allocate enough space
 * @return int16_t      0 initialization successful, -1 stack is busy, -2 wrong params,
 *                      -3 HW error (next call of @ref ModMasterCheck() will report eMOD_M_STATE_HW_ERROR)
 */
int16_t ModMasterReadRegs(modMasterStack_t* mstack, uint8_t modAddress, uint16_t first, uint16_t num, uint16_t* regs);

/**
 * @brief               Initialize "write holding registers" operation of slave device
 * 
 * @param mstack        Pointer to modbus stack structure
 * @param modAddress    Slave device address
 * @param first         Address of first register
 * @param num           Number of registers to write
 * @param regs          Values to write
 * @return int16_t      0 initialization successful, -1 stack is busy, -2 wrong params,
 *                      -3 HW error (next call of @ref ModMasterCheck() will report eMOD_M_STATE_HW_ERROR)
 */
int16_t ModMasterWriteRegs(modMasterStack_t* mstack, uint8_t modAddress, uint16_t first, uint16_t num, const uint16_t* regs);

/**
 * @brief               Main function of Modbus stack. Has to be called periodically until operation is finished.
 * @note                If operation is done (successfully or not) resets status to eMOD_M_STATE_STANDBY.
 * @param mstack        Pointer to modbus stack structure.
 * @param lastStatus    If operation was done, mstack->status will be set to standby but this variable returns operation result.
 * @param errCode       Modbus error code, valid only if eMOD_M_STATE_ERR_REPORTED returned, ptr can be NULL.
 * @return int16_t      0 if operation is ongoing, 1 if done
 */
int16_t ModMasterCheck(modMasterStack_t* mstack, modMasterState_t* lastStatus, uint8_t* errCode);

/**
 * @ingroup ModbusMasterCb
 * @{
 */
/**
 * @brief           Call it from HW driver when command was transmitted.
 * @param mstack    pointer to modbus stack structure
 */
void ModMasterTxDoneCallback(modMasterStack_t* mstack);

/**
 * @brief           Call it from HW driver when full message received.
 * @param mstack    pointer to modbus stack structure
 * @param msg       whole received message (data will be copied to internal structure)
 * @param len       message length
 */
void ModMasterRxDoneCallback(modMasterStack_t* mstack, const uint8_t* msg, uint16_t len);

/**
 * @brief           Call it from HW driver when error happen during receiving.
 * @param mstack    pointer to modbus stack structure
 */
void ModMasterRxErrorCallback(modMasterStack_t* mstack);
/** @} */

#endif
