/**
 * @file    mod_master_rtu.h
 * @brief   Simple implementation of Modbus RTU master stack. Does not need any RTOS only @ref ModMasterCheck()
 *          has to be periodically called until Modbus transaction (operation) is done.
 * @note    FreeRTOS included here only to define @ref ModbusMasterInternal group macros. Use any other time-reading API.
 */

#ifndef SYSTEM_MOD_MASTER_RTU_H
#define SYSTEM_MOD_MASTER_RTU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @defgroup TimeAPI Time API includes & defines
 * It is here only for @ref ModbusMasterInternal. Feel free to change to any other API.
 * @{
 */
#include "FreeRTOS.h"
#include "task.h"

#define MODBUS_RX_TIMEOUT           100     ///< in milliseconds
#define MODBUS_TIME_T               TickType_t
#define MODBUS_GET_TIME_MS          (xTaskGetTickCount() * (1000/configTICK_RATE_HZ))
#define MODBUS_GET_TIME_ISR_MS      (xTaskGetTickCountFromISR() * (1000/configTICK_RATE_HZ))
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

typedef struct modMasterStack_s modMasterStack_t;

/**
 * @defgroup ModbusMasterCb Modbus master callbacks
 * @{
 */
/**
 * @brief   User will pass pointer to function that sends @b data which are @b length bytes long over the UART.
 * @note    After successfull Tx, user has to call @ref ModMasterTxDoneCallback().
 * @return  0 if everything OK, negative value in case of failure.
 */
typedef int16_t (*pfModMSend_t)(modMasterStack_t* mstack, const uint8_t* data, uint16_t length);

/**
 * @brief   User will pass pointer to function that starts receiving data from UART.
 * @note    After successful receive of answer, user have to call @ref ModMasterRxDoneCallback().
 *          If error occurred @ref ModMasterRxErrorCallback() has to be called.
 * @warning This function is called by @ref ModMasterTxDoneCallback(), so can be called from ISR.
 * @return  0 if everything OK, negative value in case of failure.
 */
typedef int16_t (*pfModMReceive_t)(modMasterStack_t* mstack);
/** @} */

/**
 * @brief   Modbus master stack structure. Pass pointer to this tructure to each ModMaster function.
 *          More than one stack can exist in the system at one time.
 */
struct modMasterStack_s
{
    void*                       userContent;    ///< user defined pointer, can by used to pass anything
    modMasterState_t volatile   status;         ///< status of MODBUS engine

    pfModMSend_t                pfSend;         ///< send command function
    pfModMReceive_t             pfReceive;      ///< setup receiver function
    
    MODBUS_TIME_T               rxStartTime;    ///< absolute time, when Rx starts - used by timeout calculation
    uint8_t                     slaveAddr;      ///< address of slave device for command on the fly
    uint8_t                     opCode;         ///< operation code of command on the fly
    uint16_t                    firstReg;       ///< first register of R/W operation
    uint16_t                    numRegs;        ///< amount of data to read or write
    void*                       dataStorage;    ///< user defined storage for rx or tx data
    void*                       dataStorage2;   ///< extra user defined storage
    uint16_t volatile           messageLast;    ///< total length of MODBUS message - 1
    uint8_t                     message[257];   ///< the message
};

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
 * @param regs          Storage, it's caller responsibility to allocate enough space.
 *                      Data are valid only after success finish of operation
 *                      @ref ModMasterCheck() reports eMOD_M_STATE_PROCESSED.
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

#ifdef MODBUS_USER_COMMANDS
/**
 * @brief               Initialize reading of one data packet (custom user defined Modbus operation) from slave device.
 *
 * @param mstack        Pointer to modbus stack structure
 * @param modAddress    Slave device address
 * @param length        Length of received data [in bytes] will be stored into this variable, can be NULL
 * @param data          Storage, it's caller responsibility to allocate enough space.
 *                      Data are valid only after success finish of operation
 *                      @ref ModMasterCheck() reports eMOD_M_STATE_PROCESSED.
 * @return int16_t      0 initialization successful, -1 stack is busy, -2 wrong params,
 *                      -3 HW error (next call of @ref ModMasterCheck() will report eMOD_M_STATE_HW_ERROR)
 */
int16_t ModMasterReadDataPacket(modMasterStack_t* mstack, uint8_t modAddress, uint8_t* length, uint8_t* data);

/**
 * @brief               Initialize writing of one data packet (custom user defined Modbus operation) to slave device.
 *
 * @param mstack        Pointer to modbus stack structure
 * @param modAddress    Slave device address
 * @param length        Number of bytes to write, max 251
 * @param data          Data to be sent
 * @return int16_t      0 initialization successful, -1 stack is busy, -2 wrong params,
 *                      -3 HW error (next call of @ref ModMasterCheck() will report eMOD_M_STATE_HW_ERROR)
 */
int16_t ModMasterWriteDataPacket(modMasterStack_t* mstack, uint8_t modAddress, uint8_t length, const uint8_t* data);
#endif

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

#ifdef __cplusplus
}
#endif

#endif
