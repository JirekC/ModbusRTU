/**
 * @file    mod_slave_rtu.h
 * @brief   Simple implementation of Modbus RTU slave stack. Does not need any RTOS only @ref ModSlaveCheck()
 *          has to be periodically called to check new incomming messages.
 */

#ifndef SYSTEM_MOD_SLAVE_RTU_H_
#define SYSTEM_MOD_SLAVE_RTU_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @defgroup ModbusErrors Modbus error-report codes
 * @{
 */
#define MODBUS_ERR_ILLEGAL_OPCODE   0x01
#define MODBUS_ERR_ILLEGAL_ADDRESS  0x02
#define MODBUS_ERR_ILLEGAL_VALUE    0x03
#define MODBUS_ERR_DEVICE_FAULT     0x04
/** @} */

/** MODBUS engine status flags */
typedef enum
{
    eMOD_S_STATE_STANDBY,
    eMOD_S_STATE_RECEIVING,
    eMOD_S_STATE_RECEIVED,
    eMOD_S_STATE_PROCESSING,
    eMOD_S_STATE_TRANSMITTING
} modSlaveState_t;

typedef struct modSlaveStack_s modSlaveStack_t;

/**
 * @defgroup ModbusSlaveCb Modbus slave callbacks
 * @{
 */
/**
 * @brief   User will pass pointer to function that will be called from ModSlaveCheck() when eMOD_S_STATE_STANDBY.
 *          Has to turn on receiver, status will be changed by stack to eMOD_S_STATE_RECEIVING than.
 * @note    After successful receive of command, user have to call @ref ModSlaveRxDoneCallback().
 *          If error occurred @ref ModSlaveRxErrorCallback() has to be called.
 * @return  0 if everything OK, negative value in case of failure.
 */
typedef int16_t (*pfModSStandby_t) (modSlaveStack_t* mstack);

/**
 * @brief   User will pass pointer to function that sends @b data which are @b length bytes long over the UART.
 * @note    After successfull Tx, user has to call @ref ModSlaveTxDoneCallback().
 * @return  0 if everything OK, negative value in case of failure.
 */
typedef int16_t (*pfModSSendAns_t) (modSlaveStack_t* mstack, const uint8_t* data, uint16_t length);

/**
 * @brief   User will pass pointer to function that reads value of modbus register at address @b regAddr into @b *regValue
 * @return  0 if everything OK or @ref ModbusErrors code if fails (e.g. register doesn't exists)
 */
typedef uint8_t (*pfModSGetReg_t) (modSlaveStack_t* mstack, uint16_t regAddr, uint16_t * regValue);

/**
 * @brief   User will pass pointer to function that writes @b regValue to modbus register at address @b regAddr
 * @return  0 if everything OK or @ref ModbusErrors code if fails (e.g. register doesn't exists)
 */
typedef uint8_t (*pfModSSetReg_t) (modSlaveStack_t* mstack, uint16_t regAddr, uint16_t regValue);

/**
 * @brief   User will pass pointer to function that store packet from local FIFO to @b buffer and its lenth to @b length
 * @warning Maximum length of single packet is 251 Bytes
 * @return  0 if everything OK or @ref ModbusErrors code if fails (e.g. register doesn't exists)
 */
typedef uint8_t (*pfModSGetPacket_t) (modSlaveStack_t* mstack, uint8_t* buffer, uint16_t* length);

/**
 * @brief   User will pass pointer to function that store packet from master (in @b buffer) to local FIFO.
 *          Lenght of packet is @b length bytes.
 * @note    Maximum length of single packet is 251 Bytes
 * @return  0 if everything OK or @ref ModbusErrors code if fails (e.g. register doesn't exists)
 */
typedef uint8_t (*pfModSSetPacket_t) (modSlaveStack_t* mstack, const uint8_t* buffer, uint16_t length);
/** @} */

/**
 * @brief   Modbus slave stack structure. Pass pointer to this structure to each ModSlave function.
 *          More than one stack can exist in the system at one time.
 */
struct modSlaveStack_s
{
    void*                       userContent;    ///< user defined pointer, can by used to pass anything
    modSlaveState_t volatile    status;         ///< status of MODBUS engine
    uint8_t                     address;        ///< this module address
    uint16_t                    lastReg;        ///< last valid register

    pfModSStandby_t             pfStandby;      ///< called from ModSlaveCheck() when eMOD_S_STATE_STANDBY; has to turn on receiver; eMOD_S_STATE_RECEIVING than
    pfModSSendAns_t             pfSendAns;      ///< send answer function
    pfModSGetReg_t              pfGetReg;       ///< get register function
    pfModSSetReg_t              pfSetReg;       ///< set register function
    pfModSGetPacket_t           pfGetPacket;    ///< get packet from local FIFO to be sent to master
    pfModSSetPacket_t           pfSetPacket;    ///< store packet from incomming message to local FIFO

    uint16_t volatile           messageLast;    ///< total lengh of MODBUS message - 1
    uint8_t                     message[257];   ///< the message
};

/**
 * @brief           Initializes modbus stack
 * @warning         mstack structure must have valid address, lastReg and function pointers BEFORE calling this fnc
 * @param mstack    pointer to modbus stack structure
 * @return int16_t  O if OK, -1 if params are wrong
 */
int16_t ModSlaveInit(modSlaveStack_t* mstack);

/**
 * @brief           Check of MODBUS state, called periodically from main loop
 * @return int16_t  0 if no message received yet OR processed without any error,
 *                  -1 in case of corrupted / invalid message
 */
int16_t ModSlaveCheck(modSlaveStack_t* mstack);

/**
 * @ingroup ModbusSlaveCb
 * @{
 */
/**
 * @brief           Called by HW driver when full message received
 * @param mstack    pointer to modbus stack structure
 * @param msg       message payload
 * @param len       message length
 */
void ModSlaveRxDoneCallback(modSlaveStack_t* mstack, const uint8_t* msg, uint16_t len);

/**
 * @brief           Called by HW driver when error happend during receiving
 * @param mstack    pointer to modbus stack structure
 */
void ModSlaveRxErrorCallback(modSlaveStack_t* mstack);

/**
 * @brief           Called by HW when HW sent answer and is ready to Rx
 * @param mstack    pointer to modbus stack structure
 */
void ModSlaveTxDoneCallback(modSlaveStack_t* mstack);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_MOD_SLAVE_RTU_H_ */
