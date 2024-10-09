#include <stdio.h>
#include <string.h>
#include "mod_master_rtu.h"
#include <crc.h>

//MODBUS commands
#define MODBUS_OPCODE_READ_OUT_REGS     0x03
#define MODBUS_OPCODE_READ_INP_REGS     0x04
#define MODBUS_OPCODE_WRITE_MULTI_REGS  0x10
#define MODBUS_OPCODE_DIAGNOSTIC        0x08

int16_t ModMasterInit (modMasterStack_t* mstack)
{
    //MODBUS engine
    if (mstack->pfSend == NULL ||
        mstack->pfReceive == NULL)
    {
        return -1; // wrong config
    }

    mstack->status = eMOD_M_STATE_STANDBY;

    return 0;
}

//calc CRC and initialize transmit
static int16_t ModMasterSend (modMasterStack_t* mstack)
{
    int16_t retval = 0;
    uint16_t crc;
    
    // check msg length 256Bytes total (254 + space for 2 CRC)
    if (mstack->messageLast > 253)
    {
        retval = -1;
    }
    else
    {
        crc = CrcModbus ((uint8_t*)mstack->message, mstack->messageLast + 1, 0xFFFF);
        mstack->message[(++mstack->messageLast)] = (uint8_t)crc;
        mstack->message[(++mstack->messageLast)] = (uint8_t)(crc >> 8);
        //reserve the bus for us
        mstack->status = eMOD_M_STATE_TRANSMITTING;
        if (mstack->pfSend((uint8_t*)mstack->message, mstack->messageLast + 1) < 0) {
            retval = -3;
            mstack->status = eMOD_M_STATE_HW_ERROR;
        }
    }

    return retval;
}

int16_t ModMasterReadRegs(modMasterStack_t* mstack, uint8_t modAddress, uint16_t first, uint16_t num, uint16_t* regs)
{
    if( mstack->status != eMOD_M_STATE_STANDBY )
    {
        return -1; // stack is busy
    }
    if( num > 125 || regs == NULL )
    {
        return -2; // wrong params
    }

    mstack->slaveAddr = modAddress;
    mstack->opCode = MODBUS_OPCODE_READ_OUT_REGS;
    mstack->firstReg = first;
    mstack->numRegs = num;
    mstack->registers = regs;

    mstack->message[0] = modAddress;
    mstack->message[1] = mstack->opCode;
    mstack->message[2] = (uint8_t)(first >> 8);
    mstack->message[3] = (uint8_t)(first);
    mstack->message[4] = (uint8_t)(num >> 8);
    mstack->message[5] = (uint8_t)(num);
    mstack->messageLast = 5;

    return ModMasterSend(mstack);
}

int16_t ModMasterWriteRegs(modMasterStack_t* mstack, uint8_t modAddress, uint16_t first, uint16_t num, const uint16_t* regs)
{
    if( mstack->status != eMOD_M_STATE_STANDBY )
    {
        return -1; // stack is busy
    }
    if( num > 123 || regs == NULL )
    {
        return -2; // wrong params
    }

    mstack->slaveAddr = modAddress;
    mstack->opCode = MODBUS_OPCODE_WRITE_MULTI_REGS;
    mstack->firstReg = first;
    mstack->numRegs = num;

    mstack->message[0] = modAddress;
    mstack->message[1] = mstack->opCode;
    mstack->message[2] = (uint8_t)(first >> 8);
    mstack->message[3] = (uint8_t)(first);
    mstack->message[4] = (uint8_t)(num >> 8);
    mstack->message[5] = (uint8_t)(num);
    mstack->message[6] = (uint8_t)(num * 2);
    mstack->messageLast = 6;
    for(uint16_t i = 0; i < num; i++) {
        mstack->message[++mstack->messageLast] = (uint8_t)(regs[i] >> 8);
        mstack->message[++mstack->messageLast] = (uint8_t)(regs[i]);
    }
    
    return ModMasterSend(mstack);
}

// process answer PDU - check if everything is OK
static void ModMasterProcessAnswer(modMasterStack_t* mstack)
{
    if ((mstack->message[1] & 0x7F) != mstack->opCode)
    {
        // answer to wrong command :o
        mstack->status = eMOD_M_STATE_CORRUPTED;
    }
    else if (mstack->message[1] & 0x80)
    {
        // error reported
        if (mstack->messageLast < 2)
        {
            mstack->status = eMOD_M_STATE_CORRUPTED;
        }
        else
        {
            // error code in message[2]
            mstack->status = eMOD_M_STATE_ERR_REPORTED;
        }
    }
    else
    {
        switch (mstack->message[1])
        {
            case MODBUS_OPCODE_READ_OUT_REGS:
            case MODBUS_OPCODE_READ_INP_REGS:
                if (mstack->messageLast < (2 + 2 * mstack->numRegs) ||
                    mstack->message[2] != 2 * mstack->numRegs)
                {
                    mstack->status = eMOD_M_STATE_CORRUPTED;
                }
                else
                {
                    // copy register-values (validity of pointer already checked in ModMasterReadRegs())
                    for (uint16_t i = 0; i < mstack->numRegs; i++)
                    {
                        mstack->registers[i] = ((uint16_t)(mstack->message[3 + 2 * i]) << 8) | mstack->message[4 + 2 * i];
                    }
                    mstack->status = eMOD_M_STATE_PROCESSED;
                }
                break;

            case MODBUS_OPCODE_WRITE_MULTI_REGS:
                if (mstack->messageLast < 5)
                {
                    mstack->status = eMOD_M_STATE_CORRUPTED;
                }
                else
                {
                    if (mstack->message[2] == (uint8_t)(mstack->firstReg >> 8) &&
                        mstack->message[3] == (uint8_t)(mstack->firstReg) &&
                        mstack->message[4] == (uint8_t)(mstack->numRegs >> 8) &&
                        mstack->message[5] == (uint8_t)(mstack->numRegs))
                    {
                        mstack->status = eMOD_M_STATE_PROCESSED;
                    }
                    else
                    {
                        mstack->status = eMOD_M_STATE_CORRUPTED;
                    }
                }
                break;

            default:
                // something very bad happend
                mstack->status = eMOD_M_STATE_CORRUPTED;
                break;
        }
    }
}

//parse received message and answer
static void ModMasterParseAnswer (modMasterStack_t* mstack)
{
    uint16_t crc;
    
    mstack->status = eMOD_M_STATE_PROCESSING;
    
    //if message is from device we wanted
    if (mstack->message[0] == mstack->slaveAddr)
    {
        /*** test for message validity ***/
        if (mstack->messageLast < 3)
        {
            // message too short
            mstack->status = eMOD_M_STATE_CORRUPTED;
        }
        else
        {
            crc = CrcModbus ((uint8_t*)mstack->message, mstack->messageLast - 1, 0xFFFF);
            if (mstack->message[(mstack->messageLast--)] != (uint8_t)(crc >> 8) ||
                mstack->message[(mstack->messageLast--)] != (uint8_t)(crc))
            {
                // invalid CRC
                mstack->status = eMOD_M_STATE_CORRUPTED;
            }
            else
            {
                // message correct, process it (.messageLast pointing to last byte of data, not CRC)
                ModMasterProcessAnswer(mstack);
            }
        }
    }
    //if message is not from slave we wanted
    else
    {
        mstack->status = eMOD_M_STATE_CORRUPTED;
    }
}

int16_t ModMasterCheck(modMasterStack_t* mstack, modMasterState_t* lastStatus, uint8_t* errCode)
{
    int16_t retval = 0;
    modMasterState_t status;
    //INTERRUPT_PROTECT(
        status = mstack->status; // atomic read on ARM (!)
    //);

    switch (status) {
    case eMOD_M_STATE_STANDBY:
        // nothing to do, simpy return status
        retval = 1;
        break;

    case eMOD_M_STATE_TRANSMITTING:
        // nothing to do, simpy return status
        break;

    case eMOD_M_STATE_WAITING_ANSWER:
        if ((MODBUS_GET_TIME_MS - mstack->rxStartTime) > (MODBUS_TIME_T)MODBUS_RX_TIMEOUT) {
            status = eMOD_M_STATE_TIMED_OUT; // report timeout
            mstack->status = eMOD_M_STATE_STANDBY; // atomic write (!)
            retval = 1;
        }
        break;

    case eMOD_M_STATE_RECEIVED:
        ModMasterParseAnswer(mstack); // parse answer
        status = mstack->status; // refresh - status will change during parsing
        // copy error code reported by slave (if any)
        if( status == eMOD_M_STATE_ERR_REPORTED && errCode != NULL ) {
            *errCode = mstack->message[2];
        }
        mstack->status = eMOD_M_STATE_STANDBY; // operation done, go standby
        retval = 1;
        break;

    // eMOD_M_STATE_PROCESSING used only by ModMasterParseAnswer()
    // eMOD_M_STATE_TIMED_OUT managed in eMOD_M_STATE_WAITING_ANSWER

    case eMOD_M_STATE_CORRUPTED:
        mstack->status = eMOD_M_STATE_STANDBY; // operation done, go standby
        retval = 1;
        break;

    // eMOD_M_STATE_ERR_REPORTED managed in eMOD_M_STATE_WAITING_ANSWER
    // eMOD_M_STATE_PROCESSED managed in eMOD_M_STATE_WAITING_ANSWER

    case eMOD_M_STATE_HW_ERROR:
        mstack->status = eMOD_M_STATE_STANDBY; // operation done, go standby
        retval = 1;
        break;

    default:
        mstack->status = eMOD_M_STATE_STANDBY; // failsafe
        break;
    }

    *lastStatus = status;

    return retval;
}

/*************************************************/
/***** COM callbacks, can be called from ISR *****/
/*************************************************/

void ModMasterTxDoneCallback(modMasterStack_t* mstack)
{
    if (mstack->status == eMOD_M_STATE_TRANSMITTING)
    {
        mstack->status = eMOD_M_STATE_WAITING_ANSWER;
        if (mstack->pfReceive() < 0) {
            mstack->status = eMOD_M_STATE_HW_ERROR;
        }
        mstack->rxStartTime = MODBUS_GET_TIME_ISR_MS;
    }
}

void ModMasterRxDoneCallback(modMasterStack_t* mstack, const uint8_t* msg, uint16_t len)
{
    // full message received
    if (mstack->status == eMOD_M_STATE_WAITING_ANSWER)
    {
        if (len < 1 || len > 257)
        {
            // too short or too long message
            mstack->status = eMOD_M_STATE_CORRUPTED;
        }
        else
        {
            mstack->messageLast = len - 1; // index of last received byte
            if (mstack->message != msg)
            {
                // copy data only if original msg is somewhere else than internal buffer
                memcpy((uint8_t*)mstack->message, msg, len);
            }
            mstack->status = eMOD_M_STATE_RECEIVED; // parse new message in main code
        }
    }
}

void ModMasterRxErrorCallback(modMasterStack_t* mstack)
{
    // general rx error
    if (mstack->status == eMOD_M_STATE_WAITING_ANSWER)
    {
        mstack->status = eMOD_M_STATE_CORRUPTED;
    }
}
