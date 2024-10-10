#include <stdio.h>
#include <string.h>
#include "mod_slave_rtu.h"
#include "crc.h"

// MODBUS commands
#define MODBUS_OPCODE_READ_OUT_REGS     0x03
#define MODBUS_OPCODE_READ_INP_REGS     0x04
#define MODBUS_OPCODE_WRITE_MULTI_REGS  0x10
#define MODBUS_OPCODE_DIAGNOSTIC        0x08
// custom user defined commands
#define MODBUS_OPCODE_READ_DATA_PACKET  0x64
#define MODBUS_OPCODE_WRITE_DATA_PACKET 0x65

int16_t ModSlaveInit (modSlaveStack_t* mstack)
{
    int16_t retval = 0;

    //MODBUS engine

    if (mstack->address == 0 ||
        mstack->pfStandby == NULL ||
        mstack->pfGetReg == NULL ||
        mstack->pfSetReg == NULL ||
        mstack->pfSendAns == NULL  )
    {
        retval = -1; // wrong config
    }
    else
    {
        mstack->status = eMOD_S_STATE_STANDBY;
    }

    return retval;
}

//calc CRC and initialize transmit
static int16_t ModSlaveSendAnswer (modSlaveStack_t* mstack)
{
    int16_t retval = 0;
    uint16_t crc;

    // check msg lenght 256Bytes total (254 + 2CRC)
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
        mstack->status = eMOD_S_STATE_TRANSMITTING;
        retval = mstack->pfSendAns(mstack, (uint8_t*)mstack->message, mstack->messageLast + 1);
    }

    return retval;
}

//build error reporting message
static void ModSlaveErrorReport(modSlaveStack_t* mstack, uint8_t err)
{
    mstack->message[1] += 0x80;      //error report
    mstack->message[2] = err;        //error code
    mstack->messageLast = 2;
}

//process read / write commands
static int16_t ModSlaveProcessCommand(modSlaveStack_t* mstack)
{
    int16_t retval = 0;
    uint16_t i, j;

    switch (mstack->message[1])
    {
        case MODBUS_OPCODE_READ_OUT_REGS:
        case MODBUS_OPCODE_READ_INP_REGS:
            //first register address
            i  = (uint16_t)mstack->message[2] << 8;
            i |= (uint16_t)mstack->message[3];

            //check if message have correct lengh &
            //minimum 1, maximum 125 registers can be readed
            if (mstack->messageLast != 5  ||
                mstack->message[4]   != 0  ||
                mstack->message[5]   > 125 ||
                mstack->message[5]   < 1)
            {
                ModSlaveErrorReport(mstack, MODBUS_ERR_ILLEGAL_VALUE);
                retval = -1;
                break; // case
            }

            //last register address
            j = mstack->message[5] - 1 + i;
            if (i > j ||                //overflow, more than 65535?
                j > mstack->lastReg)   //address range test, first register starting at 0x0000 -> not tested :)
            {
                ModSlaveErrorReport(mstack, MODBUS_ERR_ILLEGAL_ADDRESS);
                retval = -1;
                break; // case
            }

            //build the answer
            mstack->message[2] = 2 * mstack->message[5]; //number of bytes
            mstack->messageLast = 2;
            for ( ; i <= j; i++)
            {
                uint16_t v;
                uint8_t r = mstack->pfGetReg(mstack, i, &v);
                if(r > 0)
                {
                    ModSlaveErrorReport(mstack, r);
                    retval = -1;
                    break; // for
                }
                mstack->message[(++mstack->messageLast)] = (uint8_t)(v >> 8);
                mstack->message[(++mstack->messageLast)] = (uint8_t)(v);
            }
            break;


        case MODBUS_OPCODE_WRITE_MULTI_REGS:
            //first register address
            i  = (uint16_t)mstack->message[2] << 8;
            i |= (uint16_t)mstack->message[3];

            //minimum 1, maximum 123 registers can be writen
            if (mstack->message[4] != 0  ||
                mstack->message[5] > 123 ||
                mstack->message[5] < 1)
            {
                ModSlaveErrorReport(mstack, MODBUS_ERR_ILLEGAL_VALUE);
                retval = -1;
                break; // case
            }

            //check if no_of_bytes = 2 * no_of_registers; and message have correct lengh
            if (mstack->message[6] != 2 * mstack->message[5] ||
                mstack->message[6] != mstack->messageLast - 6)
            {
                ModSlaveErrorReport(mstack, MODBUS_ERR_ILLEGAL_VALUE);
                retval = -1;
                break; // case
            }

            //last register address
            j = mstack->message[5] - 1 + i;
            if (i > j ||                //overflow, more than 65535?
                j > mstack->lastReg)   //address range test, first register starting at 0x0000 -> not tested :)
            {
                ModSlaveErrorReport(mstack, MODBUS_ERR_ILLEGAL_ADDRESS);
                retval = -1;
                break; // case
            }

            //write registers
            uint16_t idx = 6;
            for ( ; i <= j; i++)
            {
                uint16_t v  = (uint16_t)mstack->message[(++idx)] << 8;
                         v |= (uint16_t)mstack->message[(++idx)];

                uint8_t r = mstack->pfSetReg(mstack, i, v);
                if(r > 0)
                {
                    ModSlaveErrorReport(mstack, r);
                    retval = -1;
                    break; // for
                }
            }

            //answer
            if (retval == 0)
            {
                mstack->messageLast = 5; // everything was OK, send original header
            }
            break;


        case MODBUS_OPCODE_DIAGNOSTIC:
            //PING, subcode 0x0000
            if (mstack->message[2] == 0 &&
                mstack->message[3] == 0)
            {
                ;   //answer the same message
            }
            else
            {
                ModSlaveErrorReport(mstack, MODBUS_ERR_ILLEGAL_OPCODE);
                retval = -1;
            }
            break;

        case MODBUS_OPCODE_READ_DATA_PACKET:
            if (mstack->messageLast != 2)
            {
                ModSlaveErrorReport(mstack, MODBUS_ERR_ILLEGAL_VALUE);
                retval = -1;
            }
            else
            {
                uint8_t r = mstack->pfGetPacket(mstack, mstack->message + 3, &i);
                if (r > 0)
                {
                    ModSlaveErrorReport(mstack, r);
                    retval = -1;
                }
                else if (i > 251)
                {
                    // internal fault - pfGetPacket callback returned too long packet
                    ModSlaveErrorReport(mstack, MODBUS_ERR_DEVICE_FAULT);
                    retval = -1;
                }
                else
                {
                    mstack->message[2] = (uint8_t)i; // length of data
                    mstack->messageLast = i + 2;
                }
            }
            break;

        case MODBUS_OPCODE_WRITE_DATA_PACKET:
            if (mstack->messageLast != (mstack->message[2] + 2))
            {
                ModSlaveErrorReport(mstack, MODBUS_ERR_ILLEGAL_VALUE);
                retval = -1;
            }
            else
            {
                mstack->pfSetPacket(mstack, mstack->message + 3, mstack->message[2]);
                mstack->messageLast = 2; // answer
            }
            break;

        //unsupported opcode
        default:
            ModSlaveErrorReport(mstack, MODBUS_ERR_ILLEGAL_OPCODE);
            retval = -1;
            break;
    }

    return retval;
}

// parse received message and answer
static int16_t ModSlaveParseMessage(modSlaveStack_t* mstack)
{
    int16_t retval = 0;
    uint16_t crc;

    mstack->status = eMOD_S_STATE_PROCESSING;

    //if message is for me or b-cast...
    if (mstack->message[0] == mstack->address || mstack->message[0] == 0)
    {
        /*** test for message validity ***/
        if (mstack->messageLast < 3)
        {
            // message too short
            mstack->status = eMOD_S_STATE_STANDBY;
            retval = -1;
        }
        else
        {
            crc = CrcModbus ((uint8_t*)mstack->message, mstack->messageLast - 1, 0xFFFF);
            if (mstack->message[(mstack->messageLast--)] != (uint8_t)(crc >> 8) ||
                mstack->message[(mstack->messageLast--)] != (uint8_t)(crc))
            {
                // invalid CRC
                mstack->status = eMOD_S_STATE_STANDBY;
                retval = -1;
            }
            else
            {
                //message correct, process it (.messageLast pointing to last byte of data, not CRC)
                retval = ModSlaveProcessCommand(mstack);

                //if not broad-cast, send answer
                if (mstack->message[0] != 0)
                {
                    int16_t r = ModSlaveSendAnswer(mstack);
                    if( r < 0 )
                    {
                        retval = r; // sending fails, override MODslave_process_command() return value
                    }
                }
                else
                {
                    mstack->status = eMOD_S_STATE_STANDBY;
                }
            }
        }
    }
    //if message is not for me
    else
    {
        // On 485 I will receive somebody's answer as CMD, but
        // it will have it's address = ignored by me
        mstack->status = eMOD_S_STATE_STANDBY;
    }

    return retval;
}

int16_t ModSlaveCheck(modSlaveStack_t* mstack)
{
    int16_t retval = 0;

     // refresh listening-state
    if (mstack->status == eMOD_S_STATE_STANDBY)
    {
        mstack->status = eMOD_S_STATE_RECEIVING;
        retval = mstack->pfStandby(mstack);
    }
    // MODBUS receive completed check
    if (mstack->status == eMOD_S_STATE_RECEIVED)
    {
       retval = ModSlaveParseMessage(mstack);
    }

    return retval;
}

/*************************************************/
/***** COM callbacks, can be called from ISR *****/
/*************************************************/

void ModSlaveRxDoneCallback(modSlaveStack_t* mstack, const uint8_t* msg, uint16_t len)
{
    // full message received
    if (mstack->status == eMOD_S_STATE_RECEIVING)
    {
        if (len < 1 || len > 257)
        {
            // too short or too long message, ignore it
            mstack->status = eMOD_S_STATE_STANDBY; // re-start in ModSlaveCheck()
        }
        else
        {
            mstack->status = eMOD_S_STATE_RECEIVED; // parse new message in main code
            mstack->messageLast = len - 1; // index of last received byte

            if (mstack->message != msg)
            {
                // copy data only if original msg is somewhere else than internal buffer
                memcpy((uint8_t*)mstack->message, msg, len);
            }
        }
    }
}

void ModSlaveRxErrorCallback(modSlaveStack_t* mstack)
{
    // general rx error
    if (mstack->status == eMOD_S_STATE_RECEIVING)
    {
        mstack->status = eMOD_S_STATE_STANDBY; // re-start in ModSlaveCheck()
    }
}

void ModSlaveTxDoneCallback(modSlaveStack_t* mstack)
{
    if (mstack->status == eMOD_S_STATE_TRANSMITTING)
    {
        mstack->status = eMOD_S_STATE_STANDBY; // re-start in ModSlaveCheck()
    }
}
