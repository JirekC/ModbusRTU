# Simple implementation of Modbus RTU master and slave stack.

Does not need any RTOS only ModMasterCheck() or ModSlaveCheck() has to be periodically called.
Can be used with UART-style communication - RS-485 or similar standard.
Only implements Holding registers read and write commands from Modbus standard. This is because of simplicity. User can easily implement other commands if needed.

In addition to Modbus specification it implements two more custom user-define opcodes:

| OPCODE | Name | Description |
| --- | --- | --- |
| 0x64 | ReadDataPacket | Read packetized data from slave device in FIFO style |
| 0x65 | WriteDataPacket | Write packetized data to slave device in FIFO style |

It can be used if there is a stream of data packets that need to be transferred from/to the slave device. For example, packets received from another medium such as LoRa radio communication module, etc. In this case, standard modbus registers are used to read/write radio configuration and ReadDataPacket/WriteDataPacket to receive/transmitt actual data.
Please consult source code to see more details how to use it.

## Example of usage of MASTER stack (STM32 HAL used)
~~~
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

int16_t UartSendCb(modMasterStack_t* mstack, const uint8_t* data, uint16_t length)
{
    (void)mstack
    SWITCH_TX;
    HAL_UART_Transmit_DMA(&huart2, data, length);
    return 0;
}

int16_t UartReceiveCb(modMasterStack_t* mstack)
{
    HAL_UART_AbortReceive(&huart2);
    SWITCH_RX;
    // small hack - we can use message array directly by DMA
    // rx done callback will compare message printer with data param
    // and will avoid copy if pointers are same
    HAL_UART_Receive_DMA(&huart2, (uint8_t*)mstack->message, 257);
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
~~~



## Another example of usage of SLAVE stack (STM32 HAL used)
~~~
#define LAST_MODBUS_REGISTER 229
#define MY_MODBUS_ADDRESS    1

modSlaveStack_t myModbusStack;

...

// callbacks from ISR / HW driver to ModSlave

void U2RxCallback(UART_HandleTypeDef *huart)
{
    uint32_t error = HAL_UART_GetError(huart);
    uint16_t len = 257 - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    if (error & HAL_UART_ERROR_RTO) {
        // RTO = complete message received
        ModSlaveRxDoneCallback(&myModbusStack, (uint8_t*)myModbusStack.message, len);
    } else {
        // other error
        // OR no error, but full buffer (257 B) was received = overflow
        ModSlaveRxErrorCallback(&myModbusStack);
    }
    HAL_UART_AbortReceive(&huart2); // disable interrupts
}

void U2TxCallback(UART_HandleTypeDef *huart)
{
    ModSlaveTxDoneCallback(&myModbusStack);
    // HW will switch to RX in U2StandbyCb()
}

// callbacks called from ModSlave

int16_t U2StandbyCb(modSlaveStack_t* mstack)
{
    HAL_UART_AbortReceive(&huart2);
    SWITCH_RX;
    // small hack - we can use message array directly by DMA
    // rx done callback will compare message printer with data param
    // and will avoid copy if pointers are same
    HAL_UART_Receive_DMA(&huart2, (uint8_t*)mstack->message, 257);
    return 0;
}

int16_t U2SendAnswer(modSlaveStack_t* mstack, const uint8_t* data, uint16_t length)
{
    (void)mstack;
    SWITCH_TX;
    HAL_UART_Transmit_DMA(&huart2, data, length);
    return 0;
}

// will be called for every single register-read operation
uint8_t GetModReg(modSlaveStack_t* mstack, uint16_t regAddr, uint16_t * regValue)
{
    (void)mstack;
    uint8_t retval = 0;

    // some fake registers
    if(regAddr < 64)
    {
    	*regValue = timestamps[regAddr >> 1u] >> (regAddr & 1u ? 16 : 0);
    }
    else
    {
    	switch(regAddr)
    	{
			case 200:
				*regValue = relaysGet(0);
				break;
			case 201:
				*regValue = relaysGet(1);
				break;
			case 209:
				*regValue = relaysGet(9);
				break;

			default:
				retval = MODBUS_ERR_ILLEGAL_ADDRESS;
				break;
    	}
    }

    return retval;
}

// will be called for every single register-write operation
uint8_t SetModReg(modSlaveStack_t* mstack, uint16_t regAddr, uint16_t regValue)
{
    (void)mstack;
    uint8_t retval = 0;

    // some fake registers
    switch(regAddr)
    {
    	case 210:
    		relaysSet(0, regValue);
    		break;
    	case 211:
    		relaysSet(1, regValue);
    		break;
    	case 219:
    		relaysSet(9, regValue);
    		break;

    	case 220:
    		relaysReset(0, regValue);
			break;
		case 221:
			relaysReset(1, regValue);
			break;
		case 229:
			relaysReset(9, regValue);
			break;

        default:
            retval = MODBUS_ERR_ILLEGAL_ADDRESS;
    }

    return retval;
}

// callbacks for custom user defined opcodes --->
uint8_t packet[251] = "Hello!";

uint8_t GetModPacket(modSlaveStack_t* mstack, uint8_t* buffer, uint16_t* length)
{
    *length = strlen(packet) + 1; // terminating null
    *length = *length > 251 ? 251 : *length;
    memcpy(buffer, packet, *length);
    return 0;
}

uint8_t SetModPacket(modSlaveStack_t* mstack, const uint8_t* buffer, uint16_t length)
{
    memcpy(packet, buffer, length);
    return 0;
}
// <---

// initialisation procedure

void ModbusUARTInit(uint8_t address)
{
    memset(&myModbusStack, 0, sizeof(modSlaveStack_t));
    myModbusStack.address = address;
    myModbusStack.lastReg = LAST_MODBUS_REGISTER;
    myModbusStack.pfStandby = &U2StandbyCb;
    myModbusStack.pfSendAns = &U2SendAnswer;
    myModbusStack.pfGetReg = &GetModReg;
    myModbusStack.pfSetReg = &SetModReg;

// callbacks for custom user defined opcodes --->
    myModbusStack.pfGetPacket = &GetModPacket;
    myModbusStack.pfSetPacket = &SetModPacket;
// <---

    ModSlaveInit(&myModbusStack);

    // HW init
    HAL_UART_Init(&huart2);
    HAL_UART_RegisterCallback(&huart2, HAL_UART_RX_COMPLETE_CB_ID, U2RxCallback);
    HAL_UART_RegisterCallback(&huart2, HAL_UART_ERROR_CB_ID, U2RxCallback);
    HAL_UART_RegisterCallback(&huart2, HAL_UART_TX_COMPLETE_CB_ID, U2TxCallback);
    HAL_UART_EnableReceiverTimeout(&huart2);
    HAL_UART_ReceiverTimeout_Config(&huart2, 35); // 35 baud-blocks ~= 3.5 byte-times
    SWITCH_RX;
}

...

void main (void)
{
    // HW initialisation
    ...
    ModbusUARTInit(MY_MODBUS_ADDRESS); // listen on address defined above
    ...
    for (;;) {
        ...
        // will call GetModReg() and SetModReg() callbacks if master wants to R/W modbus registers
        (void)ModSlaveCheck(&myModbusStack);
        ...
    }
}
~~~
