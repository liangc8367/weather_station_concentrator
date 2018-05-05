/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/***** Includes *****/
/* XDCtools Header files */ 
#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include "ConcentratorRadioTask.h"

/* BIOS Header files */ 
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>

/* Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>

/* Board Header files */
#include "Board.h"

/* EasyLink API Header files */ 
#include "easylink/EasyLink.h"

/* Application Header files */ 
#include "RadioProtocol.h"


/***** Defines *****/
#define CONCENTRATORRADIO_TASK_STACK_SIZE 1024
#define CONCENTRATORRADIO_TASK_PRIORITY   3

#define RADIO_EVENT_ALL                  0xFFFFFFFF
#define RADIO_EVENT_VALID_PACKET_RECEIVED      (uint32_t)(1 << 0)
#define RADIO_EVENT_INVALID_PACKET_RECEIVED (uint32_t)(1 << 1)

#define CONCENTRATORRADIO_MAX_RETRIES 2
#define NORERADIO_ACK_TIMEOUT_TIME_MS (160)


#define CONCENTRATOR_ACTIVITY_LED Board_PIN_LED0

/***** Type declarations *****/



/***** Variable declarations *****/
static Task_Params concentratorRadioTaskParams;
Task_Struct concentratorRadioTask; /* not static so you can see in ROV */
static uint8_t concentratorRadioTaskStack[CONCENTRATORRADIO_TASK_STACK_SIZE];
Event_Struct radioOperationEvent;  /* not static so you can see in ROV */
static Event_Handle radioOperationEventHandle;



//static ConcentratorRadio_PacketReceivedCallback packetReceivedCallback;
//static union ConcentratorPacket latestRxPacket;
//static EasyLink_TxPacket txPacket;
//static struct AckPacket ackPacket;
//static uint8_t concentratorAddress;
//static int8_t latestRssi;

sensorDataReceived gSensorDataCb = NULL;
sensorConfigReceived gSensorConfigCb = NULL;
static EasyLink_RxPacket gRxedPacket; //cached rxed packet


/***** Prototypes *****/
static void concentratorRadioTaskFunction(UArg arg0, UArg arg1);
static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status);
//static void notifyPacketReceived(union ConcentratorPacket* latestRxPacket);
//static void sendAck(uint8_t latestSourceAddress);
static void notifyPacketReceived();

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/* Configure LED Pin */
PIN_Config ledPinTable[] = {
        CONCENTRATOR_ACTIVITY_LED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

// Public MAC address of the IoT hub.
uint64_t iotHubAddr = 0;

// some stats
uint64_t gRxedSensorDataCnt = 0;
uint64_t gRxedSensorConfCnt = 0;
uint64_t gRxedUnknownCnt = 0;

/***** Function definitions *****/
void ConcentratorRadioTask_init(void)
{
    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
	if (!ledPinHandle)
	{
        System_abort("Error initializing board 3.3V domain pins\n");
    }

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&radioOperationEvent, &eventParam);
    radioOperationEventHandle = Event_handle(&radioOperationEvent);

    /* Create the concentrator radio protocol task */
    Task_Params_init(&concentratorRadioTaskParams);
    concentratorRadioTaskParams.stackSize = CONCENTRATORRADIO_TASK_STACK_SIZE;
    concentratorRadioTaskParams.priority = CONCENTRATORRADIO_TASK_PRIORITY;
    concentratorRadioTaskParams.stack = &concentratorRadioTaskStack;
    Task_construct(&concentratorRadioTask, concentratorRadioTaskFunction, &concentratorRadioTaskParams, NULL);
}

//void ConcentratorRadioTask_registerPacketReceivedCallback(ConcentratorRadio_PacketReceivedCallback callback) {
//    packetReceivedCallback = callback;
//}

static void concentratorRadioTaskFunction(UArg arg0, UArg arg1)
{
    /* Initialize EasyLink */
	EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);
	
	easyLink_params.ui32ModType = RADIO_EASYLINK_MODULATION;
	
	if(EasyLink_init(&easyLink_params) != EasyLink_Status_Success){ 
		System_abort("EasyLink_init failed");
	}	

    /* If you wich to use a frequency other than the default use
     * the below API
     * EasyLink_setFrequency(868000000);
     */

    /* Set concentrator address */;
//    concentratorAddress = RADIO_CONCENTRATOR_ADDRESS;
    iotHubAddr = RADIO_CONCENTRATOR_ADDRESS;
    EasyLink_enableRxAddrFilter((uint8_t *)&iotHubAddr, 8, 1);

//    /* Set up Ack packet */
//    ackPacket.header.sourceAddress = concentratorAddress;
//    ackPacket.header.packetType = RADIO_PACKET_TYPE_ACK_PACKET;

    /* Enter receive */
    if(EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success) {
        System_abort("EasyLink_receiveAsync failed");
    }

    while (1) {
        uint32_t events = Event_pend(radioOperationEventHandle, 0, RADIO_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* If valid packet received */
        if(events & RADIO_EVENT_VALID_PACKET_RECEIVED) {

//            /* Send ack packet */
//            sendAck(latestRxPacket.header.sourceAddress);
//
//            /* Call packet received callback */
//            notifyPacketReceived(&latestRxPacket);
            notifyPacketReceived();

            /* Go back to RX */
            if(EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success) {
                System_abort("EasyLink_receiveAsync failed");
            }

            /* toggle Activity LED */
            PIN_setOutputValue(ledPinHandle, CONCENTRATOR_ACTIVITY_LED,
                    !PIN_getOutputValue(CONCENTRATOR_ACTIVITY_LED));
        }

        /* If invalid packet received */
        if(events & RADIO_EVENT_INVALID_PACKET_RECEIVED) {
            /* Go back to RX */
            if(EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success) {
                System_abort("EasyLink_receiveAsync failed");
            }
        }
    }
}

//static void sendAck(uint8_t latestSourceAddress) {
//
//    /* Set destinationAdress, but use EasyLink layers destination adress capability */
//    txPacket.dstAddr[0] = latestSourceAddress;
//
//    /* Copy ACK packet to payload, skipping the destination adress byte.
//     * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
//    memcpy(txPacket.payload, &ackPacket.header, sizeof(ackPacket));
//    txPacket.len = sizeof(ackPacket);
//
//    /* Send packet  */
//    if (EasyLink_transmit(&txPacket) != EasyLink_Status_Success)
//    {
//        System_abort("EasyLink_transmit failed");
//    }
//}

static void notifyPacketReceived()
{
    int8_t rssi;
    struct PacketHeader header;
    struct IoTSensorData sensorData;
    struct IoTSensorConfig sensorConfig;

    rssi = (int8_t)gRxedPacket.rssi;

    // Deserialize the rxed packet
    memcpy(&header._sourceAddress, gRxedPacket.payload, sizeof(uint64_t));
    header._packetType = gRxedPacket.payload[sizeof(uint64_t)];

    switch(header._packetType) {
    case RADIO_PACKET_TYPE_SENSOR_DATA:
        gRxedSensorDataCnt++;
        memcpy(&sensorData, gRxedPacket.payload + OTA_PACKET_HEADER_SIZE, sizeof(struct IoTSensorData));
        if( gSensorDataCb ) {
            gSensorDataCb( &header, &sensorData, rssi );
        }
        break;
    case RADIO_PACKET_TYPE_SENSOR_CONF:
        gRxedSensorConfCnt++;
        memcpy(&sensorConfig, gRxedPacket.payload + OTA_PACKET_HEADER_SIZE, sizeof(struct IoTSensorConfig));
        if( gSensorConfigCb ) {
            gSensorConfigCb( &header, &sensorConfig, rssi);
        }
        break;
    default:
        gRxedUnknownCnt++;
    }
}

static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
//    union ConcentratorPacket* tmpRxPacket;

    /* If we received a packet successfully */
    if (status == EasyLink_Status_Success) {
        memcpy(&gRxedPacket, rxPacket, sizeof(EasyLink_RxPacket));

        /* Signal packet received */
        Event_post(radioOperationEventHandle, RADIO_EVENT_VALID_PACKET_RECEIVED);
    } else {
        /* Signal invalid packet received */
        Event_post(radioOperationEventHandle, RADIO_EVENT_INVALID_PACKET_RECEIVED);
    }
}

void registerSensorDataReceived(sensorDataReceived cb)
{
    gSensorDataCb = cb;
}
void registerSensorConfigReceived(sensorConfigReceived cb)
{
    gSensorConfigCb = cb;
}
