/*
 * CAN module object for STM32H7 FDCAN peripheral.
 *
 * Port driver that delegates to the low-level ICAN interface.
 *
 * @file        CO_driver.cpp
 * @ingroup     CO_driver
 * @copyright   2004 - 2020 Janez Paternoster (original template)
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this
 * file except in compliance with the License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include "301/CO_driver.h"
#include "ICAN.hpp"

#include <tx_api.h>
#include <cstring>

/* -------------------------------------------------------------------------- */
/* ThreadX OD mutex for CO_LOCK_OD / CO_UNLOCK_OD                            */
/* -------------------------------------------------------------------------- */
static TX_MUTEX odMutex;
static bool odMutexInitialized = false;

extern "C" void CO_lockOD(void)
{
    if (odMutexInitialized)
    {
        tx_mutex_get(&odMutex, TX_WAIT_FOREVER);
    }
}

extern "C" void CO_unlockOD(void)
{
    if (odMutexInitialized)
    {
        tx_mutex_put(&odMutex);
    }
}

void CO_OD_mutex_init(void)
{
    if (!odMutexInitialized)
    {
        tx_mutex_create(&odMutex, const_cast<CHAR *>("CO_OD_Mutex"), TX_INHERIT);
        odMutexInitialized = true;
    }
}

/* -------------------------------------------------------------------------- */
/* Helper: extract ICAN* from CANmodule->CANptr                               */
/* -------------------------------------------------------------------------- */
static inline Interfaces::LLDriver::ICAN *getICAN(CO_CANmodule_t *CANmodule)
{
    return static_cast<Interfaces::LLDriver::ICAN *>(CANmodule->CANptr);
}

/* -------------------------------------------------------------------------- */
/* Global CANmodule pointer for ISR callbacks                                 */
/* -------------------------------------------------------------------------- */
static CO_CANmodule_t *g_CANmodule = nullptr;

/* -------------------------------------------------------------------------- */
/* Helper: build CANMessage from CO_CANtx_t and send via LL driver            */
/* -------------------------------------------------------------------------- */
static bool sendTxBuffer(Interfaces::LLDriver::ICAN *ican, CO_CANtx_t *buffer)
{
    Interfaces::LLDriver::CANMessage msg{};
    msg.id = buffer->ident & 0x07FFU;
    msg.dlc = static_cast<uint8_t>((buffer->ident >> 11U) & 0x0FU);
    std::memcpy(msg.data.data(), buffer->data, msg.dlc);

    return ican->send(msg);
}

/* -------------------------------------------------------------------------- */
/* ISR callback: CAN RX                                                       */
/* -------------------------------------------------------------------------- */
static void canRxCallback(const Interfaces::LLDriver::CANMessage &msg, void * /* context */)
{
    CO_CANmodule_t *CANmodule = g_CANmodule;
    if (CANmodule == nullptr)
    {
        return;
    }

    /* Build a CO_CANrxMsg_t on the stack for the callback */
    CO_CANrxMsg_t rxMsg;
    rxMsg.ident = msg.id;
    rxMsg.DLC = msg.dlc;
    std::memcpy(rxMsg.data, msg.data.data(), msg.dlc);

    /* Search rxArray for a matching filter */
    CO_CANrx_t *buffer = &CANmodule->rxArray[0];
    for (uint16_t i = CANmodule->rxSize; i > 0U; i--)
    {
        if (((rxMsg.ident ^ buffer->ident) & buffer->mask) == 0U)
        {
            if (buffer->CANrx_callback != nullptr)
            {
                buffer->CANrx_callback(buffer->object, static_cast<void *>(&rxMsg));
            }
            break;
        }
        buffer++;
    }
}

/* -------------------------------------------------------------------------- */
/* ISR callback: CAN TX complete — drain pending buffers                      */
/* -------------------------------------------------------------------------- */
static void canTxCompleteCallback(void * /* context */)
{
    CO_CANmodule_t *CANmodule = g_CANmodule;
    if (CANmodule == nullptr)
    {
        return;
    }

    /* First CAN message (bootup) was sent successfully */
    CANmodule->firstCANtxMessage = false;
    /* Clear flag from previous message */
    CANmodule->bufferInhibitFlag = false;

    /* Drain pending TX buffers */
    if (CANmodule->CANtxCount > 0U)
    {
        auto *ican = getICAN(CANmodule);
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        for (uint16_t i = CANmodule->txSize; i > 0U; i--)
        {
            if (buffer->bufferFull)
            {
                buffer->bufferFull = false;
                CANmodule->CANtxCount--;
                CANmodule->bufferInhibitFlag = buffer->syncFlag;
                sendTxBuffer(ican, buffer);
                break; /* send one per interrupt */
            }
            buffer++;
        }
    }
}

/* ========================================================================== */
/* CANopenNode port driver API implementation                                 */
/* ========================================================================== */

void CO_CANsetConfigurationMode(void *CANptr)
{
    if (CANptr != nullptr)
    {
        auto *ican = static_cast<Interfaces::LLDriver::ICAN *>(CANptr);
        ican->setConfigMode();
    }
}

void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
    if (CANmodule != nullptr && CANmodule->CANptr != nullptr)
    {
        auto *ican = getICAN(CANmodule);
        ican->setNormalMode();
        CANmodule->CANnormal = true;
    }
}

CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t *CANmodule, void *CANptr, CO_CANrx_t rxArray[], uint16_t rxSize,
                                   CO_CANtx_t txArray[], uint16_t txSize, uint16_t CANbitRate)
{
    /* Verify arguments */
    if (CANmodule == nullptr || rxArray == nullptr || txArray == nullptr)
    {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Ensure OD mutex is initialized */
    CO_OD_mutex_init();

    /* Configure object variables */
    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = false; /* software filtering — accept all through HW */
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;

    for (uint16_t i = 0U; i < rxSize; i++)
    {
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = nullptr;
        rxArray[i].CANrx_callback = nullptr;
    }
    for (uint16_t i = 0U; i < txSize; i++)
    {
        txArray[i].bufferFull = false;
    }

    /* Store global pointer for ISR callbacks */
    g_CANmodule = CANmodule;

    /* Initialize the LL CAN driver */
    if (CANptr != nullptr)
    {
        auto *ican = static_cast<Interfaces::LLDriver::ICAN *>(CANptr);
        ican->registerRxCallback(canRxCallback, nullptr);
        ican->registerTxCompleteCallback(canTxCompleteCallback, nullptr);

        if (!ican->init(CANbitRate))
        {
            return CO_ERROR_ILLEGAL_ARGUMENT;
        }
    }

    return CO_ERROR_NO;
}

void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
    if (CANmodule != nullptr && CANmodule->CANptr != nullptr)
    {
        auto *ican = getICAN(CANmodule);
        ican->disable();
        g_CANmodule = nullptr;
    }
}

CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index, uint16_t ident, uint16_t mask,
                                    bool_t rtr, void *object,
                                    void (*CANrx_callback)(void *object, void *message))
{
    if (CANmodule == nullptr || object == nullptr || CANrx_callback == nullptr || index >= CANmodule->rxSize)
    {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    CO_CANrx_t *buffer = &CANmodule->rxArray[index];

    buffer->object = object;
    buffer->CANrx_callback = CANrx_callback;

    /* CAN identifier and CAN mask, bit aligned for software filtering */
    buffer->ident = ident & 0x07FFU;
    if (rtr)
    {
        buffer->ident |= 0x0800U;
    }
    buffer->mask = (mask & 0x07FFU) | 0x0800U;

    return CO_ERROR_NO;
}

CO_CANtx_t *CO_CANtxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index, uint16_t ident, bool_t rtr,
                               uint8_t noOfBytes, bool_t syncFlag)
{
    CO_CANtx_t *buffer = nullptr;

    if (CANmodule != nullptr && index < CANmodule->txSize)
    {
        buffer = &CANmodule->txArray[index];

        /* Store ident with DLC packed into upper bits (for sendTxBuffer extraction) */
        buffer->ident = (static_cast<uint32_t>(ident) & 0x07FFU) | (static_cast<uint32_t>(noOfBytes & 0x0FU) << 11U) | (rtr ? 0x8000U : 0U);

        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}

CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if (buffer->bufferFull)
    {
        if (!CANmodule->firstCANtxMessage)
        {
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    CO_LOCK_CAN_SEND(CANmodule);

    /* If CAN TX buffer is free, send immediately */
    if (CANmodule->CANtxCount == 0)
    {
        CANmodule->bufferInhibitFlag = buffer->syncFlag;

        auto *ican = getICAN(CANmodule);
        if (!sendTxBuffer(ican, buffer))
        {
            /* HW FIFO full — queue for TX-complete ISR */
            buffer->bufferFull = true;
            CANmodule->CANtxCount++;
        }
    }
    else
    {
        /* Previous messages are still pending — queue this one */
        buffer->bufferFull = true;
        CANmodule->CANtxCount++;
    }

    CO_UNLOCK_CAN_SEND(CANmodule);

    return err;
}

void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);

    if (CANmodule->bufferInhibitFlag)
    {
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }

    if (CANmodule->CANtxCount != 0U)
    {
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        for (uint16_t i = CANmodule->txSize; i > 0U; i--)
        {
            if (buffer->bufferFull && buffer->syncFlag)
            {
                buffer->bufferFull = false;
                CANmodule->CANtxCount--;
                tpdoDeleted = 2U;
            }
            buffer++;
        }
    }

    CO_UNLOCK_CAN_SEND(CANmodule);

    if (tpdoDeleted != 0U)
    {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}

void CO_CANmodule_process(CO_CANmodule_t *CANmodule)
{
    uint16_t rxErr = 0;
    uint16_t txErr = 0;
    uint16_t ovf = 0;

    if (CANmodule->CANptr != nullptr)
    {
        auto *ican = getICAN(CANmodule);
        ican->getErrorCounters(rxErr, txErr, ovf);
    }

    uint32_t err = (static_cast<uint32_t>(txErr) << 16U) | (static_cast<uint32_t>(rxErr) << 8U) | ovf;

    if (CANmodule->errOld != err)
    {
        uint16_t status = CANmodule->CANerrorStatus;
        CANmodule->errOld = err;

        if (txErr >= 256U)
        {
            status |= CO_CAN_ERRTX_BUS_OFF;
        }
        else
        {
            status &= 0xFFFFU ^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE);

            if (rxErr >= 128)
            {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE;
            }
            else if (rxErr >= 96)
            {
                status |= CO_CAN_ERRRX_WARNING;
            }

            if (txErr >= 128)
            {
                status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE;
            }
            else if (txErr >= 96)
            {
                status |= CO_CAN_ERRTX_WARNING;
            }

            if ((status & CO_CAN_ERRTX_PASSIVE) == 0)
            {
                status &= 0xFFFFU ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }

        if (ovf != 0)
        {
            status |= CO_CAN_ERRRX_OVERFLOW;
        }

        CANmodule->CANerrorStatus = status;
    }
}
