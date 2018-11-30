/*
 * Copyright (c) 2015-2018, Texas Instruments Incorporated
 * Copyright (c) 2018, This. Is. IoT. - https://thisisiot.io
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
 /**
 *
 * \file
 *        A Contiki-friendly version of TI's PDM driver based on coredsk 3.60.01
 *        https://github.com/contiki-ng/coresdk_cc13xx_cc26xx@b83faf3
 *        ti/drivers/pdm
 *
 *        Key changes:
 *        - references to xdc/runtime are removed
 *        - ti/sysbios/knl/Queue replaced with ti/drivers/dpl/QueueP
 *        - Event and Task replaced with appropriate Contiki mechanisms.
 *
 *        In blocking mode this driver uses a SemaphoreP to protect a buffer
 *        between the interrupt handler context and the user task (PDM driver) context.
 *
 *        The PDM driver, however, uses PDMCC26XX_I2S_CALLBACK_MODE which does not
 *        require the use of the I2S internal semaphore, delegating the processing
 *        of the buffers to the PDM task.  The PDM callback handler must use
 *        contiki process_poll() to poll its internal process when a buffer is ready.
 *        The process may acquire the buffer without worry about acquiring a semaphore.

 *
 * \author
 *        Evan Ross <evan@thisisiot.io>
 */

#include <ti/drivers/dpl/SemaphoreP.h>
#include "QueueP.h" /* still missing a couple of functions implemented below */
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

#include "PDMCC26XX_util_contiki.h"

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_ints.h)
#include DeviceFamily_constructPath(inc/hw_types.h)
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)
#include DeviceFamily_constructPath(driverlib/ioc.h)
#include DeviceFamily_constructPath(driverlib/rom.h)
#include DeviceFamily_constructPath(driverlib/prcm.h)
#include DeviceFamily_constructPath(driverlib/i2s.h)


 /*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
 /*---------------------------------------------------------------------------*/

#define UArg uintptr_t

/* PDMCC26XX_I2S functions */
void                 PDMCC26XX_I2S_Contiki_init(PDMCC26XX_I2S_Handle handle);
PDMCC26XX_I2S_Handle PDMCC26XX_I2S_Contiki_open(PDMCC26XX_I2S_Handle handle, PDMCC26XX_I2S_Params *params);
void                 PDMCC26XX_I2S_Contiki_close(PDMCC26XX_I2S_Handle handle);
bool                 PDMCC26XX_I2S_Contiki_startStream(PDMCC26XX_I2S_Handle handle);
bool                 PDMCC26XX_I2S_Contiki_stopStream(PDMCC26XX_I2S_Handle handle);
bool                 PDMCC26XX_I2S_Contiki_requestBuffer(PDMCC26XX_I2S_Handle handle, PDMCC26XX_I2S_BufferRequest *bufferRequest);
void                 PDMCC26XX_I2S_Contiki_releaseBuffer(PDMCC26XX_I2S_Handle handle, PDMCC26XX_I2S_BufferRelease *bufferRelease);

/* Internal functions */
static void PDMCC26XX_I2S_initHw(PDMCC26XX_I2S_Handle handle);
static bool PDMCC26XX_I2S_initIO(PDMCC26XX_I2S_Handle handle);
static void PDMCC26XX_I2S_hwiFxn (UArg arg);
static void PDMCC26XX_I2S_callback(PDMCC26XX_I2S_Handle handle, PDMCC26XX_I2S_StreamNotification *msg);
static void PDMCC26XX_I2S_deallocateBuffers(PDMCC26XX_I2S_Handle handle);
static bool PDMCC26XX_I2S_allocateBuffers(PDMCC26XX_I2S_Handle handle);

/*! Struct that contains an I2S queue element and a pointer to the PDM buffer it is responsible for */
typedef struct {
  QueueP_Elem    queueElement;       /*!< Queue element */
  void          *buffer;            /*!< Buffer pointer */
} PDMCC26XX_I2S_QueueNode;

PDMCC26XX_I2S_QueueNode *i2sBlockActive = NULL;     /* Reference to the element which is currently being filled by I2S DMA In */
PDMCC26XX_I2S_QueueNode *i2sBlockNext = NULL;       /* Reference to the next element which will be filled by I2S DMA In */

QueueP_Obj i2sBlockEmptyQueueStruct;
QueueP_Handle i2sBlockEmptyQueue;

QueueP_Obj i2sBlockFullQueueStruct;
QueueP_Handle i2sBlockFullQueue;


I2SControlTable g_ControlTable;



/*------------------------------------------------------------------------------------------*/
/* RTSC Adaptation */

#define Assert_isTrue(...) 

/*------------------------------------------------------------------------------------------*/

/*
 *  ======== PDMCC26XX_I2S_init ========
 *  @pre    Function assumes that the handle is not NULL
 */
void PDMCC26XX_I2S_Contiki_init(PDMCC26XX_I2S_Handle handle) {
    PDMCC26XX_I2S_Object         *object;

    Assert_isTrue(handle, NULL);

    /* Get the pointer to the object */
    object = handle->object;

    /* Mark the object as available */
    object->isOpen = false;

    /* Make sure struct in driverlib I2S driver is initialized */
    g_pControlTable = &g_ControlTable;
}

/*
 *  ======== PDMCC26XX_I2S_open ========
 *  @pre    Function assumes that the handle is not NULL
 */
PDMCC26XX_I2S_Handle PDMCC26XX_I2S_Contiki_open(PDMCC26XX_I2S_Handle handle, PDMCC26XX_I2S_Params *params) {
    /* Use union to save on stack allocation */
    HwiP_Params                   hwiParams;
    PDMCC26XX_I2S_Object         *object;
    PDMCC26XX_I2S_HWAttrs const  *hwAttrs;

    Assert_isTrue(params->blockCount >= 3, NULL);
    Assert_isTrue(handle, NULL);

    PRINTF("DMCC26XX_I2S_open [enter]\n");

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Check if the I2S is open already with the base addr. */
    if (object->isOpen == true) {
        return (NULL);
    }
    /* Mark the handle as in use */
    object->isOpen = true;

    /* Initialize the I2S object */
    object->requestMode                 = params->requestMode;
    object->requestTimeout              = params->requestTimeout;
    object->mallocFxn                   = params->mallocFxn;
    object->freeFxn                     = params->freeFxn;
    object->blockCount                  = params->blockCount;
    object->blockSizeInSamples          = params->blockSizeInSamples;
    object->currentStream               = params->currentStream;
    object->currentStream->status       = PDMCC26XX_I2S_STREAM_IDLE;

    /* The following are constants that apply to PDM */
    /* Notes:
     *   Master clock (MCLK) is not used by I2S hardware, so ignore.
     *   Bit clock (BCLK) drives the microphone bit sample clock and is the primary clock.
     *   Word clock (WCLK) controls how to group sampled microphone bits into a "word".
     *   
     *   BCLK controls the sample rate, and must be within the normal operating frequency of the microphone.
     *   WCLK is 16 because the driver works with 16-bit groupings.
     *   !Changing BCLK has consequence on the downsample filter in pdm2pcm_cc26x2_gnu.s, as it was designed
     *   to obtain 16 kHz, 16-bit PCM from a 1 MHz BCLK (good for sampling 8 kHz voice).
     *
     *   WCLK phase is "dual" to co-incide with the 2-channel (STEREO) setup.  All this means is that
     *   the microphone is sampled continuously and samples of the two channels are contiguously
     *   placed into the buffers (at least, that is the design intent here).  The microphone data is
     *   sampled on the rising edge of BCLK only (microphone is setup in Left channel mode).
     *
     *   The default parameters for the 1 MHz sample were:
     *         object->audioClkCfg.bclkDiv = 47;                                        
     *
     *   To work with MP34DT05, we need BCLK to be at least 1.2 MHz and no more than about 3.25 MHz.
     *   All I can do here, without understanding the decimation filter design enough to modify it,
     *   is to:
     *      1. Increase BCLK to 1.2 MHz
     *      2. Hope this doesn't affect the decimation filter too much.
     *
    */
    object->sampleRate = -1;                                                  /* If negative then use user configured clock division */
    object->audioClkCfg.wclkDiv = 16;                                            /* I2S Word Clock divider override*/
    object->audioClkCfg.sampleOnPositiveEdge = PDMCC26XX_I2S_SampleEdge_Postive; /* I2S Sample Edge */
    object->audioClkCfg.wclkPhase = PDMCC26XX_I2S_WordClockPhase_Dual;           /* I2S Word Clock Phase */
    object->audioClkCfg.wclkInverted = PDMCC26XX_I2S_ClockSource_Normal;         /* I2S Invert Word Clock */
    object->audioClkCfg.wclkSource = PDMCC26XX_I2S_WordClockSource_Int;          /* I2S Word Clock source */
    object->audioClkCfg.bclkDiv = 40;                                            /* I2S Bit Clock divider override for 1.2 MHz mic clock */
    object->audioClkCfg.reserved = 0;
    object->audioClkCfg.bclkSource = PDMCC26XX_I2S_BitClockSource_Int;           /* I2S Bit Clock source */
    object->audioClkCfg.mclkDiv = 6;                                             /* I2S Master Clock divider override */

    object->audioPinCfg.bitFields.ad1Usage = PDMCC26XX_I2S_ADUsageDisabled;      /* I2S AD1 usage (0: Disabled, 1: Input, 2: Output) */
    object->audioPinCfg.bitFields.enableMclkPin = PDMCC26XX_I2S_GENERIC_DISABLED;/* I2S Enable Master clock output on pin */
    object->audioPinCfg.bitFields.reserved = 0;
    object->audioPinCfg.bitFields.ad1NumOfChannels = 0;                          /* I2S AD1 number of channels (1 - 8). !Must match channel mask */
    object->audioPinCfg.bitFields.ad1ChannelMask = PDMCC26XX_I2S_DISABLED_MODE;  /* I2S AD1 Channel Mask */
    object->audioPinCfg.bitFields.ad0Usage = PDMCC26XX_I2S_ADUsageInput;         /* I2S AD0 usage (0: Disabled, 1: Input, 2: Output) */
    object->audioPinCfg.bitFields.enableWclkPin = PDMCC26XX_I2S_GENERIC_DISABLED;/* I2S Enable Word clock output on pin */
    object->audioPinCfg.bitFields.enableBclkPin = PDMCC26XX_I2S_GENERIC_ENABLED; /* I2S Enable Bit clock output on pin */
    object->audioPinCfg.bitFields.ad0NumOfChannels = 2;                          /* I2S AD0 number of channels (1 - 8). !Must match channel mask. \sa PDM_NUM_OF_CHANNELS */
    object->audioPinCfg.bitFields.ad0ChannelMask = PDMCC26XX_I2S_STEREO_MODE;    /* I2S AD0 Channel Mask */

    object->audioFmtCfg.wordLength = PDMCC26XX_I2S_WordLength16;                 /* Number of bits per word (8-24). Exact for single phase, max for dual phase */
    object->audioFmtCfg.sampleEdge = PDMCC26XX_I2S_PositiveEdge;                 /* Data and Word clock is samples, and clocked out, on opposite edges of BCLK */
    object->audioFmtCfg.dualPhase = PDMCC26XX_I2S_SinglePhase;                   /* Selects dual- or single phase format (0: Single, 1: Dual) */
    object->audioFmtCfg.memLen = PDMCC26XX_I2S_MemLen16bit;                      /* Size of each word stored to or loaded from memory (0: 16, 1: 24) */
    object->audioFmtCfg.dataDelay = PDMCC26XX_I2S_FormatLJF;                     /* Number of BCLK perids between a WCLK edge and MSB of the first word in a phase */


    /* Find out how many channels are In and Out respectively */
    uint8_t totalNumberOfChannelsIn = 0;
    totalNumberOfChannelsIn += object->audioPinCfg.bitFields.ad0NumOfChannels;

    object->blockSizeInBytes = (object->blockSizeInSamples * ( (object->audioFmtCfg.memLen) ? 3 : 2 ) * totalNumberOfChannelsIn);

    /* Setup queues now that we now whether they are needed */
    QueueP_construct(&i2sBlockFullQueueStruct, NULL);
    i2sBlockFullQueue = QueueP_handle(&i2sBlockFullQueueStruct);

    QueueP_construct(&i2sBlockEmptyQueueStruct, NULL);
    i2sBlockEmptyQueue = QueueP_handle(&i2sBlockEmptyQueueStruct);

    /* Try to allocate memory for the PDM buffers */
    if (!PDMCC26XX_I2S_allocateBuffers(handle)){

        return NULL;
    }

    /* Register power dependency - i.e. power up and enable clock for I2S. */
    Power_setDependency(hwAttrs->powerMngrId);

    /* Configure IOs after hardware has been initialized so that IOs aren't */
    /* toggled unnecessary and make sure it was successful */
    if (!PDMCC26XX_I2S_initIO(handle)) {
        /* Trying to use I2S driver when some other driver or application
        *  has already allocated these pins, error! */

        /* Deallocate all memory used by the driver */
        PDMCC26XX_I2S_deallocateBuffers(handle);

        /* Release power dependency - i.e. potentially power down serial domain. */
        Power_releaseDependency(hwAttrs->powerMngrId);

        /* Mark the module as available */
        object->isOpen = false;

        /* Signal back to application that I2S driver was not succesfully opened */
        return (NULL);
    }

    /* Create the Hwi for this I2S peripheral. */
    HwiP_Params_init(&hwiParams);
    hwiParams.arg = (UArg) handle;
    hwiParams.priority = hwAttrs->intPriority;
    HwiP_construct(&(object->hwi), (int) hwAttrs->intNum, PDMCC26XX_I2S_hwiFxn,
            &hwiParams);

    /* Create a semaphore to block task execution while stopping the stream */
    SemaphoreP_constructBinary(&(object->semStopping), 0);

    /* Check the transfer mode */
    if (object->requestMode == PDMCC26XX_I2S_MODE_BLOCKING) {
        /*
         *  Create a semaphore to block task execution for the duration of the
         *  I2S transfer.  This is a counting semaphore.
         */
        SemaphoreP_construct(&(object->blockComplete), 0, NULL);

        /* Store internal callback function */
        object->callbackFxn = PDMCC26XX_I2S_callback;
    }
    else {
        /* Check to see if a callback function was defined for async mode */
        Assert_isTrue(params->callbackFxn != NULL, NULL);

        /* Save the callback function pointer */
        object->callbackFxn = params->callbackFxn;
    }

    PRINTF("DMCC26XX_I2S_open [exit success]\n");

    return (handle);
}

/*
 *  ======== PDMCC26XX_I2S_deallocateBuffers ========
 *  @pre    Function assumes that the handle is not NULL
 */
static void PDMCC26XX_I2S_deallocateBuffers(PDMCC26XX_I2S_Handle handle){
    PDMCC26XX_I2S_Object            *object;

    Assert_isTrue(handle, NULL);
    Assert_isTrue(i2sBlockEmptyQueue, NULL);
    Assert_isTrue(i2sBlockFullQueue, NULL);

    /* Get the pointer to the object */
    object = handle->object;

    /* Empty the available queue and free the memory of the queue elements and the buffers */
    while (!QueueP_empty(i2sBlockEmptyQueue)) {
        PDMCC26XX_I2S_QueueNode *availableNode = (PDMCC26XX_I2S_QueueNode *)QueueP_dequeue(i2sBlockEmptyQueue);
        /* Free up memory used for PCM data buffer */
        object->freeFxn(availableNode->buffer, object->blockSizeInBytes);
        /* Then free up memory used by the queue element */
        object->freeFxn(availableNode, sizeof(PDMCC26XX_I2S_QueueNode));
    }

    /* Empty the ready queue and free the memory of the queue elements and the buffers */
    while (!QueueP_empty(i2sBlockFullQueue)) {
        PDMCC26XX_I2S_QueueNode *readyNode = (PDMCC26XX_I2S_QueueNode *)QueueP_dequeue(i2sBlockFullQueue);
        /* Free up memory used for PCM data buffer */
        object->freeFxn(readyNode->buffer, object->blockSizeInBytes);
        /* Then free up memory used by the queue element */
        object->freeFxn(readyNode, sizeof(PDMCC26XX_I2S_QueueNode));
    }

    /* Make sure we do not keep dangling pointers alive */
    if (i2sBlockActive){
        /* Free up memory used for PCM data buffer */
        object->freeFxn(i2sBlockActive->buffer, object->blockSizeInBytes);
        /* Then free up memory used by the queue element */
        object->freeFxn(i2sBlockActive, sizeof(PDMCC26XX_I2S_QueueNode));
        i2sBlockActive = NULL;
    }

    /* Make sure we do not keep dangling pointers alive */
    if (i2sBlockNext){
        /* Free up memory used for PCM data buffer */
        object->freeFxn(i2sBlockNext->buffer, object->blockSizeInBytes);
        /* Then free up memory used by the queue element */
        object->freeFxn(i2sBlockNext, sizeof(PDMCC26XX_I2S_QueueNode));
        i2sBlockNext = NULL;
    }

}

/*
 *  ======== PDMCC26XX_I2S_allocateBuffers ========
 *  @pre    Function assumes that the handle is not NULL
 */
static bool PDMCC26XX_I2S_allocateBuffers(PDMCC26XX_I2S_Handle handle){
    PDMCC26XX_I2S_Object            *object;

    Assert_isTrue(handle, NULL);
    Assert_isTrue(i2sBlockEmptyQueue, NULL);
    Assert_isTrue(i2sBlockFullQueue, NULL);

    /* Get the pointer to the object */
    object = handle->object;

    Assert_isTrue(object->isOpen, NULL);

    PRINTF("DMCC26XX_I2S_allocateBuffers [%d buffers @ %d bytes each]\n", object->blockCount, 
        object->blockSizeInBytes + sizeof(PDMCC26XX_I2S_QueueNode));

    /* Allocate the PDM block buffers and queue elements. The application provided malloc and free functions permit for both static and dynamic allocation.
     * The PDM block buffers are allocated individually. Despite the increased book-keeping overhead incurred for this in the malloc function,
     * the reduced likelyhood of heap fragmentation is worth it. When using a static allocation malloc implementation, there should be no extra cost incurred from
     * requesting block buffers individually. The same code that will be in the allocation function, would otherwise reside in this driver.
     */
    uint8_t i = 0;
    for (i = 0; i < object->blockCount; i++) {
        PDMCC26XX_I2S_QueueNode *tmpNode;
        tmpNode = object->mallocFxn(sizeof(PDMCC26XX_I2S_QueueNode));

        if(tmpNode){
            tmpNode->buffer = object->mallocFxn(object->blockSizeInBytes);
            if(tmpNode->buffer){
                /* Enqueue the PDM block in the available queue if the allocation was successful */
                QueueP_put(i2sBlockEmptyQueue, &tmpNode->queueElement);
            }
            else{
                /* Otherwise, free the node memory, unravel all other allocations, and fail the driver initialsation */
                object->freeFxn(tmpNode, sizeof(PDMCC26XX_I2S_QueueNode));
                PDMCC26XX_I2S_deallocateBuffers(handle);
                return false;
            }
        }
        else{
            /* Unravel all other allocations and fail the function */
            PDMCC26XX_I2S_deallocateBuffers(handle);
            return false;
        }
    }

    PRINTF("DMCC26XX_I2S_allocateBuffers [success]\n");

    return true;
}

/*
 *  ======== PDMCC26XX_I2S_close ========
 *  @pre    Function assumes that the handle is not NULL
 */
void PDMCC26XX_I2S_Contiki_close(PDMCC26XX_I2S_Handle handle) {
    PDMCC26XX_I2S_Object            *object;
    PDMCC26XX_I2S_HWAttrs const     *hwAttrs;

    /* Get the pointer to the object and hwAttrs */
    hwAttrs = handle->hwAttrs;
    object = handle->object;

    /* Deallocate pins */
    PIN_close(object->pinHandle);

    /* Disable the I2S */
    I2SDisable(hwAttrs->baseAddr);

    /* Destroy the Hwi */
    HwiP_destruct(&(object->hwi));

    /* Release power dependency on I2S. */
    Power_releaseDependency(hwAttrs->powerMngrId);

    /* Destroy the stopping semaphore */
    SemaphoreP_destruct(&(object->semStopping));

    if (object->requestMode == PDMCC26XX_I2S_MODE_BLOCKING) {
        /* Destroy the block complete semaphore */
        SemaphoreP_destruct(&(object->blockComplete));
    }

    /* Flush any unprocessed I2S data */
    PDMCC26XX_I2S_deallocateBuffers(handle);

    QueueP_destruct(&i2sBlockFullQueueStruct);
    QueueP_destruct(&i2sBlockEmptyQueueStruct);

    /* Mark the module as available */
    object->isOpen = false;
}

volatile int numHwiFxnInts = 0;
volatile uint32_t lastIntStatus = 0;
volatile int numHwiFxnDropped = 0;
volatile int numHwiFxnErr = 0;
/*
 *  ======== PDMCC26XX_I2S_hwiFxn ========
 *  ISR for the I2S
 */
static void PDMCC26XX_I2S_hwiFxn (UArg arg) {

    PDMCC26XX_I2S_StreamNotification *notification;
    PDMCC26XX_I2S_Object        *object;
    PDMCC26XX_I2S_HWAttrs const *hwAttrs;
    uint32_t                    intStatus;

    numHwiFxnInts++;

    /* Get the pointer to the object and hwAttrs */
    object = ((PDMCC26XX_I2S_Handle)arg)->object;
    hwAttrs = ((PDMCC26XX_I2S_Handle)arg)->hwAttrs;

    Assert_isTrue(i2sBlockEmptyQueue, NULL);
    Assert_isTrue(i2sBlockFullQueue, NULL);

    /* Get the interrupt status of the I2S controller */
    intStatus = I2SIntStatus(hwAttrs->baseAddr, true);
    I2SIntClear(hwAttrs->baseAddr, intStatus);

    lastIntStatus = intStatus;
#if 1
    if (intStatus & I2S_IRQMASK_AIF_DMA_IN) {
        Assert_isTrue(i2sBlockActive, NULL);

        /* Move completed buffer to ready queue */
        QueueP_put(i2sBlockFullQueue, &i2sBlockActive->queueElement);
        /* Setup next active buffer */
        i2sBlockActive = i2sBlockNext;
        /* Mark next buffer as empty*/
        i2sBlockNext = NULL;

        if (object->currentStream->status == PDMCC26XX_I2S_STREAM_STOPPING) {
            /* Part of shut down sequence*/
            object->currentStream->status = PDMCC26XX_I2S_STREAM_STOPPED;
        }
        else if (object->currentStream->status != PDMCC26XX_I2S_STREAM_STOPPED) {
            if (!QueueP_empty(i2sBlockEmptyQueue)) {
                /* There is an empty buffer available. */
                i2sBlockNext = (PDMCC26XX_I2S_QueueNode *)QueueP_get(i2sBlockEmptyQueue);
                object->currentStream->status = PDMCC26XX_I2S_STREAM_BUFFER_READY;
            }
            else {
                numHwiFxnDropped++;
                /* If there is no empty buffer available, there should be full buffers we could drop. */
                Assert_isTrue(!QueueP_empty(i2sBlockFullQueue), NULL);
                /* The PDM driver did not process the buffers in time. The I2S module needs to drop
                 * some old data and notify the PDM driver that it did so.
                 */
                i2sBlockNext = (PDMCC26XX_I2S_QueueNode *)QueueP_get(i2sBlockFullQueue);
                object->currentStream->status = PDMCC26XX_I2S_STREAM_BUFFER_DROPPED;
            }
            Assert_isTrue(i2sBlockNext, NULL);

            I2SPointerSet(hwAttrs->baseAddr, true, (uint32_t *)i2sBlockNext->buffer);

            /* Use a temporary stream pointer in case the callback function
            * attempts to perform another PDMCC26XX_I2S_bufferRequest call
            */
            notification = object->currentStream;

            /* Notify caller about availability of buffer */
            object->callbackFxn((PDMCC26XX_I2S_Handle)arg, notification);
        }
    }

    /* Error handling:
    * Overrun in the RX Fifo -> at least one sample in the shift
    * register has been discarded  */
    if (intStatus & I2S_IRQMASK_PTR_ERR) {
        numHwiFxnErr++;
        /* disable the interrupt */
        I2SIntDisable(hwAttrs->baseAddr, I2S_INT_PTR_ERR);
        /* Check if we are expecting this interrupt as part of stopping */
        if ( object->currentStream->status == PDMCC26XX_I2S_STREAM_STOPPED ) {
            /* This happened because PDMCC26XX_I2S_stopStream was called for some reason
             * Post the semaphore
             */
            SemaphoreP_post(&(object->semStopping));
        }
        else {
            __asm(" NOP");
            object->currentStream->status = PDMCC26XX_I2S_STREAM_ERROR;
            /* Use a temporary stream pointer in case the callback function
            * attempts to perform another PDMCC26XX_I2S_bufferRequest call
            */
            notification = object->currentStream;
            /* Notify caller about availability of buffer */
            object->callbackFxn((PDMCC26XX_I2S_Handle)arg, notification);
        }
    }
#endif
}

/*
 *  ======== PDMCC26XX_I2S_startStream ========
 *  @pre    Function assumes that handle is not NULL
 */
bool PDMCC26XX_I2S_Contiki_startStream(PDMCC26XX_I2S_Handle handle) {
    PDMCC26XX_I2S_Object        *object;
    PDMCC26XX_I2S_HWAttrs const *hwAttrs;

    Assert_isTrue(handle, NULL);
    Assert_isTrue(i2sBlockFullQueue, NULL);
    Assert_isTrue(i2sBlockEmptyQueue, NULL);

    PRINTF("PDMCC26XX_I2S_startStream [enter]\n");

    /* Get the pointer to the object and hwAttr*/
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    Assert_isTrue(object->isOpen, NULL);

    /* Checking if a transfer is in progress */
    if (object->currentStream->status != PDMCC26XX_I2S_STREAM_IDLE) {

        /* Flag that the transfer failed to start */
        object->currentStream->status = PDMCC26XX_I2S_STREAM_FAILED;

        /* Transfer is in progress */
        return false;
    }

    /* Make sure to flag that a stream is now active */
    object->currentStream->status = PDMCC26XX_I2S_STREAM_STARTED;

    i2sBlockActive = (PDMCC26XX_I2S_QueueNode *)QueueP_dequeue(i2sBlockEmptyQueue);
    i2sBlockNext = (PDMCC26XX_I2S_QueueNode *)QueueP_dequeue(i2sBlockEmptyQueue);

    /* Configure the hardware module */
    PDMCC26XX_I2S_initHw(handle);

    /* Configuring sample stamp generator will trigger the audio stream to start */
    I2SSampleStampConfigure(hwAttrs->baseAddr, true, false);

    /* Configure buffers */
    I2SBufferConfig(hwAttrs->baseAddr,
                    (uint32_t)(i2sBlockActive->buffer),
                    0, 
                    object->blockSizeInSamples,
                    PDMCC26XX_I2S_DEFAULT_SAMPLE_STAMP_MOD);

//#if 0  THIS WORKS TO AVOID CRASH

    /* Enable the I2S module. This will set first buffer and DMA length */
    I2SEnable(hwAttrs->baseAddr);
//#endif

//#if 0 // does this? YES this also avoids crash.
    /* Kick off clocks */
    PRCMAudioClockEnable();
    PRCMLoadSet();
//#endif
    /* Second buffer is then set in hardware after DMA length is set */
    I2SPointerSet(hwAttrs->baseAddr, true, (uint32_t *)i2sBlockNext->buffer);
#if 1 // does this? Yes
    /* Enable the RX overrun interrupt in the I2S module */
    I2SIntEnable(hwAttrs->baseAddr, I2S_INT_DMA_IN | I2S_INT_PTR_ERR);

    /* Clear internal pending interrupt flags */
    I2SIntClear(I2S0_BASE, I2S_INT_ALL);
#endif

    /* Enable samplestamp */
    I2SSampleStampEnable(hwAttrs->baseAddr);

    /* Clear potential pending I2S interrupt to CM3 */
    HwiP_clearInterrupt(INT_I2S_IRQ);
    /* Enable I2S interrupt to CM3 */
    HwiP_enableInterrupt(INT_I2S_IRQ);


    PRINTF("PDMCC26XX_I2S_startStream [exit success]\n");

    return true;
}

/*
 *  ======== PDMCC26XX_I2S_stopStream ========
 *  @pre    Function assumes that handle is not NULL
 */
bool PDMCC26XX_I2S_Contiki_stopStream(PDMCC26XX_I2S_Handle handle) {
    PDMCC26XX_I2S_Object                    *object;
    PDMCC26XX_I2S_HWAttrs const             *hwAttrs;

    Assert_isTrue(handle, NULL);

    PRINTF("DMCC26XX_I2S_stopStream [enter]\n");

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    Assert_isTrue(object->isOpen, NULL);

    /* Check if there is an active stream */
    if ( (object->currentStream->status == PDMCC26XX_I2S_STREAM_STOPPING) ||
         (object->currentStream->status == PDMCC26XX_I2S_STREAM_STOPPED) ||
         (object->currentStream->status == PDMCC26XX_I2S_STREAM_IDLE) ) {

        return false;
    }

    /* Begin stopping sequence, if not stopped because of error */
    if (object->currentStream->status != PDMCC26XX_I2S_STREAM_ERROR) {
        object->currentStream->status = PDMCC26XX_I2S_STREAM_STOPPING;

        /* Reenable the interrupt as it may have been disabled during an error*/
        I2SIntEnable(hwAttrs->baseAddr, I2S_INT_PTR_ERR);

        /* Wait for I2S module to complete all buffers*/
        if (SemaphoreP_OK != SemaphoreP_pend(&(object->semStopping), 40000)) {
            object->currentStream->status = PDMCC26XX_I2S_STREAM_FAILED_TO_STOP;
            return false;
        }
    }

    /* Flush the blockFullQueue and move those elements to the blockEmptyQueue.
     * Since this function shuts down the driver synchronously by letting it run out
     * of buffers, we do not need to worry about losing interesting data.
     */
    while (!QueueP_empty(i2sBlockFullQueue)) {
        PDMCC26XX_I2S_QueueNode *tmpNode = (PDMCC26XX_I2S_QueueNode *)QueueP_get(i2sBlockFullQueue);
        QueueP_put(i2sBlockEmptyQueue, &tmpNode->queueElement);
    }

    /* Disable the I2S module */
    I2SDisable(hwAttrs->baseAddr);

    /* Turn off clocks */
    PRCMAudioClockDisable();
    PRCMLoadSet();

    /* Disable and clear any pending interrupts */
    I2SIntDisable(hwAttrs->baseAddr, I2S_INT_ALL);
    I2SIntClear(hwAttrs->baseAddr, I2S_INT_ALL);
    HwiP_clearInterrupt(INT_I2S_IRQ);
    HwiP_disableInterrupt(INT_I2S_IRQ);

    /* Indicate we are done with this stream */
    object->currentStream->status = PDMCC26XX_I2S_STREAM_IDLE;

    PRINTF("DMCC26XX_I2S_stopStream [exit success]\n");

    /* Stream was successfully stopped */
    return true;
}

/*
 *  ======== PDMCC26XX_I2S_requestBuffer ========
 *  @pre    Function assumes that stream has started and that bufferRequest is not NULL
 */
bool PDMCC26XX_I2S_Contiki_requestBuffer(PDMCC26XX_I2S_Handle handle, PDMCC26XX_I2S_BufferRequest *bufferRequest) {
    PDMCC26XX_I2S_Object        *object;
    bool                        retVal = false;

    Assert_isTrue(handle, NULL);
    Assert_isTrue(bufferRequest, NULL);

    /* Get the pointer to the object */
    object = handle->object;

    Assert_isTrue(object->isOpen, NULL);

    if (object->requestMode == PDMCC26XX_I2S_MODE_BLOCKING) {
        if (SemaphoreP_OK != SemaphoreP_pend(&(object->blockComplete),
                    object->requestTimeout)) {
            /* Stop stream, if we experience a timeout */
            PDMCC26XX_I2S_Contiki_stopStream(handle);

            bufferRequest->status = object->currentStream->status;

            return false;
        }
    };
    bufferRequest->bufferHandleIn = NULL;
    /* When in callback mode we typically expect the user to call this
    * after being notified of available buffers. Hence we may directly
    * check queue and dequeue buffer
    */
    if (!QueueP_empty(i2sBlockFullQueue)) {
        PDMCC26XX_I2S_QueueNode *readyNode = (PDMCC26XX_I2S_QueueNode *)QueueP_get(i2sBlockFullQueue);
        bufferRequest->bufferIn = readyNode->buffer;
        bufferRequest->status = object->currentStream->status;
        bufferRequest->bufferHandleIn = readyNode;
        retVal = true;
    }


    return retVal;
}

/*
 *  ======== PDMCC26XX_I2S_releaseBuffer ========
 *  @pre    Function assumes bufferRelease contains a valid bufferHandle (identical to
 *          the one provided in the bufferRequest in PDMCC26XX_I2S_requestBuffer
 */
void PDMCC26XX_I2S_Contiki_releaseBuffer(PDMCC26XX_I2S_Handle handle, PDMCC26XX_I2S_BufferRelease *bufferRelease) {
    Assert_isTrue(handle, NULL);
    Assert_isTrue(bufferRelease, NULL);
    Assert_isTrue(bufferRelease->bufferHandleIn, NULL);
    Assert_isTrue(i2sBlockEmptyQueue, NULL);

    /* Place released buffer back in available queue */
    QueueP_put(i2sBlockEmptyQueue, &((PDMCC26XX_I2S_QueueNode *)bufferRelease->bufferHandleIn)->queueElement);
}

/*
 *  ======== PDMCC26XX_I2S_callback ========
 *  Callback function for when the I2S is in PDMCC26XX_I2S_MODE_BLOCKING
 *
 *  @pre    Function assumes that the handle is not NULL
 */
static void PDMCC26XX_I2S_callback(PDMCC26XX_I2S_Handle handle, PDMCC26XX_I2S_StreamNotification *msg) {
    PDMCC26XX_I2S_Object         *object;

    /* Get the pointer to the object */
    object = handle->object;

    /* Post the semaphore */
    SemaphoreP_post(&(object->blockComplete));
}

/*
*  ======== PDMCC26XX_I2S_hwInit ========
*  This functions initializes the I2S hardware module.
*
*  @pre    Function assumes that the I2S handle is pointing to a hardware
*          module which has already been opened.
*/
static void PDMCC26XX_I2S_initHw(PDMCC26XX_I2S_Handle handle) {
    PDMCC26XX_I2S_Object        *object;
    PDMCC26XX_I2S_HWAttrs const *hwAttrs;

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Similarly can't use PRINTF with interrupts shut off */
    //PRINTF("DMCC26XX_I2S_initHw [enter] object=%p hwAttrs=%p baseAddr=%p\n", object, hwAttrs, (void*)hwAttrs->baseAddr);

    /* Disable I2S operation */
    I2SDisable(hwAttrs->baseAddr);

    /* Configure Audio format */
    I2SAudioFormatConfigure(hwAttrs->baseAddr,
                            *(uint32_t *)&object->audioFmtCfg,
                            object->audioFmtCfg.dataDelay);

    /* Configure Channels */
    I2SChannelConfigure(hwAttrs->baseAddr,
                        object->audioPinCfg.driverLibParams.ad0,
                        object->audioPinCfg.driverLibParams.ad1);

    /* Configure Clocks*/
    uint32_t clkCfg = object->audioClkCfg.wclkSource;
    clkCfg |= (object->audioClkCfg.wclkInverted) ? I2S_INVERT_WCLK : 0;
    I2SClockConfigure(hwAttrs->baseAddr, clkCfg);
    uint32_t mstDiv = object->audioClkCfg.mclkDiv;
    uint32_t bitDiv = object->audioClkCfg.bclkDiv;
    uint32_t wordDiv = object->audioClkCfg.wclkDiv;
    clkCfg = (object->audioClkCfg.wclkPhase << PRCM_I2SCLKCTL_WCLK_PHASE_S);
    clkCfg |= (object->audioClkCfg.sampleOnPositiveEdge << PRCM_I2SCLKCTL_SMPL_ON_POSEDGE_S);
    if ( (object->sampleRate >= I2S_SAMPLE_RATE_16K) &&
        (object->sampleRate <= I2S_SAMPLE_RATE_48K)) {
        PRCMAudioClockConfigSet(clkCfg, object->sampleRate);
    }
    else{
        PRCMAudioClockConfigSetOverride(clkCfg, mstDiv, bitDiv, wordDiv);
    }
    /* TODO: Replace this with Driverlib code */
    HWREG(PRCM_BASE + PRCM_O_I2SBCLKSEL) = (object->audioClkCfg.bclkSource & PRCM_I2SBCLKSEL_SRC_M);
    /* Apply configuration */
    PRCMLoadSet();

    /* Disable I2S module interrupts */
    I2SIntDisable(hwAttrs->baseAddr, I2S_INT_ALL);
}

/*
*  ======== PDMCC26XX_I2S_initIO ========
*  This functions initializes the I2S IOs.
*
*  @pre    Function assumes that the I2S handle is pointing to a hardware
*          module which has already been opened.
*/
static bool PDMCC26XX_I2S_initIO(PDMCC26XX_I2S_Handle handle) {
    PDMCC26XX_I2S_Object        *object;
    PDMCC26XX_I2S_HWAttrs const *hwAttrs;
    PIN_Config                  i2sPinTable[6];
    uint8_t                     i = 0;

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Configure IOs */
    /* Build local list of pins, allocate through PIN driver and map HW ports */
    if (object->audioPinCfg.bitFields.enableMclkPin) {
      i2sPinTable[i++] = hwAttrs->mclkPin | IOC_STD_OUTPUT;
    }
    if (object->audioPinCfg.bitFields.enableWclkPin) {
      i2sPinTable[i++] = hwAttrs->wclkPin | IOC_STD_OUTPUT;
    }
    if (object->audioPinCfg.bitFields.enableBclkPin) {
      i2sPinTable[i++] = hwAttrs->bclkPin | IOC_STD_OUTPUT;
    }
    if (object->audioPinCfg.bitFields.ad0Usage == PDMCC26XX_I2S_ADUsageInput) {
      i2sPinTable[i++] = hwAttrs->ad0Pin | PIN_INPUT_EN | PIN_NOPULL;
    }
    else if (object->audioPinCfg.bitFields.ad0Usage == PDMCC26XX_I2S_ADUsageOutput) {
      i2sPinTable[i++] = hwAttrs->ad0Pin | IOC_STD_OUTPUT;
    }
    i2sPinTable[i++] = PIN_TERMINATE;

    /* Open and assign pins through pin driver */
    if (!(object->pinHandle = PIN_open(&(object->pinState), i2sPinTable))) {
      return false;
    }

    /* Set IO muxing for the I2S pins */
    if (object->audioPinCfg.bitFields.enableMclkPin) {
        PINCC26XX_setMux(object->pinHandle, hwAttrs->mclkPin,  IOC_PORT_MCU_I2S_MCLK);
    }
    if (object->audioPinCfg.bitFields.enableWclkPin) {
        PINCC26XX_setMux(object->pinHandle, hwAttrs->wclkPin,  IOC_PORT_MCU_I2S_WCLK);
    }
    if (object->audioPinCfg.bitFields.enableBclkPin) {
        PINCC26XX_setMux(object->pinHandle, hwAttrs->bclkPin,  IOC_PORT_MCU_I2S_BCLK);
    }
    if (object->audioPinCfg.bitFields.ad0Usage != PDMCC26XX_I2S_ADUsageDisabled) {
        PINCC26XX_setMux(object->pinHandle, hwAttrs->ad0Pin,  IOC_PORT_MCU_I2S_AD0);
    }

    return true;
}


/*
 *  ======== i2sPostNotify ========
 *  This functions is called to notify the I2S driver of an ongoing transition
 *  out of sleep mode.
 *
 *  @pre    Function assumes that the I2S handle (clientArg) is pointing to a
 *          hardware module which has already been opened.
 */
int i2sPostNotify(char eventType, uint32_t clientArg) {
    PDMCC26XX_I2S_Handle i2sHandle;

    /* Get the pointers to I2S objects */
    i2sHandle = (PDMCC26XX_I2S_Handle) clientArg;

    /* Reconfigure the hardware when returning from standby */
    PDMCC26XX_I2S_initHw(i2sHandle);

    return Power_NOTIFYDONE;
}
