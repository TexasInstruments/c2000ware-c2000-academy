//############################################################################
//
// FILE: lab_main.c
//
// TITLE: Lab - FSI communication
//
// C2K ACADEMY URL: #############################
//
//! \addtogroup academy_lab_list
//! <h1> Lab solution on Using Communication Peripherals </h1>
//!
//! The objective of this lab is to become familiar with the on-board FSI
//! (Fast Serial Interface) by sending and receiving data between the
//! receiver(FSIRX) and transmitter(FSITX) modules. During initialization,
//! the transmitter module will be configured to send a ping frame
//! periodically to service the receiver module's ping watch-dog timer.
//! Data frames will be triggered via software and the data transmitted
//! will be compared against the data received in the receiver ISR to check
//! for errors. Interrupts will be triggered on error events for both the
//! receiver and transmitter modules. the receiver and transmitter modules
//! will be connected via internal loop-back mode.
//!
//! \b External \b Connections \n
//!  - None.
//!
//! \b Watch \b Variables \n
//!  - None.
//!
//############################################################################
// $Copyright:


//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"

//
//Globals
//
uint16_t txBufData[16];                     //used to write to FSI transmit buffer
uint16_t rxDataArray[16];                   //buffer used to access FSI receive buffer
uint32_t dataFrameCntr = 0;                 //counts how many data frames have been received
uint16_t txEventSts = 0, rxEventSts = 0;    //captures FSITX/FSIRX event status in case of error
uint16_t txIntReceived = 0;                 //flag to signal TX interrupt has occurred
uint16_t rxIntReceived = 0;                 //flag to signal RX interrupt has occurred
uint32_t softwareTimeoutCntr = 0x100000;    //software watchdog counter in case interrupts fail

//
// Function Prototypes
//
void prepareTxBufData(void);

//
// Main
//
void main(void)
{
    //
    // CPU Initialization
    //
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    //
    // Configure GPIO pins
    //
    Device_initGPIO();

    //
    // Initialize the FSI
    //
    Board_init();

    //
    //Insert delay before enabling FSIRX ping watch-dog to allow FSITX
    //module to start sending ping frames
    //
    DEVICE_DELAY_US(1000);
    FSI_enableRxPingWatchdog(FSIRXA_BASE, 81920);

    //
    // Enable global interrupts and real-time debug
    //
    EINT;
    ERTM;
    while(1)
    {
        //
        // Write data into TX buffer
        //
        prepareTxBufData();
        FSI_writeTxBuffer(FSITXA_BASE, txBufData, myFSITX0_nWords, 0U);

        //
        // Start Transfer
        //
        FSI_startTxTransmit(FSITXA_BASE);

        //
        //Wait for FSITX and FSIRX ISRs to complete
        //
        while (txIntReceived == 0 || rxIntReceived == 0)
        {
            //
            //Decrement software watch-dog counter to detect missing interrupts
            //
            softwareTimeoutCntr--;

            //
            //If software counter reaches zero, terminate program
            //
            if (softwareTimeoutCntr == 0){
                ESTOP0;
            }
        }

        //
        // Re-initialize flags, counter and buffer pointers before next transmission
        //
        txIntReceived = 0;
        rxIntReceived = 0;
        softwareTimeoutCntr = 0x100000;
        FSI_setTxBufferPtr(FSITXA_BASE, 0U);
        FSI_setRxBufferPtr(FSIRXA_BASE, 0U);
    }

}

//
//Function to generate new values to write to transmit buffer
//
void prepareTxBufData(void)
{
    uint16_t i;

    for(i = 0; i < myFSITX0_nWords; i++)
    {
        txBufData[i] = txBufData[i] + 1;
    }
}

//
// fsiTxInt1ISR - FSI Tx Interrupt on INsT1 line
//
__interrupt void fsiTxInt1ISR(void)
{
    //
    //Set flag that interrupt has occurred, capture FSITX event status.
    txIntReceived = 1;
    txEventSts = FSI_getTxEventStatus(FSITXA_BASE);

    //
    // Clear the interrupt flag and issue ACK
    //
    FSI_clearTxEvents(FSITXA_BASE, FSI_TX_EVTMASK);
    Interrupt_clearACKGroup(INT_myFSITX0_1_INTERRUPT_ACK_GROUP);
}

//
// fsiTxInt2ISR - FSI Tx Interrupt on INT2 line
//
__interrupt void fsiTxInt2ISR(void)
{
    txEventSts = FSI_getTxEventStatus(FSITXA_BASE);

    //
    // INT2 line is set to fire for error events, stop immediately. Actual Error
    // is captured in txEventSts for debug
    //
    ESTOP0;
}

//
// fsiRxInt1ISR - FSI Rx Interrupt on INT1 line
//
__interrupt void fsiRxInt1ISR(void)
{
    //
    //Set flag that interrupt has occurred, capture FSIRX event status, increment data frame count.
    //
    rxIntReceived = 1;
    rxEventSts = FSI_getRxEventStatus(FSIRXA_BASE);
    dataFrameCntr++;

    //
    //Transfer data from receive buffer to array
    //
    FSI_readRxBuffer(FSIRXA_BASE, rxDataArray, myFSIRX0_nWords, 0);

    //
    //Validate that data transmitted matches data received, otherwise terminate program.
    //
    uint16_t i;
    for(i = 0; i < myFSIRX0_nWords; i++)
    {
        if(rxDataArray[i] != txBufData[i])
        {
            ESTOP0;
        }
    }

    //
    // Clear the interrupt flag and issue ACK
    //
    FSI_clearRxEvents(FSIRXA_BASE,rxEventSts);
    Interrupt_clearACKGroup(INT_myFSIRX0_1_INTERRUPT_ACK_GROUP);
}

//
// fsiRxInt2ISR - FSI Rx Interrupt on INT2 line
//
__interrupt void fsiRxInt2ISR(void)
{
    rxEventSts = FSI_getRxEventStatus(FSIRXA_BASE);

    //
    // INT2 line is set to fire for error events, stop immediately. Error
    // is captured in rxEventSts for debug
    //
    ESTOP0;
}

//
// End of File
//
