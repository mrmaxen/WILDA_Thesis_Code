/*

    Responder for DS-TWR Using multiple anchors

    Created By Max Nilsson, Based on DecaWave Examples
	
	For use with DW1000Tag.c

	
*/

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <deca_device_api.h>
#include <deca_regs.h>
#include <sleep.h>
#include <msp432_setup.h>


// Antenna delays; needs to be properly calibrated, 16436 is default value
#define TX_ANTENNA_DELAY 16436
#define RX_ANTENNA_DELAY 16436
typedef unsigned long long uint64;
typedef signed long long int64;

typedef struct {
    uint32          sysStatus ;     // last read of SYS_STATUS register
    int8            rxFlag ;        // Flag to indicate receive, but not read
    uint8           devAddressLow ; // first byte of address
    uint8           devAddressHigh ;// second byte of address
    uint_fast8_t    spiPort ;       // port of SS-pin
    uint_fast16_t   spiPin ;        // SS-pin
    uint16          antennaDelay ;
    double          lastDist ;
    uint64          ts_1 ;          // first timestamp
    uint64          ts_2 ;          // second timestamp
    uint64          ts_3 ;          // third timestamp
} deviceData ;

static dwt_config_t config = {
    2,                  //Channel number
    DWT_PRF_64M,        // Pulse Repetition Frequency
    DWT_PLEN_2048,      // Preamble length, TX
    DWT_PAC64,          // Preamble Acquisition Chunk size, RX
    9,                  // TX preamble code
    9,                  // RX preamble code
    1,                  // 0 to use standard SFD, 1 to use non-standard
    DWT_BR_110K,        // Data Rate
    DWT_PHRMODE_STD,    // PHY header mode
    (2049 + 64 - 32)    // SFD timeout (preamble length +1 + SFD length - PAC size), RX
};

static deviceData deviceA = {
    0,//sysstatus
    0,//rxFlag
    0xA1,
    0xA2,
    GPIO_PORT_P2,
    GPIO_PIN3,
    16545,
    0,
    0,
    0,
    0
};

static deviceData deviceB = {
    0,
    0,
    0xB1,
    0xB2,
    GPIO_PORT_P2,
    GPIO_PIN4,
    16550,
    0,
    0,
    0,
    0
};

static deviceData deviceC = {
    0,
    0,
    0xC1,
    0xC2,
    GPIO_PORT_P2,
    GPIO_PIN7,
    16620,
    0,
    0,
    0,
    0
};




// Transmission frame function codes
#define REQ_MSG 0xE0
#define ACK_MSG 0xE1
#define FINAL_MSG 0xE2
#define EOE_MSG 0xEE

// Common bytes in transmission frames that can be used for checks
#define SEQ_NMB_IDX 2
#define DEST_LOW 5
#define DEST_HIGH 6
#define SRC_LOW 7
#define SRC_HIGH 8
#define FNC_CODE_IDX 9

// Bytes in data message
#define REQ_TX_TS_IDX 10
#define RXA_TS_IDX 15
#define RXB_TS_IDX 20
#define RXC_TS_IDX 25
#define FINAL_TX_TS_IDX 30

// Transmission frames, bytes set to comply with IEEE 802.15.4
/*
    byte 0-1: frame control (0x8841 for data frame using 16 bit addressing)
    byte 2:   sequence number, incremented for each new frame
    byte 3-4: PAN ID (0xDECA)
    byte 5-6: Destination Address (Low-High bitorder)
    byte 7-8: Source Address
    byte 9:   function code, indicates which message in the process it is

    Each timestamp is 5 bytes

    every frame ends with two empty bytes that will be used by the DW1000 as checksum
*/
// message to initiate ranging
static uint8 ackMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xE1, 0, 0};

static uint8 eoeMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xEE, 0, 0};

// final message containing timestamps recorded on intiator (tx1a, rx2a, tx3a)
//(not used by responder)
//static uint8 finalMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE1, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0};



// Hold copy of status register here, useful for debugging
//static uint32 status_reg = 0;

// Buffer to hold incoming data
#define FRAME_LEN_MAX 127
static uint8 rxBuffer[FRAME_LEN_MAX];

static uint16 frame_len = 0;
// global rx_flag
static uint8 rx_flag = 0;
uint32_t deviceId;
static bool eoe = false;
static uint32 timerValue;
static uint16 antDly;

#define COUNT_VAL 1000
static double distA[COUNT_VAL];
static double distB[COUNT_VAL];
static double distC[COUNT_VAL];

static uint16 corrCount = 0;

// Timestamp containers

static uint64 rx2a;
static uint64 tx3a;
static uint64 tx1a;
/*static uint64 rx1b;
static uint64 tx2b;
static uint64 rx3b;*/

// Speed of light in air, in metres per second.
#define SPEED_OF_LIGHT 299702547

// Declaration of static functions.
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void data_msg_get_ts(const uint8 *ts_field, uint64 *ts);
static int send_Data(uint16 txFrameLength, uint8 *txFrameBytes, uint16 txBufferOffset, int ranging, uint8 mode);
static void calcDistance(deviceData *device);
static double diff(uint64 a, uint64 b);

void main()
{
    // initiate processor
    peripherals_init();

    reset_DW1000(); // Reset all chips at once
    int ii;
    for (ii = 0; ii<3; ii++)
    {
        switch (ii){
            case 0:
                spi_select_slave(deviceA.spiPort, deviceA.spiPin);
                antDly = deviceA.antennaDelay;
            break;
            case 1:
                spi_select_slave(deviceB.spiPort, deviceB.spiPin);
                antDly = deviceB.antennaDelay;
            break;
            case 2:
                spi_select_slave(deviceC.spiPort, deviceC.spiPin);
                antDly = deviceC.antennaDelay;
            break;

        }

        // reset DW1000 before init


        deviceId = dwt_readdevid();
            printf("Device %i ID is: %X \n", ii, deviceId);
        // if SPI frequency is higher than 2MHz, needs to be lowered before initialization
        if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
        {
            puts("Init Failed");
            while(1)
            {};
        }
        // configure using set parameters
        dwt_configure(&config);

        // set antenna delays
        dwt_setrxantennadelay(antDly);
        dwt_settxantennadelay(antDly);
    }
    // main loop start
    while(1)
    {
        // enable receive on all devices
        for (ii = 0; ii<3; ii++)
        {
            switch (ii){
                case 0:
                    spi_select_slave(deviceA.spiPort, deviceA.spiPin);
                break;
                case 1:
                    spi_select_slave(deviceB.spiPort, deviceB.spiPin);
                break;
                case 2:
                    spi_select_slave(deviceC.spiPort, deviceC.spiPin);
                break;
            }
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        // Poll for receive event in any device
        // This assumes that if any device has received, all devices have;
        // Which should be true given the small distance between them, unless an error has occurred somewhere
        init_timer();
        start_timer();
        while(!(deviceA.rxFlag && deviceB.rxFlag && deviceC.rxFlag))
        {
            if(!deviceA.rxFlag)
            {
                spi_select_slave(deviceA.spiPort, deviceA.spiPin);
                if((deviceA.sysStatus = dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_RXFCG))
                {
                    deviceA.rxFlag = 1;
                }
            }
            if(!deviceB.rxFlag)
            {
                spi_select_slave(deviceB.spiPort, deviceB.spiPin);
                if (deviceB.sysStatus = dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_RXFCG)
                {
                    deviceB.rxFlag = 1;
                }
            }
            if(!deviceC.rxFlag)
            {
                spi_select_slave(deviceC.spiPort, deviceC.spiPin);
                if (deviceC.sysStatus = dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_RXFCG)
                {
                    deviceC.rxFlag = 1;
                }
            }
            timerValue = check_timer();
            if( timerValue == 0x00)
            {
           //     printf("Timeout break\n");
               // eoe = true;

                spi_select_slave(deviceA.spiPort, deviceA.spiPin);
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
               // deviceA.rxFlag = 0;
                spi_select_slave(deviceB.spiPort, deviceB.spiPin);
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
               // deviceB.rxFlag = 0;
                spi_select_slave(deviceC.spiPort, deviceC.spiPin);
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
               // deviceC.rxFlag = 0;
                break;
            }
        } // receive poll end

        if(1)
        {

            rx_flag = 0;
            //Same message goes to all devices, read once
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= FRAME_LEN_MAX)
            {
                dwt_readrxdata(rxBuffer, frame_len, 0);
            }
        //    puts("Received\n");
        // read deviceA and reply-------------------------------------------------------
            spi_select_slave(deviceA.spiPort, deviceA.spiPin);
            if(deviceA.rxFlag)//deviceA.sysStatus = dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_RXFCG)
            {
                deviceA.rxFlag = 0;
                // clear good rx frame in dw1000
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);



                if(rxBuffer[FNC_CODE_IDX] == REQ_MSG)
                {
                    // get return address from rxBuffer
                   // ackMsg[DEST_LOW] = rxBuffer[SRC_LOW];
                   // ackMsg[DEST_HIGH] = rxBuffer[SRC_HIGH];
                    ackMsg[SRC_LOW] = deviceA.devAddressLow;
                    ackMsg[SRC_HIGH] = deviceA.devAddressHigh;
                //    puts("A sending ACK\n");
                    // send ACK msg
                    send_Data(sizeof(ackMsg), ackMsg, 0, 0, DWT_START_TX_IMMEDIATE);
                    // poll to check to see that the message was sent correctly
                    while(!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                    { };

                    // Clear TX frame sent event
                   // dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                    deviceA.ts_1 = get_rx_timestamp_u64();

                } else if (rxBuffer[FNC_CODE_IDX] == FINAL_MSG)
                {
                    deviceA.ts_2 = get_tx_timestamp_u64();
                    deviceA.ts_3 = get_rx_timestamp_u64();
                    data_msg_get_ts(&rxBuffer[REQ_TX_TS_IDX], &tx1a);
                    data_msg_get_ts(&rxBuffer[FINAL_TX_TS_IDX], &tx3a);
                   // calcDistance(&deviceA);
                  //  printf("%X:\t%3.2f\t", deviceA.devAddressLow, deviceA.lastDist);
                    eoe = true;
                }
            } else {
                deviceA.rxFlag = 0;
              //  printf("%X:\t--.--\t", deviceA.devAddressLow);
                deviceA.lastDist = 0.0;
                /* Clear RX error/timeout events in the DW1000 status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

                /* Reset RX to properly reinitialise LDE operation. */
              //  dwt_rxreset();
                eoe = true;
            }
        // read deviceB and reply-----------------------------------------------------
            spi_select_slave(deviceB.spiPort, deviceB.spiPin);
            //while(!(deviceB.sysStatus = dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_RXFCG))
           // {};
            if(deviceB.rxFlag)//deviceB.sysStatus = dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_RXFCG )
            {
                deviceB.rxFlag = 0;
                // clear good rx frame in dw1000
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

               /* frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
                if (frame_len <= FRAME_LEN_MAX)
                {
                    dwt_readrxdata(rxBuffer, frame_len, 0);
                }*/

                if(rxBuffer[FNC_CODE_IDX] == REQ_MSG)
                {
                   // ackMsg[DEST_LOW] = rxBuffer[SRC_LOW];
                   // ackMsg[DEST_HIGH] = rxBuffer[SRC_HIGH];
                    ackMsg[SRC_LOW] = deviceB.devAddressLow;
                    ackMsg[SRC_HIGH] = deviceB.devAddressHigh;
                 //   puts("B sending ACK\n");
                    // send ACK msg
                    send_Data(sizeof(ackMsg), ackMsg, 0, 0, DWT_START_TX_IMMEDIATE);
                    // poll to check to see that the message was sent correctly
                    while(!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                    { };

                    // Clear TX frame sent event
                  //  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                    deviceB.ts_1 = get_rx_timestamp_u64();

                } else if (rxBuffer[FNC_CODE_IDX] == FINAL_MSG)
                {
                    deviceB.ts_2 = get_tx_timestamp_u64();
                    deviceB.ts_3 = get_rx_timestamp_u64();
                   // calcDistance(&deviceB);
                 //   printf("%X\t%3.2f\t", deviceB.devAddressLow, deviceB.lastDist);
                    eoe = true;
                }
            } else {
                deviceB.rxFlag = 0;
               // printf("%X:\t--.--\t", deviceB.devAddressLow);
                deviceB.lastDist = 0.0;
                /* Clear RX error/timeout events in the DW1000 status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

                /* Reset RX to properly reinitialise LDE operation. */
               // dwt_rxreset();
                eoe = true;
            }
        // read deviceC and reply--------------------------------------------------------
            spi_select_slave(deviceC.spiPort, deviceC.spiPin);
           // while(!(deviceC.sysStatus = dwt_read32bitreg(SYS_STATUS_ID) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
            //{};
            if(deviceC.rxFlag)//deviceC.sysStatus = dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_RXFCG )
            {
                deviceC.rxFlag = 0;
                // clear good rx frame in dw1000
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

                /*frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
                if (frame_len <= FRAME_LEN_MAX)
                {
                    dwt_readrxdata(rxBuffer, frame_len, 0);
                }*/

                if(rxBuffer[FNC_CODE_IDX] == REQ_MSG)
                {
                    //ackMsg[DEST_LOW] = rxBuffer[SRC_LOW];
                   // ackMsg[DEST_HIGH] = rxBuffer[SRC_HIGH];
                    ackMsg[SRC_LOW] = deviceC.devAddressLow;
                    ackMsg[SRC_HIGH] = deviceC.devAddressHigh;
                   // puts("C sending ACK\n");
                    // send ACK msg
                    send_Data(sizeof(ackMsg), ackMsg, 0, 0, DWT_START_TX_IMMEDIATE);
                    // poll to check to see that the message was sent correctly
                    while(!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                    { };

                    // Clear TX frame sent event
                   // dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                    deviceC.ts_1 = get_rx_timestamp_u64();

                } else if (rxBuffer[FNC_CODE_IDX] == FINAL_MSG)
                {
                    deviceC.ts_2 = get_tx_timestamp_u64();
                    deviceC.ts_3 = get_rx_timestamp_u64();
                    calcDistance(&deviceA);
                    calcDistance(&deviceB);
                    calcDistance(&deviceC);
                    printf("%X\t%3.2f\t%X\t%3.2f\t%X\t%3.2f\n", deviceA.devAddressLow, deviceA.lastDist
                                                        , deviceB.devAddressLow, deviceB.lastDist
                                                            , deviceC.devAddressLow, deviceC.lastDist);
                    eoe = true;
                }
            } else {
                deviceC.rxFlag = 0;
               // printf("%X:\t--.--\n", deviceC.devAddressLow);
                deviceC.lastDist = 0.0;
                printf("%X\t%3.2f\t%X\t%3.2f\t%X\t%3.2f\n", deviceA.devAddressLow, deviceA.lastDist
                                                    , deviceB.devAddressLow, deviceB.lastDist
                                                        , deviceC.devAddressLow, deviceC.lastDist);
                /* Clear RX error/timeout events in the DW1000 status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

                /* Reset RX to properly reinitialise LDE operation. */
               // dwt_rxreset();
                eoe = true;
            }
        }

        if( (deviceA.lastDist>0) && (deviceB.lastDist>0) &&  (deviceC.lastDist>0)  )
        {
            if ((deviceA.lastDist<1000) && (deviceB.lastDist<1000) && (deviceC.lastDist<1000) )
            {
                distA[corrCount] = deviceA.lastDist;
                distB[corrCount] = deviceB.lastDist;
                distC[corrCount] = deviceC.lastDist;

                corrCount++;

                if(corrCount == COUNT_VAL-1)
                {
                    double avgA = 0.0, avgB = 0.0, avgC = 0.0;
                    int i;
                    for(i = 0; i < COUNT_VAL; i++)
                    {
                        avgA += distA[i];
                        avgB += distB[i];
                        avgC += distC[i];
                    }
                    avgA = avgA/COUNT_VAL;
                    avgB = avgB/COUNT_VAL;
                    avgC = avgC/COUNT_VAL;

                    printf("Average distances over %u measurements\n", COUNT_VAL);
                    printf("A: %3.2f\tB: %3.2f\tC: %3.2f\n",avgA, avgB, avgC);
                    while(1)
                    {};
                }
            }
        }

        if(eoe)
        {
            spi_select_slave(deviceA.spiPort, deviceA.spiPin);
            // send EOE message
            send_Data(sizeof(eoeMsg), eoeMsg, 0, 0, DWT_START_TX_IMMEDIATE);

            // poll to check to see that the message was sent correctly
            while(!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
            { };

            // Clear TX frame sent event
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
            eoe = false;
        }
    } // inf while end
}// main end

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn calcDistance()
 *
 * @brief Calculate distance from tag to device
 *
 * @param  deviceData data struct of device
 *
 * @return  none
 */
static void calcDistance(deviceData *device)
{
    double tRound1, tRound2, tReply1, tReply2, tof, distance;
    int64 tof_dtu;

    switch (device->devAddressLow){
        case 0xA1:
            data_msg_get_ts(&rxBuffer[RXA_TS_IDX], &rx2a);
        break;
        case 0xB1:
            data_msg_get_ts(&rxBuffer[RXB_TS_IDX], &rx2a);
        break;
        case 0xC1:
            data_msg_get_ts(&rxBuffer[RXC_TS_IDX], &rx2a);
        break;
    }


    tRound1 = diff(rx2a, tx1a);
    tRound2 = diff(device->ts_3, device->ts_2);

    tReply1 = diff(device->ts_2, device->ts_1);
    tReply2 = diff(tx3a, rx2a);

    tof_dtu = (int64)( ( tRound1 * tRound2 - tReply1 * tReply2 )/(tRound1 + tRound2 + tReply1 + tReply2) );
    tof = tof_dtu * DWT_TIME_UNITS;
    distance = tof * SPEED_OF_LIGHT;

    device->lastDist = distance;


}

double diff(uint64 a, uint64 b)
{
    double retval;
    if(a > b)
    {
        retval = a-b;
    } else {
        retval = (0xFFFFFFFFFF-b) + a;
    }
    return retval;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn calcCoords()
 *
 * @brief Calculate coordinates using Linear least squares
 *
 * @param  none
 *
 * @return  none
 */
/*static void calcCoords()
{
    double r1, r2, r3, x1, x2, x3, y1, y2, y3;
    double b1, b2, a11, a12, a21, a22, coeff, x, y;
    r1 = deviceA.lastDist;
    x1 = deviceA.xCoord;
    y1 = deviceA.yCoord;

    r2 = deviceB.lastDist;
    x2 = deviceB.xCoord;
    y2 = deviceB.yCoord;

    r3 = deviceC.lastDist;
    x3 = deviceC.xCoord;
    y3 = deviceC.yCoord;

    b1 = pow(r1,2) - pow(x1,2) - pow(y1,2) - pow(r3,2) + pow(x3,2) + pow(y3,2);
    b2 = pow(r2,2) - pow(x2,2) - pow(y2,2) - pow(r3,2) + pow(x3,2) + pow(y3,2);

    a11 = 2*(x3 - x1);
    a12 = 2*(y3 - y1);
    a21 = 2*(x3 - x2);
    a22 = 2*(y3 - y2);

    coeff = 1.0/( (pow(a11,2) + pow(a21,2))*(pow(a12,2) + pow(a22,2)) -  pow((a11*a12+a21*a22),2));

    x = coeff * (  b1 * ( a11*(pow(a12,2)+pow(a22,2)) + a12*(-a11*a12-a21*a22))  +
            b2 * ( a21*(pow(a12,2)+pow(22,2)) a22*(-a11*a12-a21*a22)) );

    y = coeff * (b1 * ( a11*(-a11*a12-a21*a22) + a12*(pow(a11,2)+pow(a21,2)))  +
            b2 * ( a21*(-a11*a12-a21*a22) + a22*(pow(a11,2) + pow(a21,2))));

    printf("Target Coordinates: (%3.2f, %3.2f)\n", x, y);
}*/

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn send_Data()
 *
 * @brief wrapper function for sending data
 *
 * @param  see dwt function descriptions
 *
 * @return  _ret DWT_ERROR if dwt_starttx fails, DWT_SUCCESS otherwise. Can only fail when using delayed TX
 */
static int send_Data(uint16 txFrameLength, uint8 *txFrameBytes, uint16 txBufferOffset, int ranging, uint8 mode)
{
    int _ret;
    // frame sequence number
    static uint8 frameSeqNmb = 0;
    txFrameBytes[SEQ_NMB_IDX] = frameSeqNmb;
    dwt_writetxdata(txFrameLength, txFrameBytes, txBufferOffset);
    dwt_writetxfctrl(txFrameLength, txBufferOffset, ranging);
    _ret = dwt_starttx(mode);
    frameSeqNmb++;
    return _ret;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
static void data_msg_get_ts(const uint8 *ts_field, uint64 *ts)
{
    int i;
    *ts = 0;
    for (i = 4; i >= 0; i--)
    {

            *ts = *ts << 8;
            *ts += ts_field[i];

    }
}
