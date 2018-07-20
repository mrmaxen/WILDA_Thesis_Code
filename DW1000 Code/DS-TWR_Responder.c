/*

    Responder for DS-TWR

    Created By Max Nilsson, Based on DecaWave Examples

    For use with DS-TWR_Initiator.c

*/

#include <stdio.h>
#include <string.h>
#include <math.h> // ADDED

#include <deca_device_api.h>
#include <deca_regs.h>
#include <sleep.h>
#include <msp432_setup.h>

typedef unsigned long long uint64;
typedef signed long long int64;

// ADDED
 struct deviceData {
    uint32          sysStatus ;     // last read of SYS_STATUS register
    int8            rxFlag ;        // Flag to indicate receive, but not read
    uint8           devAddressLow ; // first byte of address
    uint8           devAddressHigh ;// second byte of address
    uint_fast8_t    spiPort ;       // port of SS-pin
    uint_fast16_t   spiPin ;        // SS-pin
    uint_fast8_t    irqPort ;       // port of IRQ-pin
    uint_fast16_t   irqPin ;        // IRQ-pin
    double          xCoord ;        // relative x-coordinate
    double          yCoord ;        // relative y-coordinate
    double          lastDist ;
    uint64          ts_1 ;          // first timestamp
    uint64          ts_2 ;          // second timestamp
    uint64          ts_3 ;          // third timestamp
}  ;

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

// ADDDED
static struct deviceData deviceA = {
    0,
    0,
    0xA1,
    0xA2,
    GPIO_PORT_P2,
    GPIO_PIN3,
    GPIO_PORT_P5,
    GPIO_PIN0,
    0.0,
    3.0,
    0,
    0,
    0,
    0
};

/* Hold copy of diagnostics data so that it can be examined at a debug breakpoint. */
    static dwt_rxdiag_t rx_diag;

// Antenna delays; needs to be properly calibrated, 16436 is default value
#define TX_ANTENNA_DELAY 32770
#define RX_ANTENNA_DELAY 32770

// Transmission frame function codes
#define REQ_MSG 0xE0
#define ACK_MSG 0xE1
#define FINAL_MSG 0xE2
#define EOE_MSG 0xEE

// Common bytes in transmission frames that can be used for checks
#define SEQ_NMB_IDX 2
#define FNC_CODE_IDX 9

// Bytes in data message
#define TX1A_TS_IDX 10
#define RX2A_TS_IDX 15
#define TX3A_TS_IDX 20

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

#define TX_DELAY 10000

// Transmission frames, bytes set to comply with IEEE 802.15.4
/*
    byte 0-1: frame control (0x8841 for data frame using 16 bit addressing)
    byte 2:   sequence number, incremented for each new frame
    byte 3-4: PAN ID (0xDECA)
    byte 5-6: Destination Address (this is a simple first test, currently hardcoded)
    byte 7-8: Source Address
    byte 9:   function code, indicates which message in the processs it is

    Each timestamp is 5 bytes

    every frame ends with two empty bytes that will be used by the DW1000 as checksum
*/
// message to initiate ranging
static uint8 ackMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0};
// End of exchange message
static uint8 eoeMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xEE, 0, 0};
// final message containing timestamps recorded on intiator (tx1a, rx2a, tx3a)
//(not used by responder)
//static uint8 finalMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE1, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0};

#define MAX_RANGE_COUNT 1000
static double sumDist;
static double rangeArray[MAX_RANGE_COUNT];

static uint16 rangeCount = 0;

// Hold copy of status register here, useful for debugging
static uint32 status_reg = 0;

// Buffer to hold incoming data
#define FRAME_LEN_MAX 127
static uint8 rxBuffer[FRAME_LEN_MAX];

static uint16 frame_len = 0;
uint32_t deviceId;


// Timestamp containers
static uint64 rx2a;
static uint64 tx3a;
static uint64 tx1a;
static uint64 rx1b;
static uint64 tx2b;
static uint64 rx3b;

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

// Declaration of static functions.
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void data_msg_get_ts(const uint8 *ts_field, uint64 *ts);
static int send_Data(uint16 txFrameLength, uint8 *txFrameBytes, uint16 txBufferOffset, int ranging, uint8 mode);
static void calcDistance(struct deviceData *device);

void main()
{
    spi_select_slave(deviceA.spiPort, deviceA.spiPin);
    // initiate processor
    peripherals_init();

    // reset DW1000 before init
    reset_DW1000();

    deviceId = dwt_readdevid();
        printf("Device ID is: %X \n", deviceId);
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
    dwt_setrxantennadelay(RX_ANTENNA_DELAY);
    dwt_settxantennadelay(TX_ANTENNA_DELAY);

    // main loop start
    while(1)
    {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        // wait for initiator to start exchange
        // poll DW1000 chip for receive frame event
        //while( !( ( status_reg = dwt_read32bitreg(SYS_STATUS_ID) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR) ) ) )
        init_timer();
        start_timer();
        while( !( ( deviceA.sysStatus = dwt_read32bitreg(SYS_STATUS_ID) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR) ) ) )
        {
            status_reg = check_timer();
            if(status_reg == 0)
            {
                break;
            }
        }

        if (deviceA.sysStatus & SYS_STATUS_RXFCG)
        {
            /* Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= FRAME_LEN_MAX)
            {
                dwt_readrxdata(rxBuffer, frame_len, 0);
                // may want to clear rx_buffer between receives
            }
            if(rxBuffer[FNC_CODE_IDX] == REQ_MSG)
            {
                uint32 resp_tx_time;
                int tx_status;
                deviceA.ts_1 = get_rx_timestamp_u64();
                resp_tx_time = (deviceA.ts_1 + (TX_DELAY * UUS_TO_DWT_TIME)) >> 8;
                                dwt_setdelayedtrxtime(resp_tx_time);

               // tx_status = send_Data(sizeof(ackMsg), ackMsg, 0, 0, DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
                send_Data(sizeof(ackMsg), ackMsg, 0, 0, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

                // poll to check to see that the message was sent correctly
                // useful for debugging, not needed otherwise
                while(!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                { };

                // Clear TX frame sent event
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                // wait for next receive
                while( !( ( status_reg = dwt_read32bitreg(SYS_STATUS_ID) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR) ) ) )
                { };

                if (status_reg & SYS_STATUS_RXFCG)
                {
                    double tRound1, tRound2, tReply1, tReply2, tof, distance;
                    int64 tof_dtu;

                    /* Clear good RX frame event in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
                    if (frame_len <= FRAME_LEN_MAX)
                    {
                        dwt_readrxdata(rxBuffer, frame_len, 0);
                    }
                    if(rxBuffer[FNC_CODE_IDX] == FINAL_MSG)
                    {
                        deviceA.ts_2 = get_tx_timestamp_u64();
                        deviceA.ts_3 = get_rx_timestamp_u64();

                        calcDistance(&deviceA);
                        if( (deviceA.lastDist > 0) && (deviceA.lastDist < 1000))
                        {
                            rangeArray[rangeCount] = deviceA.lastDist;
                            rangeCount++;
                            sumDist += deviceA.lastDist;

                            if (rangeCount == MAX_RANGE_COUNT)
                            {
                                sumDist = sumDist/MAX_RANGE_COUNT;

                               /* int i;
                                for(i = 0; i<MAX_RANGE_COUNT; i++)
                                {
                                    printf("A1:\t%3.2f\n", rangeArray[i]);
                                }
*/
                                printf("Average: %3.2f\n", sumDist);
                                /*dwt_readdiagnostics(&rx_diag);
                                printf("maxNoise: %u\t stdNoise: %u\t CIR: %u\n rxPreamCount: %u\t firstPathPower: %u\t fpAmp1-3: %u, %u, %u\n", rx_diag.maxNoise, rx_diag.stdNoise, rx_diag.maxGrowthCIR, rx_diag.rxPreamCount, rx_diag.firstPath, rx_diag.firstPathAmp1, rx_diag.firstPathAmp2, rx_diag.firstPathAmp3);
                                double fp_power = 10* log10( (pow(rx_diag.firstPathAmp1,2) + pow(rx_diag.firstPathAmp2,2) + pow(rx_diag.firstPathAmp3,2))/ pow(rx_diag.rxPreamCount,2)) - 121.74;
                                double rx_level = 10*log10(rx_diag.maxGrowthCIR * pow(2,17)/pow(rx_diag.rxPreamCount,2)) - 121.74;
                                printf("FP Power: %3.2f dBm\n RX Level: %3.2f dBm\n", fp_power, rx_level);
                                uint16 t_vb = dwt_readtempvbat(0);
                                float vbat = (0x00FF & t_vb) * 0.0057 + 2.3;
                                float temp = (t_vb>>8) * 1.13 - 113.0;
                                printf("VBat: %3.2f\t Temp: %3.2f\n",vbat, temp);
                                */
                                while(1)
                                { };
                            }
                        }

                        /*
                        data_msg_get_ts(&rxBuffer[TX1A_TS_IDX], &tx1a);
                        data_msg_get_ts(&rxBuffer[RX2A_TS_IDX], &rx2a);
                        data_msg_get_ts(&rxBuffer[TX3A_TS_IDX], &tx3a);

                        tRound1 = (double)(rx2a - tx1a);
                        tRound2 = (double)(rx3b - tx2b);

                        tReply1 = (double)(tx2b - rx1b);
                        tReply2 = (double)(tx3a - rx2a);

                        tof_dtu = (int64)( ( tRound1 * tRound2 - tReply1 * tReply2 )/(tRound1 + tRound2 + tReply1 + tReply2) );
                        tof = tof_dtu * DWT_TIME_UNITS;
                        distance = tof * SPEED_OF_LIGHT;
                        printf("Distance: %3.2f\n",distance);*/
                    } else {
                       // puts("Wrong Message received instead of Final");
                    }
                } else {
                    /* Clear RX error/timeout events in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

                    /* Reset RX to properly reinitialise LDE operation. */
                    dwt_rxreset();
                }
            } else {
               // puts("Wrong Message received instead of REQ");
            }

        } else if(deviceA.sysStatus & SYS_STATUS_ALL_RX_ERR){
            /* Clear RX error/timeout events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

            /* Reset RX to properly reinitialise LDE operation. */
            dwt_rxreset();
        }


        // send EOE message
        send_Data(sizeof(eoeMsg), eoeMsg, 0, 0, DWT_START_TX_IMMEDIATE);

        // poll to check to see that the message was sent correctly
        while(!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
        { };

        // Clear TX frame sent event
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    }// main loop end
} // main end


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
 * @fn calcDistance()
 *
 * @brief Calculate distance from tag to device
 *
 * @param  deviceData data struct of device
 *
 * @return  none
 */
static void calcDistance(struct deviceData *device)
{
    double tRound1, tRound2, tReply1, tReply2, tof, distance;
    int64 tof_dtu;

    data_msg_get_ts(&rxBuffer[RX2A_TS_IDX], &rx2a);
    data_msg_get_ts(&rxBuffer[TX1A_TS_IDX], &tx1a);
    data_msg_get_ts(&rxBuffer[TX3A_TS_IDX], &tx3a);

    tRound1 = (double)(rx2a - tx1a);
    tRound2 = (double)(device->ts_3 - device->ts_2);

    tReply1 = (double)(device->ts_2 - device->ts_1);
    tReply2 = (double)(tx3a - rx2a);

    tof_dtu = (int64)( ( tRound1 * tRound2 - tReply1 * tReply2 )/(tRound1 + tRound2 + tReply1 + tReply2) );
    //tof_dtu = (tRound1 - tReply2 + tRound2 - tReply1)/4;
    tof = tof_dtu * DWT_TIME_UNITS;
    distance = tof * SPEED_OF_LIGHT;

    device->lastDist = distance;

    if(1)//( (distance > 0) && (distance < 1000))
    {
        //double delta_reply = tReply1 - tReply2 - RX_ANTENNA_DELAY;
        //printf("delta Reply: %3.2f\t", delta_reply);
        printf("Distance %X: %3.2f\n", device->devAddressLow, distance);


      //  dwt_readdiagnostics(&rx_diag);
       // double rx_level = 10*log10(rx_diag.maxGrowthCIR * pow(2,17)/pow(rx_diag.rxPreamCount,2)) - 121.74;
        //printf("RX_Level: %3.2f", rx_level);
        //double bias_correction = 0.0014*pow(rx_level,3) + 0.32*pow(rx_level,2) + 23*rx_level + 540;
        /*double bias_correction = -0.7*rx_level - 55;
        double corr_dist = distance - (bias_correction/100.0);
        printf("Corrected Distance: %3.2f\n", corr_dist);
        */
    }
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
