/*

    Responder for DS-TWR Using multiple anchors

    Created By Max Nilsson, Based on DecaWave Examples
	
	For use with DW1000Anchors.c

*/

#include <stdio.h>
#include <string.h>

#include <deca_device_api.h>
#include <deca_regs.h>
#include <sleep.h>
#include <msp432_setup.h>


typedef unsigned long long uint64;
typedef signed long long int64;

typedef struct {
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
    0,
    0,
    0xA1,
    0xA2,
    GPIO_PORT_P2,
    GPIO_PIN3,
    GPIO_PORT_P3,
    GPIO_PIN0,
    0.0,
    0.0,
    0,
    0,
    0,
    0
};


// Antenna delays; needs to be properly calibrated, 16436 is default value
#define TX_ANTENNA_DELAY 16436
#define RX_ANTENNA_DELAY 16436

#define FINAL_TX_DELAY 10000

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

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
static uint8 reqMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xE0, 0, 0};

// final message containing timestamps recorded on intiator (tx1a, rx2a, tx3a)
//(not used by responder)
static uint8 finalMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xE2, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0};



// Hold copy of status register here, useful for debugging
static uint32 status_reg = 0;

// Buffer to hold incoming data
#define FRAME_LEN_MAX 127
static uint8 rxBuffer[FRAME_LEN_MAX];

static uint16 frame_len = 0;
uint32_t deviceId;
static bool eoe = 0;

// Timestamp containers

static uint64 txReq;
static uint64 rxA;
static uint64 rxB;
static uint64 rxC;
static uint64 txFinal;
static uint64 lastRxTs;

// Speed of light in air, in metres per second.
#define SPEED_OF_LIGHT 299702547

// Declaration of static functions.
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void data_msg_set_ts(uint8 *ts_field, uint64 ts);
static int send_Data(uint16 txFrameLength, uint8 *txFrameBytes, uint16 txBufferOffset, int ranging, uint8 mode);

void main()
{
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

    reqMsg[SRC_LOW] = deviceA.devAddressLow;
    reqMsg[SRC_HIGH] = deviceA.devAddressHigh;

    // main loop start
    while(1)
    {
        uint8 rec = 0;

        send_Data(sizeof(reqMsg), reqMsg, 0, 0, DWT_START_TX_IMMEDIATE);

        // poll to check to see that the message was sent correctly
        while(!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
        { };

        // Clear TX frame sent event
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
        txReq = get_tx_timestamp_u64();

        while(rec < 7)
        {
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            while( !(deviceA.sysStatus = dwt_read32bitreg(SYS_STATUS_ID) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)) )
            { };

            if(deviceA.sysStatus & SYS_STATUS_RXFCG)
            {
                // clear good rx frame in dw1000
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

                frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
                if (frame_len <= FRAME_LEN_MAX)
                {
                    dwt_readrxdata(rxBuffer, 10, 0);
                }
                if(rxBuffer[FNC_CODE_IDX] == ACK_MSG)
                {
                    switch(rxBuffer[SRC_LOW]){
                        case 0xA1:
                            rxA = get_rx_timestamp_u64();
                            rec+=1;
                        break;

                        case 0xB1:
                            rxB = get_rx_timestamp_u64();
                            rec+=2;
                        break;

                        case 0xC1:
                            rxC = get_rx_timestamp_u64();
                            rec+=4;
                        break;
                    }
                }
                else if (rxBuffer[FNC_CODE_IDX] == EOE_MSG)
                {
                    eoe = 1;
                    puts("EoE\n");
                    break;
                }
            } else {
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

                dwt_rxreset();
            }
        }
        if(!eoe)
        {
          //  puts("Starting final message");
            uint32 final_tx_time;
            // Compute final message transmission time. We want to send tx3a with last message
            final_tx_time = (rxC + (FINAL_TX_DELAY * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(final_tx_time);

            // Final TX timestamp is the transmission time we programmed plus the TX antenna delay.
            txFinal = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANTENNA_DELAY;

            data_msg_set_ts(&finalMsg[REQ_TX_TS_IDX], txReq);
            data_msg_set_ts(&finalMsg[RXA_TS_IDX], rxA);
            data_msg_set_ts(&finalMsg[RXB_TS_IDX], rxB);
            data_msg_set_ts(&finalMsg[RXC_TS_IDX], rxC);
            data_msg_set_ts(&finalMsg[FINAL_TX_TS_IDX], txFinal);

            finalMsg[SRC_LOW] = deviceA.devAddressLow;
            finalMsg[SRC_HIGH] = deviceA.devAddressHigh;

            // Delayed TX can fail if FINAL_TX_DELAY is too small
            int tx_status = send_Data(sizeof(finalMsg), finalMsg, 0, 0, DWT_START_TX_DELAYED);

            if(tx_status == DWT_ERROR)
            {
                puts("Delayed send error, check status_reg for sys_status register\n");
                status_reg = dwt_read32bitoffsetreg(SYS_STATUS_ID,1);
                while(1)
                {};

            } else {
                // wait for EoE before starting next exchange
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                // Poll DW1000 until TX frame sent event set.
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                { };

                // Clear TXFRS event.
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                while( !( ( status_reg = dwt_read32bitreg(SYS_STATUS_ID) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR) ) ) )
                {  };

               /* Clear good RX frame event in the DW1000 status register. */
               dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            }
        }/* else {
            // wait for EoE before starting next exchange
            dwt_rxenable(DWT_START_RX_IMMEDIATE);

            while( !( ( status_reg = dwt_read32bitreg(SYS_STATUS_ID) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR) ) ) )
            {  };
*/
           // Clear good RX frame event in the DW1000 status register.
           dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
           eoe = 0;

        //}
        //sleep_ms(15);
    }
}

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

static void data_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < 5; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}
