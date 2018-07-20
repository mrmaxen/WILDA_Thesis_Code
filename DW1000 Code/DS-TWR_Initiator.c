/*

    Initiator for DS-TWR

    Created By Max Nilsson, Based on DecaWave Examples
	
	For use with DS-TWR_Responder.c

*/

#include <stdio.h>
#include <string.h>

#include <deca_device_api.h>
#include <deca_regs.h>
#include <sleep.h>
#include <msp432_setup.h>

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

// Antenna delays; needs to be properly calibrated, 16436 is default value
#define TX_ANTENNA_DELAY 0
#define RX_ANTENNA_DELAY 0

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

#define FINAL_TX_DELAY 10000

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
static uint8 reqMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};

// final message containing timestamps recorded on intiator (tx1a, rx2a, tx3a)
static uint8 finalMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE2, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0};



// Hold copy of status register here, useful for debugging
static uint32 status_reg = 0;

// Buffer to hold incoming data
#define FRAME_LEN_MAX 127
static uint8 rxBuffer[FRAME_LEN_MAX];

static uint16 frame_len = 0;
uint32_t deviceId;

typedef unsigned long long uint64;
// Timestamp containers
static uint64 rx2a;
static uint64 tx3a;
static uint64 tx1a;

// Declaration of static functions.
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void set_data_timestamp(uint8 *ts_field, uint64 ts);
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

    // main loop start
    while(1)
    {
        // initiate exchange
        send_Data(sizeof(reqMsg), reqMsg, 0, 0, DWT_START_TX_IMMEDIATE);

        // poll to check to see that the message was sent correctly

        while(!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
        { };
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        // Clear TX frame sent event
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

        // poll DW1000 chip for receive frame event
        while( !( ( status_reg = dwt_read32bitreg(SYS_STATUS_ID) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR ) ) ) )
        {  };

        if (status_reg & SYS_STATUS_RXFCG)
        {
            uint32 final_tx_time;
            int tx_status;
            /* Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= FRAME_LEN_MAX)
            {
                dwt_readrxdata(rxBuffer, frame_len, 0);
                // may want to clear rx_buffer between receives
            }
            if(rxBuffer[FNC_CODE_IDX] == ACK_MSG)
            {

                // get first tx timestamp
                tx1a = get_tx_timestamp_u64();
                // save RX timestamp
                rx2a = get_rx_timestamp_u64();

                // Compute final message transmission time. We want to send tx3a with last message
                final_tx_time = (rx2a + (FINAL_TX_DELAY * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(final_tx_time);

                // Final TX timestamp is the transmission time we programmed plus the TX antenna delay.
                tx3a = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANTENNA_DELAY;

                set_data_timestamp(&finalMsg[TX1A_TS_IDX], tx1a);
                set_data_timestamp(&finalMsg[RX2A_TS_IDX], rx2a);
                set_data_timestamp(&finalMsg[TX3A_TS_IDX], tx3a);

                tx_status = send_Data(sizeof(finalMsg), finalMsg, 0, 0, DWT_START_TX_DELAYED);

                if(tx_status == DWT_ERROR)
                {
                    puts("Delayed send error, check status_reg for sys_status register\n");
                    status_reg = dwt_read32bitreg(SYS_STATUS_ID);
                    while(1)
                    {};

                } else {
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    // Poll DW1000 until TX frame sent event set. See NOTE 9 below.
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                    { };

                    // Clear TXFRS event.
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                    while( !( ( status_reg = dwt_read32bitreg(SYS_STATUS_ID) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR) ) ) )
                   {  };

                   /* Clear good RX frame event in the DW1000 status register. */
                   dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
                }
            }
        } else {
            /* Clear RX error/timeout events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

            /* Reset RX to properly reinitialise LDE operation. */
            dwt_rxreset();
        }
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

// TODO add device parameter to timestamp getters and setters
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

static void set_data_timestamp(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < 5; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}
