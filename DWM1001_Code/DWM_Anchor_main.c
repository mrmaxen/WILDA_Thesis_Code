/*
        Simple Code for ranging using DWM1001 with a MSP432P401R

        Based on examples provided by DecaWave
		
		Code used to program DWM1001 for use as anchors.
		
		Data is gathered by DWM_Tag_main.c
		
		Author: Max Nilsson
*/

#define DEVICE_C // define which device is being programmed

#include "dwm_api.h"
#include "driverlib.h"
#include "msp432_lib.h"
#include <stdio.h>

void main()
{
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);    // Set clock frequency to 12 MHz

    dwm_init();

    dwm_cfg_anchor_t cfg_anchor;
    //dwm_cfg_t cfg_node;
    dwm_pos_t pos;
    dwm_pos_t g_pos;
    pos.qf = 100;
#ifdef DEVICE_A
    // Position in mm
    pos.x = 150;
    pos.y = 0;
    cfg_anchor.initiator = 1; // Initiator role enabled
#elif defined DEVICE_B
    pos.x = -150;
    pos.y = 0;
    cfg_anchor.initiator = 0;
#elif defined DEVICE_C
    pos.x = 0;
    pos.y = 150;
    cfg_anchor.initiator = 0;
#else
    printf("No Device Defined\n");
#endif

    pos.z = 0;

    cfg_anchor.bridge = 0;                          // Bridge role disabled
    cfg_anchor.common.led_en = 1;                   // Leading edge detection I think? Datasheet is unclear
    cfg_anchor.common.ble_en = 1;                   // Bluetooth enable, required; see Note 1
    cfg_anchor.common.uwb_mode = DWM_UWB_MODE_ACTIVE;  // UWB mode
    cfg_anchor.common.fw_update_en = 1;             // Firmware update enable, antennas with different FW version will not work

    dwm_cfg_anchor_set(&cfg_anchor);				// write config values to DWM

    dwm_pos_set(&pos);								// write x, y, z position values to DWM. Only necessary if new values are being programmed

    dwm_reset();                                    // reset to apply config values

    sleep_ms(2000); // wait for reset

    //dwm_pos_get(&g_pos); // can be used to check that programmed values were in fact set.
	//printf("%X\n", g_pos.y);
    while(1); // loop indefinitely, no further action necessary
}

/**********************************************
Note 1: With current firmware, a network can only be created using Bluetooth, I used the android app which is very easy.
			decaWave says that in Firmware version 2 network creation should be possible using UART.


***********************************************/
