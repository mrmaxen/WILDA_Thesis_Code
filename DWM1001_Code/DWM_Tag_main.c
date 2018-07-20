/*
        Simple Code for ranging using DWM1001 with a MSP432P401R

        Based on examples provided by DecaWave
		
		Data is extracted by printing to console, copying to textfile and importing to MATLAB.
		Slow and inefficient, but easy and simple.
		
		Author: Max Nilsson
*/

#define CONFIG //config is only needed if new values are being set

#include "dwm_api.h"
#include "driverlib.h"
#include "msp432_lib.h"
#include <stdio.h>

void main()
{

    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);    // Set clock frequency to 12 MHz

    // Declare variables

    dwm_cfg_tag_t cfg_tag; // Tag config struct
    dwm_cfg_t cfg_node;

    dwm_init(); // initialize UART. Calls LMH_Init, which needs to be adjusted for platform
#ifdef CONFIG
    cfg_tag.low_power_en = 0;                       // low power mode enable
    cfg_tag.accel_en = 0;                           // Accelerometer enable
    cfg_tag.meas_mode = DWM_MEAS_MODE_TWR;          // measurement mode; 0=TWR; 1,2,3 reserved
    cfg_tag.loc_engine_en = 1;                      // internal location engine enable, uses decaWave algorithm
    cfg_tag.common.led_en = 1;                      // Leading edge Detection? nothing in datasheet
    cfg_tag.common.ble_en = 1;                      // Bluetooth enable, see note 1
    cfg_tag.common.uwb_mode = DWM_UWB_MODE_ACTIVE;  // UWB mode; 0=offline, 1=passive; 2=active, if inactive will not communicate with anchors
    cfg_tag.common.fw_update_en = 1;                // firmware update enable

    dwm_cfg_tag_set(&cfg_tag);                      // set config values

    dwm_reset();									// reset to apply config values
    
    sleep_ms(2000);    // wait for reset

#endif
    // For debugging, can check set vs get here, see example
#define MAX_COUNT 2000 // how many measurements we want to take
    static uint16_t end_counter = 0, i;
    dwm_status_t status;
    dwm_loc_data_t loc;
    dwm_pos_t pos;
    loc.p_pos = &pos; // x, y, z and qf (quality factor)

    while(1)
    {
        if( (dwm_status_get(&status) == RV_OK) && (status.loc_data)) // get loc and check for error
        {
            dwm_loc_get(&loc);
           // printf("x: %d\t y: %d\t z: %d\t qf: %u\n", pos.x, pos.y, pos.z, loc.p_pos->qf);
            //int32_t distance = loc.anchors.dist.dist[0];
			// printed in this format to simplify moving data to matlab
            printf("A:\t%i\tB:\t%i\tC:\t%i\n", loc.anchors.dist.dist[0],loc.anchors.dist.dist[1],loc.anchors.dist.dist[2]);

            end_counter++;
           if (end_counter == MAX_COUNT)
           {
               printf("Done!\n");
               while(1);
           }
        }
    }
}

/**********************************************
Note 1: With current firmware, a network can only be created using Bluetooth, I used the android app which is very easy.
			decaWave says that in Firmware version 2 network creation should be possible using UART.


***********************************************/
