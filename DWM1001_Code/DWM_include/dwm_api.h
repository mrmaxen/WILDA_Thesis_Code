/*! ------------------------------------------------------------------------------------------------------------------
 * @file    dwm_api.h
 * @brief   DWM1001 host API header 
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#ifndef _DWM_API_H_
#define _DWM_API_H_

#include "dwm1001_tlv.h"

/**
 * @brief DWM Error codes, returned from DWM1001 module
 */
#define DWM_OK				   (0)
#define DWM_ERR_INTERNAL	(-1)
#define DWM_ERR_BUSY		   (-2)
#define DWM_ERR_INVAL_ADDR	(-3)
#define DWM_ERR_INVAL_PARAM   (-4)
#define DWM_ERR_OVERRUN		(-5)
#define DWM_ERR_I2C_ANACK	(-10)
#define DWM_ERR_I2C_DNACK	(-11)

/**
 * @brief Return value Error codes, returned from DWM API functions
 */
#define RV_OK           (0)      /*  ret value OK */
#define RV_ERR          (1)      /*  ret value ERROR*/

/********************************************************************************************************************/
/*                                                     API LIST                                                     */
/********************************************************************************************************************/

/**
 * @brief Initializes the required components for DWM1001 module, 
 *       especially the Low-level Module Handshake (LMH) devides. 
 *
 * @param[in] none
 *
 * @return none
 */
void dwm_init(void);

/**
 * @brief Position coordinates in millimeters + quality factor
 */
typedef struct {
	int32_t x;
	int32_t y;
	int32_t z;
	uint8_t qf;
} dwm_pos_t;
/**
 * @brief Sets position of anchor node
 *
 * @param[in] pos, pointer to position coordinates
 *
 * @return Error code
 */
int dwm_pos_set(dwm_pos_t* pos);

/**
 * @brief Gets position of the node
 *
 * @param[out] pos, pointer to position
 *
 * @return Error code
 */
int dwm_pos_get(dwm_pos_t* pos);

/* maximum and minimum update rate in multiple of 100 ms */
enum dwm_upd_rate{
	DWM_UPD_RATE_MAX = 600,	/* 1 minute */
	DWM_UPD_RATE_MIN = 1	/* 100 ms */
};
/**
 * @brief Sets update rate
 *
 * @param[in] ur, Update rate in multiply of 100 ms, [min,max] = [DWM_UPD_RATE_MIN, DWM_UPD_RATE_MAX]
 * @param[in] urs, Stationary update rate in multiply of 100 ms, [min,max] = [DWM_UPD_RATE_MIN, DWM_UPD_RATE_MAX]
 *
 * @return Error code
 */
int dwm_upd_rate_set(uint16_t ur, uint16_t urs);

/**
 * @brief Gets update rate
 *
 * @param[out] ur, Pointer to update rate, update rate is multiply of 100 ms
 * [min,max] = [DWM_UPD_RATE_MIN, DWM_UPD_RATE_MAX]
 * @param[out] urs, Pointer to stationary update rate, update rate is multiply of 100 ms
 * [min,max] = [DWM_UPD_RATE_MIN, DWM_UPD_RATE_MAX]
 *
 * @return Error code
 */
int dwm_upd_rate_get(uint16_t *ur, uint16_t *urs);

/**
 * @brief Position measurement modes
 */
typedef enum {
	DWM_MEAS_MODE_TWR = 0,//!< DWM_MEAS_MODE_TWR
	DWM_MEAS_MODE_TDOA = 1//!< DWM_MEAS_MODE_TDOA
} dwm_meas_mode_t;

/**
 * @brief Device modes
 */
typedef enum {
	DWM_MODE_TAG = 0,  //!< DWM_MODE_TAG
	DWM_MODE_ANCHOR = 1//!< DWM_MODE_ANCHOR
} dwm_mode_t;

typedef enum {
	DWM_UWB_MODE_OFF = 0,
	DWM_UWB_MODE_PASSIVE = 1,
	DWM_UWB_MODE_ACTIVE = 2
}dwm_uwb_mode_t;

typedef struct dwm_cfg_common {
	bool fw_update_en;
	dwm_uwb_mode_t uwb_mode;
	bool security_en;
	bool ble_en;
	bool led_en;
} dwm_cfg_common_t;

typedef struct dwm_cfg_anchor {
	dwm_cfg_common_t common;
	bool bridge;
	bool initiator;
} dwm_cfg_anchor_t;

typedef struct dwm_cfg_tag {
	dwm_cfg_common_t common;
	bool loc_engine_en;
	bool low_power_en;
	bool accel_en;
	dwm_meas_mode_t meas_mode;
} dwm_cfg_tag_t;

typedef struct dwm_cfg {
	dwm_cfg_common_t common;
	bool loc_engine_en;
	bool low_power_en;
	bool accel_en;
	dwm_meas_mode_t meas_mode;
	bool bridge;
	bool initiator;
	dwm_mode_t mode;
} dwm_cfg_t;

/**
 * @brief Configures node to tag mode with given options
 *
 * @param[in] cfg, Tag configuration options
 *
 * @return Error code
 */
int dwm_cfg_tag_set(dwm_cfg_tag_t* cfg);

/**
 * @brief Configures node to anchor mode with given options
 *
 * @param[in] cfg, Anchor configuration options
 *
 * @return Error code
 */
int dwm_cfg_anchor_set(dwm_cfg_anchor_t* cfg);

/**
 * @brief Reads configuration of the node
 *
 * @param[out] cfg, Node configuration
 *
 * @return Error code
 */
int dwm_cfg_get(dwm_cfg_t* cfg);

/**
 * @brief Puts device to sleep mode
 *
 * @return Error code
 */
int dwm_sleep(void);


/* Maximum count of nodes in location data */
#define DWM_LOC_CNT_MAX	15

/**
 * @brief Distances of ranging anchors
 */
typedef struct {
	uint8_t cnt;
	uint64_t addr[DWM_LOC_CNT_MAX];
	uint32_t dist[DWM_LOC_CNT_MAX];
	uint8_t qf[DWM_LOC_CNT_MAX];
}dwm_distance_t;

/**
 * @brief Position of ranging anchors
 */
typedef struct {
	uint8_t cnt;
	dwm_pos_t pos[DWM_LOC_CNT_MAX];
}dwm_anchor_pos_t;

/**
 * @brief Distances and position of ranging anchors
 */
typedef struct {
	dwm_distance_t dist;
	dwm_anchor_pos_t an_pos;
}dwm_ranging_anchors_t;

/**
 * @brief Location data (position of current node and list of positions
 * and distances of ranging anchors)
 */
typedef struct dwm_loc_data_t {
	dwm_pos_t* p_pos;
	dwm_ranging_anchors_t anchors;
} dwm_loc_data_t;
/**
 * @brief Gets location data
 *
 * @param[out] loc Pointer to location data
 *
 * @return Error code
 */
int dwm_loc_get(dwm_loc_data_t* loc);

/**
 * @brief Resets DWM module
 *
 * @return Error code
 */
int dwm_reset(void);

/**
 * @brief Firmware version data
 */
typedef struct dwm_fw_ver_t {
	uint8_t maj;
	uint8_t min;
	uint8_t patch;
	uint8_t res;
	uint8_t var;
}dwm_fw_ver_t;

/**
 * @brief Version data
 */
typedef struct dwm_ver_t {
	dwm_fw_ver_t fw;
	uint32_t cfg;
	uint32_t hw;
} dwm_ver_t;

/**
 * @brief Gets versions
 *
 * @param[out] ver, pointer to version data
 *
 * @return Error code
 */
int dwm_ver_get(dwm_ver_t* ver);

/**
 * @brief DWM status bits mask
 */ 
#define DWM_STATUS_LOC_DATA_READY_MASK    0x01
#define DWM_STATUS_UWBMAC_JOINED_MASK     0x02

/**
 * @brief DWM status
 */
typedef struct dwm_status_t {
	bool loc_data;
	bool uwbmac_joined;
} dwm_status_t;

/**
 * @brief Get system status: Location Data ready
 *
 * @param[out] status
 *
 * @return Error code
 */
int dwm_status_get(dwm_status_t* p_status);
	

#endif //_DWM_API_H_

