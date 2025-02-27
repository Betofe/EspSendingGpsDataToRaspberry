//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_UAVIONIX_H
#define FASTMAVLINK_UAVIONIX_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FASTMAVLINK_BUILD_DATE
#define FASTMAVLINK_BUILD_DATE  "Wed Jun 01 2022"
#endif

#ifndef FASTMAVLINK_DIALECT_VERSION
#define FASTMAVLINK_DIALECT_VERSION  0  // this is the version specified in the dialect xml file
#endif


//------------------------------
//-- Message credentials
//-- The values of msg_entry_t for all messages in the dialect.
//-- msgid, extra crc, max length, flag, target sysid offset, target compid offset
//------------------------------

#include "uAvionix_msg_entries.h"

#ifndef FASTMAVLINK_MESSAGE_CRCS
#define FASTMAVLINK_MESSAGE_CRCS  FASTMAVLINK_MSG_ENTRIES
#endif


//------------------------------
//-- FastMavlink lib
//------------------------------

#include "../lib/fastmavlink.h"

#ifdef FASTMAVLINK_PYMAVLINK_ENABLED
#include "../lib/fastmavlink_pymavlink.h"
#endif


//------------------------------
//-- Enum definitons
//------------------------------

#ifndef FASTMAVLINK_TEST_EXCLUDE_ENUMS

#ifndef FASTMAVLINK_HAS_ENUM_UAVIONIX_ADSB_OUT_DYNAMIC_STATE
#define FASTMAVLINK_HAS_ENUM_UAVIONIX_ADSB_OUT_DYNAMIC_STATE
typedef enum UAVIONIX_ADSB_OUT_DYNAMIC_STATE {
    UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE = 1,  //  
    UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED = 2,  //  
    UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED = 4,  //  
    UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND = 8,  //  
    UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT = 16,  //  
    UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ENUM_END = 17,  // end marker
} UAVIONIX_ADSB_OUT_DYNAMIC_STATE;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_UAVIONIX_ADSB_OUT_RF_SELECT
#define FASTMAVLINK_HAS_ENUM_UAVIONIX_ADSB_OUT_RF_SELECT
typedef enum UAVIONIX_ADSB_OUT_RF_SELECT {
    UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY = 0,  //  
    UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED = 1,  //  
    UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED = 2,  //  
    UAVIONIX_ADSB_OUT_RF_SELECT_ENUM_END = 3,  // end marker
} UAVIONIX_ADSB_OUT_RF_SELECT;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX
#define FASTMAVLINK_HAS_ENUM_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX
typedef enum UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX {
    UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0 = 0,  //  
    UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_1 = 1,  //  
    UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_2D = 2,  //  
    UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_3D = 3,  //  
    UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS = 4,  //  
    UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_RTK = 5,  //  
    UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_ENUM_END = 6,  // end marker
} UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_UAVIONIX_ADSB_RF_HEALTH
#define FASTMAVLINK_HAS_ENUM_UAVIONIX_ADSB_RF_HEALTH
typedef enum UAVIONIX_ADSB_RF_HEALTH {
    UAVIONIX_ADSB_RF_HEALTH_INITIALIZING = 0,  //  
    UAVIONIX_ADSB_RF_HEALTH_OK = 1,  //  
    UAVIONIX_ADSB_RF_HEALTH_FAIL_TX = 2,  //  
    UAVIONIX_ADSB_RF_HEALTH_FAIL_RX = 16,  //  
    UAVIONIX_ADSB_RF_HEALTH_ENUM_END = 17,  // end marker
} UAVIONIX_ADSB_RF_HEALTH;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE
#define FASTMAVLINK_HAS_ENUM_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE
typedef enum UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE {
    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_NO_DATA = 0,  //  
    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L15M_W23M = 1,  //  
    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25M_W28P5M = 2,  //  
    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25_34M = 3,  //  
    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L35_33M = 4,  //  
    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L35_38M = 5,  //  
    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_39P5M = 6,  //  
    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_45M = 7,  //  
    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_45M = 8,  //  
    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_52M = 9,  //  
    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_59P5M = 10,  //  
    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_67M = 11,  //  
    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W72P5M = 12,  //  
    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W80M = 13,  //  
    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W80M = 14,  //  
    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W90M = 15,  //  
    UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_ENUM_END = 16,  // end marker
} UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT
#define FASTMAVLINK_HAS_ENUM_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT
typedef enum UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT {
    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_NO_DATA = 0,  //  
    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_2M = 1,  //  
    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_4M = 2,  //  
    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_6M = 3,  //  
    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_0M = 4,  //  
    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_2M = 5,  //  
    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_4M = 6,  //  
    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M = 7,  //  
    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_ENUM_END = 8,  // end marker
} UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON
#define FASTMAVLINK_HAS_ENUM_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON
typedef enum UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON {
    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA = 0,  //  
    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR = 1,  //  
    UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_ENUM_END = 2,  // end marker
} UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_UAVIONIX_ADSB_EMERGENCY_STATUS
#define FASTMAVLINK_HAS_ENUM_UAVIONIX_ADSB_EMERGENCY_STATUS
typedef enum UAVIONIX_ADSB_EMERGENCY_STATUS {
    UAVIONIX_ADSB_OUT_NO_EMERGENCY = 0,  //  
    UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY = 1,  //  
    UAVIONIX_ADSB_OUT_LIFEGUARD_EMERGENCY = 2,  //  
    UAVIONIX_ADSB_OUT_MINIMUM_FUEL_EMERGENCY = 3,  //  
    UAVIONIX_ADSB_OUT_NO_COMM_EMERGENCY = 4,  //  
    UAVIONIX_ADSB_OUT_UNLAWFUL_INTERFERANCE_EMERGENCY = 5,  //  
    UAVIONIX_ADSB_OUT_DOWNED_AIRCRAFT_EMERGENCY = 6,  //  
    UAVIONIX_ADSB_OUT_RESERVED = 7,  //  
    UAVIONIX_ADSB_EMERGENCY_STATUS_ENUM_END = 8,  // end marker
} UAVIONIX_ADSB_EMERGENCY_STATUS;
#endif

#endif // FASTMAVLINK_DO_NOT_INCLUDE_ENUMS


//------------------------------
//-- Message definitions
//------------------------------

#ifdef FASTMAVLINK_IGNORE_WADDRESSOFPACKEDMEMBER
  #if defined __GNUC__ && __GNUC__ >= 9
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Waddress-of-packed-member"
  #endif
#endif

#include "./mavlink_msg_uavionix_adsb_out_cfg.h"
#include "./mavlink_msg_uavionix_adsb_out_dynamic.h"
#include "./mavlink_msg_uavionix_adsb_transceiver_health_report.h"

#ifdef FASTMAVLINK_IGNORE_WADDRESSOFPACKEDMEMBER
  #if defined __GNUC__ && __GNUC__ >= 9
    #pragma GCC diagnostic pop
  #endif
#endif


//------------------------------
//-- Dialect includes
//------------------------------

#include "../common/common.h"


#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_UAVIONIX_H
