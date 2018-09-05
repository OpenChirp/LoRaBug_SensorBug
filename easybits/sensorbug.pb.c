/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.4.0-dev at Tue Sep  4 20:01:14 2018. */

#include "sensorbug.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t SensorBugUplinkMsg_fields[15] = {
    PB_FIELD(  1, UINT32  , OPTIONAL, STATIC  , FIRST, SensorBugUplinkMsg, counter, counter, 0),
    PB_FIELD(  2, UINT32  , OPTIONAL, STATIC  , OTHER, SensorBugUplinkMsg, battery, counter, 0),
    PB_FIELD(  3, UINT32  , OPTIONAL, STATIC  , OTHER, SensorBugUplinkMsg, light, battery, 0),
    PB_FIELD(  4, UINT32  , OPTIONAL, STATIC  , OTHER, SensorBugUplinkMsg, pir_count, light, 0),
    PB_FIELD(  5, UINT32  , OPTIONAL, STATIC  , OTHER, SensorBugUplinkMsg, motion_count, pir_count, 0),
    PB_FIELD(  6, FLOAT   , OPTIONAL, STATIC  , OTHER, SensorBugUplinkMsg, temperature, motion_count, 0),
    PB_FIELD(  7, FLOAT   , OPTIONAL, STATIC  , OTHER, SensorBugUplinkMsg, humidity, temperature, 0),
    PB_FIELD(  8, FLOAT   , OPTIONAL, STATIC  , OTHER, SensorBugUplinkMsg, pressure, humidity, 0),
    PB_FIELD(  9, FLOAT   , OPTIONAL, STATIC  , OTHER, SensorBugUplinkMsg, gas_resistance, pressure, 0),
    PB_FIELD( 10, UINT32  , OPTIONAL, STATIC  , OTHER, SensorBugUplinkMsg, ambient_noise, gas_resistance, 0),
    PB_FIELD( 11, UINT32  , OPTIONAL, STATIC  , OTHER, SensorBugUplinkMsg, period, ambient_noise, 0),
    PB_FIELD( 12, BOOL    , OPTIONAL, STATIC  , OTHER, SensorBugUplinkMsg, motion_en, period, 0),
    PB_FIELD( 13, BOOL    , OPTIONAL, STATIC  , OTHER, SensorBugUplinkMsg, light_en, motion_en, 0),
    PB_FIELD( 14, BOOL    , OPTIONAL, STATIC  , OTHER, SensorBugUplinkMsg, mic_en, light_en, 0),
    PB_LAST_FIELD
};

const pb_field_t SensorBugDownlinkMsg_fields[5] = {
    PB_FIELD(  1, UINT32  , OPTIONAL, STATIC  , FIRST, SensorBugDownlinkMsg, period, period, 0),
    PB_FIELD(  2, BOOL    , OPTIONAL, STATIC  , OTHER, SensorBugDownlinkMsg, motion_en, period, 0),
    PB_FIELD(  3, BOOL    , OPTIONAL, STATIC  , OTHER, SensorBugDownlinkMsg, light_en, motion_en, 0),
    PB_FIELD(  4, BOOL    , OPTIONAL, STATIC  , OTHER, SensorBugDownlinkMsg, mic_en, light_en, 0),
    PB_LAST_FIELD
};


/* @@protoc_insertion_point(eof) */
