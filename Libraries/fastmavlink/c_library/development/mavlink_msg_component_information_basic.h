//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_H
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_H


//----------------------------------------
//-- Message COMPONENT_INFORMATION_BASIC
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_component_information_basic_t {
    uint64_t capabilities;
    uint32_t time_boot_ms;
    uint8_t vendor_name[32];
    uint8_t model_name[32];
    char software_version[24];
    char hardware_version[24];
}) fmav_component_information_basic_t;


#define FASTMAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC  396

#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_PAYLOAD_LEN_MAX  124
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_CRCEXTRA  122

#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FLAGS  0
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FRAME_LEN_MAX  149

#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_VENDOR_NAME_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_VENDOR_NAME_LEN  32 // length of array = number of bytes
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_MODEL_NAME_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_MODEL_NAME_LEN  32 // length of array = number of bytes
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_SOFTWARE_VERSION_NUM  24 // number of elements in array
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_SOFTWARE_VERSION_LEN  24 // length of array = number of bytes
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_HARDWARE_VERSION_NUM  24 // number of elements in array
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_HARDWARE_VERSION_LEN  24 // length of array = number of bytes

#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_CAPABILITIES_OFS  0
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_TIME_BOOT_MS_OFS  8
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_VENDOR_NAME_OFS  12
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_MODEL_NAME_OFS  44
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_SOFTWARE_VERSION_OFS  76
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_HARDWARE_VERSION_OFS  100


//----------------------------------------
//-- Message COMPONENT_INFORMATION_BASIC pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_information_basic_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, const uint8_t* vendor_name, const uint8_t* model_name, const char* software_version, const char* hardware_version, uint64_t capabilities,
    fmav_status_t* _status)
{
    fmav_component_information_basic_t* _payload = (fmav_component_information_basic_t*)_msg->payload;

    _payload->capabilities = capabilities;
    _payload->time_boot_ms = time_boot_ms;
    memcpy(&(_payload->vendor_name), vendor_name, sizeof(uint8_t)*32);
    memcpy(&(_payload->model_name), model_name, sizeof(uint8_t)*32);
    memcpy(&(_payload->software_version), software_version, sizeof(char)*24);
    memcpy(&(_payload->hardware_version), hardware_version, sizeof(char)*24);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_information_basic_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_component_information_basic_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_component_information_basic_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->vendor_name, _payload->model_name, _payload->software_version, _payload->hardware_version, _payload->capabilities,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_information_basic_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, const uint8_t* vendor_name, const uint8_t* model_name, const char* software_version, const char* hardware_version, uint64_t capabilities,
    fmav_status_t* _status)
{
    fmav_component_information_basic_t* _payload = (fmav_component_information_basic_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->capabilities = capabilities;
    _payload->time_boot_ms = time_boot_ms;
    memcpy(&(_payload->vendor_name), vendor_name, sizeof(uint8_t)*32);
    memcpy(&(_payload->model_name), model_name, sizeof(uint8_t)*32);
    memcpy(&(_payload->software_version), software_version, sizeof(char)*24);
    memcpy(&(_payload->hardware_version), hardware_version, sizeof(char)*24);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_information_basic_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_component_information_basic_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_component_information_basic_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->vendor_name, _payload->model_name, _payload->software_version, _payload->hardware_version, _payload->capabilities,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_information_basic_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, const uint8_t* vendor_name, const uint8_t* model_name, const char* software_version, const char* hardware_version, uint64_t capabilities,
    fmav_status_t* _status)
{
    fmav_component_information_basic_t _payload;

    _payload.capabilities = capabilities;
    _payload.time_boot_ms = time_boot_ms;
    memcpy(&(_payload.vendor_name), vendor_name, sizeof(uint8_t)*32);
    memcpy(&(_payload.model_name), model_name, sizeof(uint8_t)*32);
    memcpy(&(_payload.software_version), software_version, sizeof(char)*24);
    memcpy(&(_payload.hardware_version), hardware_version, sizeof(char)*24);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC,
        FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_information_basic_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_component_information_basic_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC,
        FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message COMPONENT_INFORMATION_BASIC decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zerofill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_component_information_basic_decode(fmav_component_information_basic_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    memcpy(payload, msg->payload, msg->len);
    // ensure that returned payload is zero filled
    if (msg->len < FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_PAYLOAD_LEN_MAX - msg->len);
    }
#else
    // this requires that msg payload had been zero filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_component_information_basic_get_field_capabilities(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_component_information_basic_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_component_information_basic_get_field_vendor_name_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[12]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_component_information_basic_get_field_vendor_name(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_VENDOR_NAME_NUM) return 0;
    return ((uint8_t*)&(msg->payload[12]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_component_information_basic_get_field_model_name_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[44]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_component_information_basic_get_field_model_name(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_MODEL_NAME_NUM) return 0;
    return ((uint8_t*)&(msg->payload[44]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_component_information_basic_get_field_software_version_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[76]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_component_information_basic_get_field_software_version(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_SOFTWARE_VERSION_NUM) return 0;
    return ((char*)&(msg->payload[76]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_component_information_basic_get_field_hardware_version_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[100]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_component_information_basic_get_field_hardware_version(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_HARDWARE_VERSION_NUM) return 0;
    return ((char*)&(msg->payload[100]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC  396

#define mavlink_component_information_basic_t  fmav_component_information_basic_t

#define MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_LEN  124
#define MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_MIN_LEN  124
#define MAVLINK_MSG_ID_396_LEN  124
#define MAVLINK_MSG_ID_396_MIN_LEN  124

#define MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC_CRC  122
#define MAVLINK_MSG_ID_396_CRC  122

#define MAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_VENDOR_NAME_LEN 32
#define MAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_MODEL_NAME_LEN 32
#define MAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_SOFTWARE_VERSION_LEN 24
#define MAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_HARDWARE_VERSION_LEN 24


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_component_information_basic_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, const uint8_t* vendor_name, const uint8_t* model_name, const char* software_version, const char* hardware_version, uint64_t capabilities)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_component_information_basic_pack(
        _msg, sysid, compid,
        time_boot_ms, vendor_name, model_name, software_version, hardware_version, capabilities,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_component_information_basic_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, const uint8_t* vendor_name, const uint8_t* model_name, const char* software_version, const char* hardware_version, uint64_t capabilities)
{
    return fmav_msg_component_information_basic_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, vendor_name, model_name, software_version, hardware_version, capabilities,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_component_information_basic_decode(const mavlink_message_t* msg, mavlink_component_information_basic_t* payload)
{
    fmav_msg_component_information_basic_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_COMPONENT_INFORMATION_BASIC_H
