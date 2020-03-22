#pragma once
// MESSAGE AvoidControlFlow PACKING

#define MAVLINK_MSG_ID_AvoidControlFlow 1234

MAVPACKED(
typedef struct __mavlink_avoidcontrolflow_t {
 int32_t suggest_move_x; /*< [pixel] suggest_move_x.*/
 int32_t suggest_move_y; /*< [pixel] suggest_move_y.*/
 int32_t suggest_move_z; /*< [pixel] suggest_move_z.*/
 int32_t prefer_target_x; /*< [pixel] prefer_target_x.*/
 int32_t prefer_target_y; /*< [pixel] prefer_target_y.*/
 int32_t prefer_target_z; /*< [pixel] prefer_target_z.*/
 float reservered_param[4]; /*< [none] Total current.*/
}) mavlink_avoidcontrolflow_t;

#define MAVLINK_MSG_ID_AvoidControlFlow_LEN 40
#define MAVLINK_MSG_ID_AvoidControlFlow_MIN_LEN 40
#define MAVLINK_MSG_ID_1234_LEN 40
#define MAVLINK_MSG_ID_1234_MIN_LEN 40

#define MAVLINK_MSG_ID_AvoidControlFlow_CRC 31
#define MAVLINK_MSG_ID_1234_CRC 31

#define MAVLINK_MSG_AvoidControlFlow_FIELD_RESERVERED_PARAM_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AvoidControlFlow { \
    1234, \
    "AvoidControlFlow", \
    7, \
    {  { "suggest_move_x", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_avoidcontrolflow_t, suggest_move_x) }, \
         { "suggest_move_y", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_avoidcontrolflow_t, suggest_move_y) }, \
         { "suggest_move_z", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_avoidcontrolflow_t, suggest_move_z) }, \
         { "prefer_target_x", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_avoidcontrolflow_t, prefer_target_x) }, \
         { "prefer_target_y", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_avoidcontrolflow_t, prefer_target_y) }, \
         { "prefer_target_z", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_avoidcontrolflow_t, prefer_target_z) }, \
         { "reservered_param", NULL, MAVLINK_TYPE_FLOAT, 4, 24, offsetof(mavlink_avoidcontrolflow_t, reservered_param) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AvoidControlFlow { \
    "AvoidControlFlow", \
    7, \
    {  { "suggest_move_x", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_avoidcontrolflow_t, suggest_move_x) }, \
         { "suggest_move_y", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_avoidcontrolflow_t, suggest_move_y) }, \
         { "suggest_move_z", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_avoidcontrolflow_t, suggest_move_z) }, \
         { "prefer_target_x", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_avoidcontrolflow_t, prefer_target_x) }, \
         { "prefer_target_y", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_avoidcontrolflow_t, prefer_target_y) }, \
         { "prefer_target_z", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_avoidcontrolflow_t, prefer_target_z) }, \
         { "reservered_param", NULL, MAVLINK_TYPE_FLOAT, 4, 24, offsetof(mavlink_avoidcontrolflow_t, reservered_param) }, \
         } \
}
#endif

/**
 * @brief Pack a avoidcontrolflow message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param suggest_move_x [pixel] suggest_move_x.
 * @param suggest_move_y [pixel] suggest_move_y.
 * @param suggest_move_z [pixel] suggest_move_z.
 * @param prefer_target_x [pixel] prefer_target_x.
 * @param prefer_target_y [pixel] prefer_target_y.
 * @param prefer_target_z [pixel] prefer_target_z.
 * @param reservered_param [none] Total current.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_avoidcontrolflow_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t suggest_move_x, int32_t suggest_move_y, int32_t suggest_move_z, int32_t prefer_target_x, int32_t prefer_target_y, int32_t prefer_target_z, const float *reservered_param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AvoidControlFlow_LEN];
    _mav_put_int32_t(buf, 0, suggest_move_x);
    _mav_put_int32_t(buf, 4, suggest_move_y);
    _mav_put_int32_t(buf, 8, suggest_move_z);
    _mav_put_int32_t(buf, 12, prefer_target_x);
    _mav_put_int32_t(buf, 16, prefer_target_y);
    _mav_put_int32_t(buf, 20, prefer_target_z);
    _mav_put_float_array(buf, 24, reservered_param, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AvoidControlFlow_LEN);
#else
    mavlink_avoidcontrolflow_t packet;
    packet.suggest_move_x = suggest_move_x;
    packet.suggest_move_y = suggest_move_y;
    packet.suggest_move_z = suggest_move_z;
    packet.prefer_target_x = prefer_target_x;
    packet.prefer_target_y = prefer_target_y;
    packet.prefer_target_z = prefer_target_z;
    mav_array_memcpy(packet.reservered_param, reservered_param, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AvoidControlFlow_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AvoidControlFlow;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AvoidControlFlow_MIN_LEN, MAVLINK_MSG_ID_AvoidControlFlow_LEN, MAVLINK_MSG_ID_AvoidControlFlow_CRC);
}

/**
 * @brief Pack a avoidcontrolflow message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param suggest_move_x [pixel] suggest_move_x.
 * @param suggest_move_y [pixel] suggest_move_y.
 * @param suggest_move_z [pixel] suggest_move_z.
 * @param prefer_target_x [pixel] prefer_target_x.
 * @param prefer_target_y [pixel] prefer_target_y.
 * @param prefer_target_z [pixel] prefer_target_z.
 * @param reservered_param [none] Total current.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_avoidcontrolflow_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t suggest_move_x,int32_t suggest_move_y,int32_t suggest_move_z,int32_t prefer_target_x,int32_t prefer_target_y,int32_t prefer_target_z,const float *reservered_param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AvoidControlFlow_LEN];
    _mav_put_int32_t(buf, 0, suggest_move_x);
    _mav_put_int32_t(buf, 4, suggest_move_y);
    _mav_put_int32_t(buf, 8, suggest_move_z);
    _mav_put_int32_t(buf, 12, prefer_target_x);
    _mav_put_int32_t(buf, 16, prefer_target_y);
    _mav_put_int32_t(buf, 20, prefer_target_z);
    _mav_put_float_array(buf, 24, reservered_param, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AvoidControlFlow_LEN);
#else
    mavlink_avoidcontrolflow_t packet;
    packet.suggest_move_x = suggest_move_x;
    packet.suggest_move_y = suggest_move_y;
    packet.suggest_move_z = suggest_move_z;
    packet.prefer_target_x = prefer_target_x;
    packet.prefer_target_y = prefer_target_y;
    packet.prefer_target_z = prefer_target_z;
    mav_array_memcpy(packet.reservered_param, reservered_param, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AvoidControlFlow_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AvoidControlFlow;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AvoidControlFlow_MIN_LEN, MAVLINK_MSG_ID_AvoidControlFlow_LEN, MAVLINK_MSG_ID_AvoidControlFlow_CRC);
}

/**
 * @brief Encode a avoidcontrolflow struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param avoidcontrolflow C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_avoidcontrolflow_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_avoidcontrolflow_t* avoidcontrolflow)
{
    return mavlink_msg_avoidcontrolflow_pack(system_id, component_id, msg, avoidcontrolflow->suggest_move_x, avoidcontrolflow->suggest_move_y, avoidcontrolflow->suggest_move_z, avoidcontrolflow->prefer_target_x, avoidcontrolflow->prefer_target_y, avoidcontrolflow->prefer_target_z, avoidcontrolflow->reservered_param);
}

/**
 * @brief Encode a avoidcontrolflow struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param avoidcontrolflow C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_avoidcontrolflow_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_avoidcontrolflow_t* avoidcontrolflow)
{
    return mavlink_msg_avoidcontrolflow_pack_chan(system_id, component_id, chan, msg, avoidcontrolflow->suggest_move_x, avoidcontrolflow->suggest_move_y, avoidcontrolflow->suggest_move_z, avoidcontrolflow->prefer_target_x, avoidcontrolflow->prefer_target_y, avoidcontrolflow->prefer_target_z, avoidcontrolflow->reservered_param);
}

/**
 * @brief Send a avoidcontrolflow message
 * @param chan MAVLink channel to send the message
 *
 * @param suggest_move_x [pixel] suggest_move_x.
 * @param suggest_move_y [pixel] suggest_move_y.
 * @param suggest_move_z [pixel] suggest_move_z.
 * @param prefer_target_x [pixel] prefer_target_x.
 * @param prefer_target_y [pixel] prefer_target_y.
 * @param prefer_target_z [pixel] prefer_target_z.
 * @param reservered_param [none] Total current.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_avoidcontrolflow_send(mavlink_channel_t chan, int32_t suggest_move_x, int32_t suggest_move_y, int32_t suggest_move_z, int32_t prefer_target_x, int32_t prefer_target_y, int32_t prefer_target_z, const float *reservered_param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AvoidControlFlow_LEN];
    _mav_put_int32_t(buf, 0, suggest_move_x);
    _mav_put_int32_t(buf, 4, suggest_move_y);
    _mav_put_int32_t(buf, 8, suggest_move_z);
    _mav_put_int32_t(buf, 12, prefer_target_x);
    _mav_put_int32_t(buf, 16, prefer_target_y);
    _mav_put_int32_t(buf, 20, prefer_target_z);
    _mav_put_float_array(buf, 24, reservered_param, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AvoidControlFlow, buf, MAVLINK_MSG_ID_AvoidControlFlow_MIN_LEN, MAVLINK_MSG_ID_AvoidControlFlow_LEN, MAVLINK_MSG_ID_AvoidControlFlow_CRC);
#else
    mavlink_avoidcontrolflow_t packet;
    packet.suggest_move_x = suggest_move_x;
    packet.suggest_move_y = suggest_move_y;
    packet.suggest_move_z = suggest_move_z;
    packet.prefer_target_x = prefer_target_x;
    packet.prefer_target_y = prefer_target_y;
    packet.prefer_target_z = prefer_target_z;
    mav_array_memcpy(packet.reservered_param, reservered_param, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AvoidControlFlow, (const char *)&packet, MAVLINK_MSG_ID_AvoidControlFlow_MIN_LEN, MAVLINK_MSG_ID_AvoidControlFlow_LEN, MAVLINK_MSG_ID_AvoidControlFlow_CRC);
#endif
}

/**
 * @brief Send a avoidcontrolflow message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_avoidcontrolflow_send_struct(mavlink_channel_t chan, const mavlink_avoidcontrolflow_t* avoidcontrolflow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_avoidcontrolflow_send(chan, avoidcontrolflow->suggest_move_x, avoidcontrolflow->suggest_move_y, avoidcontrolflow->suggest_move_z, avoidcontrolflow->prefer_target_x, avoidcontrolflow->prefer_target_y, avoidcontrolflow->prefer_target_z, avoidcontrolflow->reservered_param);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AvoidControlFlow, (const char *)avoidcontrolflow, MAVLINK_MSG_ID_AvoidControlFlow_MIN_LEN, MAVLINK_MSG_ID_AvoidControlFlow_LEN, MAVLINK_MSG_ID_AvoidControlFlow_CRC);
#endif
}

#if MAVLINK_MSG_ID_AvoidControlFlow_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_avoidcontrolflow_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t suggest_move_x, int32_t suggest_move_y, int32_t suggest_move_z, int32_t prefer_target_x, int32_t prefer_target_y, int32_t prefer_target_z, const float *reservered_param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, suggest_move_x);
    _mav_put_int32_t(buf, 4, suggest_move_y);
    _mav_put_int32_t(buf, 8, suggest_move_z);
    _mav_put_int32_t(buf, 12, prefer_target_x);
    _mav_put_int32_t(buf, 16, prefer_target_y);
    _mav_put_int32_t(buf, 20, prefer_target_z);
    _mav_put_float_array(buf, 24, reservered_param, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AvoidControlFlow, buf, MAVLINK_MSG_ID_AvoidControlFlow_MIN_LEN, MAVLINK_MSG_ID_AvoidControlFlow_LEN, MAVLINK_MSG_ID_AvoidControlFlow_CRC);
#else
    mavlink_avoidcontrolflow_t *packet = (mavlink_avoidcontrolflow_t *)msgbuf;
    packet->suggest_move_x = suggest_move_x;
    packet->suggest_move_y = suggest_move_y;
    packet->suggest_move_z = suggest_move_z;
    packet->prefer_target_x = prefer_target_x;
    packet->prefer_target_y = prefer_target_y;
    packet->prefer_target_z = prefer_target_z;
    mav_array_memcpy(packet->reservered_param, reservered_param, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AvoidControlFlow, (const char *)packet, MAVLINK_MSG_ID_AvoidControlFlow_MIN_LEN, MAVLINK_MSG_ID_AvoidControlFlow_LEN, MAVLINK_MSG_ID_AvoidControlFlow_CRC);
#endif
}
#endif

#endif

// MESSAGE AvoidControlFlow UNPACKING


/**
 * @brief Get field suggest_move_x from avoidcontrolflow message
 *
 * @return [pixel] suggest_move_x.
 */
static inline int32_t mavlink_msg_avoidcontrolflow_get_suggest_move_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field suggest_move_y from avoidcontrolflow message
 *
 * @return [pixel] suggest_move_y.
 */
static inline int32_t mavlink_msg_avoidcontrolflow_get_suggest_move_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field suggest_move_z from avoidcontrolflow message
 *
 * @return [pixel] suggest_move_z.
 */
static inline int32_t mavlink_msg_avoidcontrolflow_get_suggest_move_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field prefer_target_x from avoidcontrolflow message
 *
 * @return [pixel] prefer_target_x.
 */
static inline int32_t mavlink_msg_avoidcontrolflow_get_prefer_target_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field prefer_target_y from avoidcontrolflow message
 *
 * @return [pixel] prefer_target_y.
 */
static inline int32_t mavlink_msg_avoidcontrolflow_get_prefer_target_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field prefer_target_z from avoidcontrolflow message
 *
 * @return [pixel] prefer_target_z.
 */
static inline int32_t mavlink_msg_avoidcontrolflow_get_prefer_target_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field reservered_param from avoidcontrolflow message
 *
 * @return [none] Total current.
 */
static inline uint16_t mavlink_msg_avoidcontrolflow_get_reservered_param(const mavlink_message_t* msg, float *reservered_param)
{
    return _MAV_RETURN_float_array(msg, reservered_param, 4,  24);
}

/**
 * @brief Decode a avoidcontrolflow message into a struct
 *
 * @param msg The message to decode
 * @param avoidcontrolflow C-struct to decode the message contents into
 */
static inline void mavlink_msg_avoidcontrolflow_decode(const mavlink_message_t* msg, mavlink_avoidcontrolflow_t* avoidcontrolflow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    avoidcontrolflow->suggest_move_x = mavlink_msg_avoidcontrolflow_get_suggest_move_x(msg);
    avoidcontrolflow->suggest_move_y = mavlink_msg_avoidcontrolflow_get_suggest_move_y(msg);
    avoidcontrolflow->suggest_move_z = mavlink_msg_avoidcontrolflow_get_suggest_move_z(msg);
    avoidcontrolflow->prefer_target_x = mavlink_msg_avoidcontrolflow_get_prefer_target_x(msg);
    avoidcontrolflow->prefer_target_y = mavlink_msg_avoidcontrolflow_get_prefer_target_y(msg);
    avoidcontrolflow->prefer_target_z = mavlink_msg_avoidcontrolflow_get_prefer_target_z(msg);
    mavlink_msg_avoidcontrolflow_get_reservered_param(msg, avoidcontrolflow->reservered_param);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AvoidControlFlow_LEN? msg->len : MAVLINK_MSG_ID_AvoidControlFlow_LEN;
        memset(avoidcontrolflow, 0, MAVLINK_MSG_ID_AvoidControlFlow_LEN);
    memcpy(avoidcontrolflow, _MAV_PAYLOAD(msg), len);
#endif
}
