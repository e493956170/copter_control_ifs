// MESSAGE AvoidControlFlow support class

#pragma once

namespace mavlink {
namespace ardupilotmega {
namespace msg {

/**
 * @brief AvoidControlFlow message
 *
 * Avoidance control cmd Flow.
 */
struct AvoidControlFlow : mavlink::Message {
    static constexpr msgid_t MSG_ID = 1234;
    static constexpr size_t LENGTH = 40;
    static constexpr size_t MIN_LENGTH = 40;
    static constexpr uint8_t CRC_EXTRA = 31;
    static constexpr auto NAME = "AvoidControlFlow";


    int32_t suggest_move_x; /*< [pixel] suggest_move_x. */
    int32_t suggest_move_y; /*< [pixel] suggest_move_y. */
    int32_t suggest_move_z; /*< [pixel] suggest_move_z. */
    int32_t prefer_target_x; /*< [pixel] prefer_target_x. */
    int32_t prefer_target_y; /*< [pixel] prefer_target_y. */
    int32_t prefer_target_z; /*< [pixel] prefer_target_z. */
    std::array<float, 4> reservered_param; /*< [none] Total current. */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  suggest_move_x: " << suggest_move_x << std::endl;
        ss << "  suggest_move_y: " << suggest_move_y << std::endl;
        ss << "  suggest_move_z: " << suggest_move_z << std::endl;
        ss << "  prefer_target_x: " << prefer_target_x << std::endl;
        ss << "  prefer_target_y: " << prefer_target_y << std::endl;
        ss << "  prefer_target_z: " << prefer_target_z << std::endl;
        ss << "  reservered_param: [" << to_string(reservered_param) << "]" << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << suggest_move_x;                // offset: 0
        map << suggest_move_y;                // offset: 4
        map << suggest_move_z;                // offset: 8
        map << prefer_target_x;               // offset: 12
        map << prefer_target_y;               // offset: 16
        map << prefer_target_z;               // offset: 20
        map << reservered_param;              // offset: 24
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> suggest_move_x;                // offset: 0
        map >> suggest_move_y;                // offset: 4
        map >> suggest_move_z;                // offset: 8
        map >> prefer_target_x;               // offset: 12
        map >> prefer_target_y;               // offset: 16
        map >> prefer_target_z;               // offset: 20
        map >> reservered_param;              // offset: 24
    }
};

} // namespace msg
} // namespace ardupilotmega
} // namespace mavlink
