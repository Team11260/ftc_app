package org.upacreekrobotics.dashboard;

public enum MessageType {
    /* status (also serve as a heartbeat) */
    ROBOT_STATUS,
    RESTART_ROBOT,
    HAND_SHAKE,
    BATTERY,
    PHONE_BATTERY,

    /* op mode management */
    GET_OP_MODES,
    OP_MODES,
    SELECT_OP_MODE,

    VARIABLE,

    INIT_OP_MODE,
    RUN_OP_MODE,
    STOP_OP_MODE,

    /* telemetry */
    GET_VALUE,
    RETURN_VALUE,
    INFO,
    TELEMETRY,
    LOG,

    SMARTDASHBOARD_PUT,
    SMARTDASHBOARD_GET,

    GAMEPAD_1_SET,
    GAMEPAD_2_SET;

    public String getMessage(MessageType type) {
        return type.toString();
    }
}