package org.firstinspires.ftc.teamcode.examplebase.hardware;

public final class RobotState {

    public static MatchState currentMatchState = MatchState.UNKNOWN;

    public enum MatchState {
        AUTONOMOUS,
        TELEOP,
        UNKNOWN
    }
}
