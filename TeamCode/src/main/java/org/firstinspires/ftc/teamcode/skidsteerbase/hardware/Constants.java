package org.firstinspires.ftc.teamcode.skidsteerbase.hardware;


public final class Constants {
    ////////Opmodes////////


    ////////DRIVE////////
    public static final double DRIVE_SLEW_SPEED = 0.1;

    public static final double DRIVE_COUNTS_PER_INCH = 38.0;

    public static final double DRIVE_TEAM_MARKER_RETRACTED = 0;
    public static final double DRIVE_TEAM_MARKER_TELEOP_RETRACTED = DRIVE_TEAM_MARKER_RETRACTED;


    ////////ARM////////
    //Rotate
    public static final double ARM_SLEW_SPEED = 2;

    public static final int ARM_DUMP_POSITION = 3400;
    public static final int ARM_COLLECT_POSITION = 1400;

    //Intake
    public static final double INTAKE_FORWARD_POWER = 0.74;
    public static final double INTAKE_REVERSE_POWER = 0.26;
    public static final double INTAKE_STOP_POWER = 0.5;

    //Gate
    public static final double GATE_CLOSED_POSITION = 0;
    public static final double GATE_OPEN_POSITION = 0.7;

    //Angle
    public static final double ARM_ANGLE_FLAT = -0.1;
    public static final double ARM_ANGLE_INTAKE = 0.1;
    public static final double ARM_ANGLE_HOLD = 1;
    public static final double ARM_ANGLE_DUMP = 0.6;
}