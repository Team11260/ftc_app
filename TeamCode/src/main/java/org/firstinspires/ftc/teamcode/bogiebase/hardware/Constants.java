package org.firstinspires.ftc.teamcode.bogiebase.hardware;

import org.firstinspires.ftc.teamcode.framework.userhardware.paths.DriveSegment;
import org.firstinspires.ftc.teamcode.framework.userhardware.paths.Path;
import org.firstinspires.ftc.teamcode.framework.userhardware.paths.TurnSegment;
import org.upacreekrobotics.dashboard.Config;

@Config
public final class Constants {
    ////////Opmodes////////
    public static final int NORMAL_INTAKING_DELAY = 1000;
    public static final int DUMP_ROUTE_INTAKING_DELAY = 1500;

    public static final int DUMP_MINERAL_DELAY = 1000;

    public static final String OPMODE_TO_START_AFTER_AUTON = "Bogie Teleop Two Driver";


    ////////DRIVE////////
    public static final double DRIVE_SLEW_SPEED = 0.1;

    public static final double DRIVE_MINERAL_LIFT_RAISED_SCALAR = 0.55;
    public static final double DRIVE_COUNTS_PER_INCH = 38.0;

    public static final double DRIVE_RELEASE_WHEELS_POWER = -0.5;

    public static final double DRIVE_TEAM_MARKER_EXTENDED = 0.5;
    public static final double DRIVE_TEAM_MARKER_RETRACTED = 0;
    public static final double DRIVE_TEAM_MARKER_TELEOP_RETRACTED = DRIVE_TEAM_MARKER_RETRACTED;

    public static final int DRIVE_RELEASE_WHEEL_DELAY = 1000;
    public static final int DRIVE_DUMP_TEAM_MARKER_DELAY = 1000;

    //Roadrunner
    public static double TRACK_WIDTH = 26; // in

    public final static double kV = 0.04;
    public final static double kA = 0.025;
    public final static double kStatic = 0.01;


    ////////INTAKE////////
    //Brushes
    public static final double INTAKE_STOP_POWER = 0;
    public static final double INTAKE_FORWARD_POWER = 1;
    public static final double INTAKE_REVERSE_POWER = -1;


    ////////MINERAL LIFT////////
    //Lift
    public final static double MINERAL_LIFT_FULL_SPEED = 1;
    public final static double MINERAL_LIFT_AUTON_SPEED = 0.5;
    public final static double MINERAL_LIFT_SLOW_SPEED = 0.5;

    public final static int MINERAL_LIFT_AUTON_RAISED_POSITION = 225;
    public final static int MINERAL_LIFT_DUMP_POSITION = 800;
    public final static int MINERAL_LIFT_DUMP_ANGLE_TRIGGER_POSITION = 700;
    public final static int MINERAL_LIFT_SLOW_SPEED_TRIGGER_POSITION = 150;

    //Gate
    public final static double MINERAL_LIFT_GATE_OPEN_POSITION = 0.75;
    public final static double MINERAL_LIFT_GATE_HOLD_POSITION = 0.37;
    public final static double MINERAL_LIFT_GATE_CLOSED_POSITION = 0.1;

    //Angle Servo
    public final static double MINERAL_LIFT_ANGLE_SERVO_HORIZONTAL_POSITION = 0.08;
    public final static double MINERAL_LIFT_ANGLE_SERVO_STEEP_DUMP_POSITION = 0.42;
    public final static double MINERAL_LIFT_ANGLE_SERVO_SHALLOW_DUMP_POSITION = 0.35;
    public final static double MINERAL_LIFT_ANGLE_SERVO_VERTICAL_POSITION = 0.65;


    ////////ROBOT LIFT////////
    //Lift
    public final static int ROBOT_LIFT_LOWERED_POSITION = -2500;
    public final static int ROBOT_LIFT_RAISED_POSITION = 100;
    public final static int ROBOT_LIFT_RELEASE_PAWL_POSITION = 210;
    public final static double ROBOT_LIFT_LOWER_POWER = -0.7;

    public static final int ROBOT_LIFT_AUTON_DELAY = 600;

    //Pawl
    public final static double ROBOT_LIFT_PAWL_RELEASED = 0.1;
    public final static double ROBOT_LIFT_PAWL_ENGAGED = 0.0;


    ////////AUTON PATHS////////
    //Path variables
    public final static double AUTON_PATH_SPEED = 1;
    public final static double AUTON_MINERAL_TURN_SPEED = 0.4;
    public final static double AUTON_TURN_ERROR = 8;
    public final static double AUTON_MINERAL_TURN_ERROR = 3;
    public final static int AUTON_TURN_PERIOD = 250;
    public final static int AUTON_DISTANCE_ERROR = 20;


    //Crater Side
    public final static Path collectRightMineral = new Path("collect right mineral");

    static {
        collectRightMineral.addSegment(new TurnSegment("turn to gold mineral", 153, AUTON_MINERAL_TURN_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        collectRightMineral.addSegment(new DriveSegment("drive to minerals", 30, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectRightMineral.addSegment(new DriveSegment("back up from minerals", -14, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path collectLeftMineral = new Path("collect left mineral");

    static {
        collectLeftMineral.addSegment(new TurnSegment("turn to gold mineral", 209, AUTON_MINERAL_TURN_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        collectLeftMineral.addSegment(new DriveSegment("drive to minerals", 30, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectLeftMineral.addSegment(new DriveSegment("back up from minerals", -14, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path collectCenterMineral = new Path("collect center mineral");

    static {
        collectCenterMineral.addSegment(new TurnSegment("turn to gold mineral", 181, AUTON_MINERAL_TURN_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        collectCenterMineral.addSegment(new DriveSegment("drive to minerals", 28, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectCenterMineral.addSegment(new DriveSegment("back up from minerals", -15, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path craterSideToCrater = new Path("crater side to crater");

    static {
        craterSideToCrater.addSegment(new TurnSegment("turn to wall", -90, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        craterSideToCrater.addSegment(new DriveSegment("large drive to wall", 40, AUTON_PATH_SPEED, 500));
        craterSideToCrater.addSegment(new DriveSegment("drive to wall", 50, 0.4, AUTON_DISTANCE_ERROR));
        craterSideToCrater.addSegment(new TurnSegment("turn to depot", -55, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        craterSideToCrater.addSegment(new DriveSegment("drive to depot", 32, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        craterSideToCrater.addSegment(new DriveSegment("drive to crater", -74, AUTON_PATH_SPEED, 40, -25));
    }


    //Crater side dump
    public final static Path collectRightMineralDump = new Path("collect right mineral dump");

    static {
        collectRightMineralDump.addSegment(new TurnSegment("turn to gold mineral", 153, AUTON_MINERAL_TURN_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        collectRightMineralDump.addSegment(new DriveSegment("drive to minerals", 28, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectRightMineralDump.addSegment(new DriveSegment("back up from minerals", -16, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectRightMineralDump.addSegment(new TurnSegment("turn to lander", 153, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
    }

    public final static Path collectLeftMineralDump = new Path("collect left mineral dump");

    static {
        collectLeftMineralDump.addSegment(new TurnSegment("turn to gold mineral", 209, AUTON_MINERAL_TURN_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        collectLeftMineralDump.addSegment(new DriveSegment("drive to minerals", 28, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectLeftMineralDump.addSegment(new DriveSegment("back up from minerals", -12, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectLeftMineralDump.addSegment(new TurnSegment("turn to lander", 178, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
    }

    public final static Path collectCenterMineralDump = new Path("collect center mineral dump");

    static {
        collectCenterMineralDump.addSegment(new TurnSegment("turn to gold mineral", 180, AUTON_MINERAL_TURN_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        collectCenterMineralDump.addSegment(new DriveSegment("drive to minerals", 26, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectCenterMineralDump.addSegment(new DriveSegment("back up from minerals", -12, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectCenterMineralDump.addSegment(new TurnSegment("turn to lander", 162, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
    }

    public final static Path dumpMineral = new Path("single sample dump");

    static {
        dumpMineral.addSegment(new DriveSegment("drive to lander", -16, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        dumpMineral.addSegment(new DriveSegment("drive away from lander", 14, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }


    //Double Sample
    public final static Path craterSideToDepotDoubleSample = new Path("crater side to depot double sample");

    static {
        craterSideToDepotDoubleSample.addSegment(new TurnSegment("turn to wall", -90, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        craterSideToDepotDoubleSample.addSegment(new DriveSegment("large drive to wall", 40, AUTON_PATH_SPEED, 500));
        craterSideToDepotDoubleSample.addSegment(new DriveSegment("drive to wall", 50, 0.4, AUTON_DISTANCE_ERROR));
        craterSideToDepotDoubleSample.addSegment(new TurnSegment("turn to depot", -60, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        craterSideToDepotDoubleSample.addSegment(new DriveSegment("large drive to depot double sample", 21, AUTON_PATH_SPEED, 100, -57));
        craterSideToDepotDoubleSample.addSegment(new DriveSegment("drive to depot double sample", 26, AUTON_PATH_SPEED, 80, 0));
    }

    public final static Path collectRightMineralDoubleSample = new Path("collect right mineral double sample");

    static {
        collectRightMineralDoubleSample.addSegment(new TurnSegment("turn to gold mineral", 123, AUTON_MINERAL_TURN_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        collectRightMineralDoubleSample.addSegment(new DriveSegment("drive to minerals", 33, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectRightMineralDoubleSample.addSegment(new DriveSegment("back up from minerals double sample", -33, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path collectLeftMineralDoubleSample = new Path("collect left mineral double sample");

    static {
        collectLeftMineralDoubleSample.addSegment(new TurnSegment("turn to gold mineral", 52, AUTON_MINERAL_TURN_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        collectLeftMineralDoubleSample.addSegment(new DriveSegment("drive to minerals", 32, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectLeftMineralDoubleSample.addSegment(new DriveSegment("back up from minerals double sample", -32, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path collectCenterMineralDoubleSample = new Path("collect center mineral double sample");

    static {
        collectCenterMineralDoubleSample.addSegment(new TurnSegment("turn to gold mineral", 89, AUTON_MINERAL_TURN_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        collectCenterMineralDoubleSample.addSegment(new DriveSegment("drive to minerals", 30, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectCenterMineralDoubleSample.addSegment(new DriveSegment("back up from minerals double sample", -31, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path depotToCraterDoubleSample = new Path("depot to crater double sample");

    static {
        depotToCraterDoubleSample.addSegment(new TurnSegment("orient at depot", -55, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        depotToCraterDoubleSample.addSegment(new TurnSegment("turn to crater", 150, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        depotToCraterDoubleSample.addSegment(new DriveSegment("drive to crater", 82, AUTON_PATH_SPEED, 40, 145));
    }


    //Double sample dump
    public final static Path doubleSampleDepotToLander = new Path("double sample dump depot to dump");

    static {
        doubleSampleDepotToLander.addSegment(new TurnSegment("orient at depot", -55, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        doubleSampleDepotToLander.addSegment(new TurnSegment("turn to crater", -18, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        doubleSampleDepotToLander.addSegment(new DriveSegment("drive to crater", -24, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR, -42));
        doubleSampleDepotToLander.addSegment(new DriveSegment("drive away from wall", -67, AUTON_PATH_SPEED, 80, -110));
        doubleSampleDepotToLander.addSegment(new TurnSegment("turn to lander", 180, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        doubleSampleDepotToLander.addSegment(new DriveSegment("drive to lander", -20, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        doubleSampleDepotToLander.addSegment(new DriveSegment("drive away from lander", 14, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path doubleSampleLanderToCraterRight = new Path("double sample dump lander to crater right");

    static {
        doubleSampleLanderToCraterRight.addSegment(new TurnSegment("turn to crater", 140, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        doubleSampleLanderToCraterRight.addSegment(new DriveSegment("final drive to crater", 30, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path doubleSampleLanderToCraterLeft = new Path("double sample dump lander to crater left");

    static {
        doubleSampleLanderToCraterLeft.addSegment(new TurnSegment("turn to crater", -140, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        doubleSampleLanderToCraterLeft.addSegment(new DriveSegment("final drive to crater", 30, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path doubleSampleLanderToCraterCenter = new Path("double sample dump lander to crater center");

    static {
        doubleSampleLanderToCraterCenter.addSegment(new TurnSegment("turn to crater", -175, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        doubleSampleLanderToCraterCenter.addSegment(new DriveSegment("final drive to crater", 30, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }


    //Depot Routes
    public final static Path collectDepotRightMineral = new Path("collect right mineral depot");

    static {
        collectDepotRightMineral.addSegment(new TurnSegment("turn to gold mineral", 152, AUTON_MINERAL_TURN_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        collectDepotRightMineral.addSegment(new DriveSegment("drive to minerals", 34, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectDepotRightMineral.addSegment(new TurnSegment("turn to depot", -145, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        collectDepotRightMineral.addSegment(new DriveSegment("drive to depot", 26, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path collectDepotLeftMineral = new Path("collect left mineral depot");

    static {
        collectDepotLeftMineral.addSegment(new TurnSegment("turn to gold mineral", 215, AUTON_MINERAL_TURN_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        collectDepotLeftMineral.addSegment(new DriveSegment("drive to minerals", 34, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectDepotLeftMineral.addSegment(new TurnSegment("turn to depot", 155, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        collectDepotLeftMineral.addSegment(new DriveSegment("drive to depot", 22, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path collectDepotCenterMineral = new Path("collect center mineral depot");

    static {
        collectDepotCenterMineral.addSegment(new TurnSegment("turn to gold mineral", 181, AUTON_MINERAL_TURN_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        collectDepotCenterMineral.addSegment(new DriveSegment("drive to minerals", 24, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectDepotCenterMineral.addSegment(new TurnSegment("turn to depot", -165, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        collectDepotCenterMineral.addSegment(new DriveSegment("drive to depot", 25, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path depotSideToCrater = new Path("depot to crater");

    static {
        depotSideToCrater.addSegment(new TurnSegment("turn to wall", -60, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        depotSideToCrater.addSegment(new DriveSegment("drive to crater", 74, AUTON_PATH_SPEED, 40, -53));
    }

    //Depot Dump Routes
    public final static Path collectDepotDumpRightMineral = new Path("collect right mineral depot");

    static {
        collectDepotDumpRightMineral.addSegment(new TurnSegment("turn to gold mineral", 153, AUTON_MINERAL_TURN_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        collectDepotDumpRightMineral.addSegment(new DriveSegment("drive to minerals", 34, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectDepotDumpRightMineral.addSegment(new TurnSegment("turn to depot", -145, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        collectDepotDumpRightMineral.addSegment(new DriveSegment("drive to depot", 18, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectDepotDumpRightMineral.addSegment(new DriveSegment("drive away from depot", -18, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectDepotDumpRightMineral.addSegment(new TurnSegment("turn to lander", 145, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        collectDepotDumpRightMineral.addSegment(new DriveSegment("drive to lander", -35, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));

    }

    public final static Path collectDepotDumpLeftMineral = new Path("collect left mineral depot");

    static {
        collectDepotDumpLeftMineral.addSegment(new TurnSegment("turn to gold mineral", 210, AUTON_MINERAL_TURN_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        collectDepotDumpLeftMineral.addSegment(new DriveSegment("drive to minerals", 34, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectDepotDumpLeftMineral.addSegment(new TurnSegment("turn to depot", 156, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        collectDepotDumpLeftMineral.addSegment(new DriveSegment("drive to depot", 20, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectDepotDumpLeftMineral.addSegment(new DriveSegment("drive away from depot", -20, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectDepotDumpLeftMineral.addSegment(new TurnSegment("turn to lander", -146, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        collectDepotDumpLeftMineral.addSegment(new DriveSegment("drive to lander", -31, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));

    }

    public final static Path collectDepotDumpCenterMineral = new Path("collect center mineral depot");

    static {
        collectDepotDumpCenterMineral.addSegment(new TurnSegment("turn to gold mineral", 181, AUTON_MINERAL_TURN_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        collectDepotDumpCenterMineral.addSegment(new DriveSegment("drive to minerals", 22, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectDepotDumpCenterMineral.addSegment(new TurnSegment("turn to depot", -165, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        collectDepotDumpCenterMineral.addSegment(new DriveSegment("drive to depot", 24, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectDepotDumpCenterMineral.addSegment(new DriveSegment("drive away from depot", -22, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectDepotDumpCenterMineral.addSegment(new TurnSegment("turn to lander", 175, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        collectDepotDumpCenterMineral.addSegment(new DriveSegment("drive to lander", -23, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));

    }

    public final static Path depotSideToCraterDump = new Path("lander to crater depot dump");

    static {
        depotSideToCraterDump.addSegment(new TurnSegment("turn away from lander", 180, AUTON_MINERAL_TURN_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        depotSideToCraterDump.addSegment(new DriveSegment("drive away from lander", 11, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        depotSideToCraterDump.addSegment(new TurnSegment("turn to wall", -90, AUTON_MINERAL_TURN_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        depotSideToCraterDump.addSegment(new DriveSegment("large drive to wall", 44, AUTON_PATH_SPEED, 500));
        depotSideToCraterDump.addSegment(new DriveSegment("drive to wall", 50, 0.4, AUTON_DISTANCE_ERROR));
        depotSideToCraterDump.addSegment(new TurnSegment("turn to crater", -50, AUTON_MINERAL_TURN_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
    }

    //Test Routes
    public final static Path testRightTurn = new Path("tests right turn");

    static {
        testRightTurn.addSegment(new TurnSegment("turn gold right", 151, AUTON_MINERAL_TURN_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
    }

    public final static Path testLeftTurn = new Path("tests left turn");

    static {
        testLeftTurn.addSegment(new TurnSegment("turn to gold mineral", 210, AUTON_MINERAL_TURN_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
    }

    public final static Path testCenterTurn = new Path("tests center turn");

    static {
        testCenterTurn.addSegment(new TurnSegment("turn to gold mineral", 181, AUTON_MINERAL_TURN_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
    }

    public final static Path pickupMinerals = new Path("pickup minerals");

    static {
        pickupMinerals.addSegment(new DriveSegment("drive to crater", 100, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        pickupMinerals.addSegment(new DriveSegment("drive into crater", 22, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }
}