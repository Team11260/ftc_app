package org.firstinspires.ftc.teamcode.boogiewheel_base.hardware;

import org.firstinspires.ftc.teamcode.framework.userHardware.paths.DriveSegment;
import org.firstinspires.ftc.teamcode.framework.userHardware.paths.Path;
import org.firstinspires.ftc.teamcode.framework.userHardware.paths.TurnSegment;

public final class Constants {
    ////////DRIVE////////
    public static final double DRIVE_MINERAL_LIFT_RAISED_SCALAR = 0.5;
    public static final double DRIVE_COUNTS_PER_INCH = 38.0;

    public static final double DRIVE_RELEASE_WHEELS_POWER = -0.5;

    public static final boolean DRIVE_AUTO_INVERT = false;

    public static final double DRIVE_TEAM_MARKER_EXTENDED = 0.5;
    public static final double DRIVE_TEAM_MARKER_RETRACTED = 0;


    ////////INTAKE////////
    //Brushes
    public static final double INTAKE_STOP_POWER = 0;
    public static final double INTAKE_FORWARD_POWER = 1;
    public static final double INTAKE_REVERSE_POWER = -1;
    public static final double INTAKE_LOWER_POWER = 0.2;

    //Lift
    public final static double INTAKE_LIFT_LOWERED_POSITION = 1;
    public final static double INTAKE_LIFT_RAISED_POSITION = 0.2;


    ////////MINERAL LIFT////////
    //Lift
    public final static int MINERAL_LIFT_COLLECT_POSITION = -4000;
    public final static int MINERAL_LIFT_DUMP_POSITION = 3600;

    //Gate
    public final static double MINERAL_GATE_OPEN_POSITION = 0.7;
    public final static double MINERAL_GATE_CLOSED_POSITION = 0.0;


    ////////ROBOT LIFT////////
    //Lift
    public final static int ROBOT_LIFT_LOWERED_POSITION = -2500;
    public final static int ROBOT_LIFT_RAISED_POSITION = 100;
    public final static int ROBOT_LIFT_RELEASE_PAWL_POSITION = 200;
    public final static double ROBOT_LIFT_LOWER_POWER = -0.7;

    //Pawl
    public final static double ROBOT_LIFT_PAWL_RELEASED = 0.1;
    public final static double ROBOT_LIFT_PAWL_ENGAGED = 0.0;


    ////////AUTON PATHS////////
    public final static double AUTON_PATH_SPEED = 1;
    public final static double AUTON_TURN_ERROR = 8;
    public final static double AUTON_MINERAL_TURN_ERROR = 4;
    public final static int AUTON_TURN_PERIOD = 800;
    public final static int AUTON_DISTANCE_ERROR = 20;
    public final static double DISTANCE_TO_WALL = 15.0;
    public final static double DISTANCE_TO_DEPOT_WALL = 12.0;

    public final static Path collectRightMineral = new Path("collect right mineral");
    static {
        collectRightMineral.addSegment(new TurnSegment("turn to gold mineral", 152, AUTON_PATH_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        collectRightMineral.addSegment(new DriveSegment("drive to minerals", 28, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectRightMineral.addSegment(new DriveSegment("back up from minerals", -14, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path collectLeftMineral = new Path("collect left mineral");
    static {
        collectLeftMineral.addSegment(new TurnSegment("start turning", 160, AUTON_PATH_SPEED, 100, 0));
        collectLeftMineral.addSegment(new TurnSegment("turn to gold mineral", -144, AUTON_PATH_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        collectLeftMineral.addSegment(new DriveSegment("drive to minerals", 25, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectLeftMineral.addSegment(new DriveSegment("back up from minerals", -12, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path collectCenterMineral = new Path("collect center mineral");
    static {
        collectCenterMineral.addSegment(new TurnSegment("start turning", 160, AUTON_PATH_SPEED, 100, 0));
        collectCenterMineral.addSegment(new TurnSegment("turn to gold mineral", 180, AUTON_PATH_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        collectCenterMineral.addSegment(new DriveSegment("drive to minerals", 26, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectCenterMineral.addSegment(new DriveSegment("back up from minerals", -14, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path craterSideToCrater = new Path("crater side to crater");
    static {
        craterSideToCrater.addSegment(new TurnSegment("turn to wall", -90, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        craterSideToCrater.addSegment(new DriveSegment("large drive to wall", 45, AUTON_PATH_SPEED, 500));
        craterSideToCrater.addSegment(new DriveSegment("drive to wall", 20, 0.4, AUTON_DISTANCE_ERROR));
        craterSideToCrater.addSegment(new TurnSegment("turn to depot", -53, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        craterSideToCrater.addSegment(new DriveSegment("drive to depot", 32, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        craterSideToCrater.addSegment(new TurnSegment("orient at depot", -48, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        craterSideToCrater.addSegment(new DriveSegment("drive to crater", -75, AUTON_PATH_SPEED, 40, -33));
    }

    public final static Path craterSideToDepotDoubleSample = new Path("crater side to depot double sample");
    static {
        craterSideToDepotDoubleSample.addSegment(new TurnSegment("turn to wall", -90, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        craterSideToDepotDoubleSample.addSegment(new DriveSegment("large drive to wall", 45, AUTON_PATH_SPEED, 500));
        craterSideToDepotDoubleSample.addSegment(new DriveSegment("drive to wall", 20, 0.4, AUTON_DISTANCE_ERROR));
        craterSideToDepotDoubleSample.addSegment(new TurnSegment("turn to depot", -45, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        craterSideToDepotDoubleSample.addSegment(new DriveSegment("large drive to depot double sample", 42, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR, -49));
        craterSideToDepotDoubleSample.addSegment(new DriveSegment("drive to depot double sample", 30, 0.4, AUTON_DISTANCE_ERROR));
    }

    public final static Path collectRightMineralDoubleSample = new Path("collect right mineral double sample");
    static {
        collectRightMineralDoubleSample.addSegment(new TurnSegment("turn to gold mineral", 119, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        collectRightMineralDoubleSample.addSegment(new DriveSegment("drive to minerals", 26, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectRightMineralDoubleSample.addSegment(new DriveSegment("back up from minerals", -24, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path collectLeftMineralDoubleSample = new Path("collect left mineral double sample");
    static {
        collectLeftMineralDoubleSample.addSegment(new TurnSegment("turn to gold mineral", 63, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        collectLeftMineralDoubleSample.addSegment(new DriveSegment("drive to minerals", 26, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectLeftMineralDoubleSample.addSegment(new DriveSegment("back up from minerals", -24, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path collectCenterMineralDoubleSample = new Path("collect center mineral double sample");
    static {
        collectCenterMineralDoubleSample.addSegment(new TurnSegment("turn to gold mineral", 92, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        collectCenterMineralDoubleSample.addSegment(new DriveSegment("drive to minerals", 30, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectCenterMineralDoubleSample.addSegment(new DriveSegment("back up from minerals", -31, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path depotToCraterDoubleSample = new Path("depot to crater double sample");
    static {
        depotToCraterDoubleSample.addSegment(new TurnSegment("turn to crater", -26, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        depotToCraterDoubleSample.addSegment(new DriveSegment("small drive to crater", -4, AUTON_PATH_SPEED, 40, -37));
        depotToCraterDoubleSample.addSegment(new DriveSegment("drive to crater", -86, AUTON_PATH_SPEED, 40, -37));
    }

    public final static Path depotToCraterDoubleSampleDump = new Path("depot to lander double sample dump");
    static {
        depotToCraterDoubleSampleDump.addSegment(new TurnSegment("turn to crater", -27, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        depotToCraterDoubleSampleDump.addSegment(new DriveSegment("drive to crater", -40, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR, -37));
        depotToCraterDoubleSampleDump.addSegment(new TurnSegment("turn from wall", -90, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        depotToCraterDoubleSampleDump.addSegment(new DriveSegment("drive to lander", -48, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        depotToCraterDoubleSampleDump.addSegment(new TurnSegment("turn to lander", 180, AUTON_PATH_SPEED, AUTON_TURN_ERROR, AUTON_TURN_PERIOD));
        depotToCraterDoubleSampleDump.addSegment(new DriveSegment("drive to lander", -5, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path depotSideToCrater = new Path("depot to crater");
    static {
        depotSideToCrater.addSegment(new TurnSegment("turn to wall", -60, AUTON_PATH_SPEED, AUTON_TURN_ERROR,AUTON_TURN_PERIOD));
        depotSideToCrater.addSegment(new DriveSegment("drive to crater", 80, AUTON_PATH_SPEED, 40, -50));

    }

    public final static Path collectDepotRightMineral = new Path("collect right mineral depot");
    static {
        collectDepotRightMineral.addSegment(new TurnSegment("turn to gold mineral", 154, AUTON_PATH_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        collectDepotRightMineral.addSegment(new DriveSegment("drive to minerals", 34, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectDepotRightMineral.addSegment(new TurnSegment("turn to depot",-145,AUTON_PATH_SPEED,AUTON_TURN_ERROR,AUTON_TURN_PERIOD));
        collectDepotRightMineral.addSegment(new DriveSegment("drive to depot", 29, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path collectDepotLeftMineral = new Path("collect left mineral depot");
    static {
        collectDepotLeftMineral.addSegment(new TurnSegment("start turning", 160, AUTON_PATH_SPEED, 100, 0));
        collectDepotLeftMineral.addSegment(new TurnSegment("turn to gold mineral", -148, AUTON_PATH_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        collectDepotLeftMineral.addSegment(new DriveSegment("drive to minerals", 34, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectDepotLeftMineral.addSegment(new TurnSegment("turn to depot",166,AUTON_PATH_SPEED,AUTON_TURN_ERROR,AUTON_TURN_PERIOD));
        collectDepotLeftMineral.addSegment(new DriveSegment("drive to depot", 24, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }

    public final static Path collectDepotCenterMineral = new Path("collect center mineral depot");
    static {
        collectDepotCenterMineral.addSegment(new TurnSegment("start turning", 160, AUTON_PATH_SPEED, 100, 0));
        collectDepotCenterMineral.addSegment(new TurnSegment("turn to gold mineral", 179, AUTON_PATH_SPEED, AUTON_MINERAL_TURN_ERROR, AUTON_TURN_PERIOD));
        collectDepotCenterMineral.addSegment(new DriveSegment("drive to minerals", 24, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
        collectDepotCenterMineral.addSegment(new TurnSegment("turn to depot",-165,AUTON_PATH_SPEED,AUTON_TURN_ERROR,AUTON_TURN_PERIOD));
        collectDepotCenterMineral.addSegment(new DriveSegment("drive to depot", 30, AUTON_PATH_SPEED, AUTON_DISTANCE_ERROR));
    }
}