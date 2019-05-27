package org.firstinspires.ftc.teamcode.bogiebase.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants;
import org.firstinspires.ftc.teamcode.bogiebase.hardware.Robot;
import org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAutonNew;
import org.firstinspires.ftc.teamcode.framework.util.PathState;
import org.firstinspires.ftc.teamcode.framework.util.State;
import org.upacreekrobotics.dashboard.Dashboard;

@Autonomous(name = "Bogie Auton Depot Dump", group = "New")
//@Disabled

public class BogieAutonDepotDump extends AbstractAutonNew {

    Robot robot;

    @Override
    public void RegisterStates() {
        addState(new State("auton release wheels sequence", "start", robot.autonReleaseWheelsSequenceCallable()));
        addState(new State("auton mineral lift zero sequence", "start", robot.autonLowerMineralLiftSequenceCallable()));
        addState(new PathState("finish lowering robot lift", "turn to gold mineral", robot.finishRobotLiftToBottomSequenceCallable()));
        addState(new PathState("intaking pause", "turn to depot", () -> {
            RobotState.currentPath.pause();
            delay(Constants.NORMAL_INTAKING_DELAY);
            RobotState.currentPath.resume();
        }));
        addState(new PathState("begin intaking", "turn to gold mineral", robot.beginIntakingCallable()));
        addState(new PathState("finish intaking", "drive to depot", robot.finishIntakingCallable()));
        addState(new PathState("drop marker", "drive to depot", robot.dropMarkerCallable()));
        addState(new PathState("raise lift", "drive away from depot", robot.autonMoveMineralLiftToDumpPositionSequenceCallable()));
        addState(new PathState("dump pause", "turn away from lander", () -> {
            RobotState.currentPath.pause();
            delay(Constants.DUMP_MINERAL_DELAY);
            RobotState.currentPath.resume();
        }));
        addState(new PathState("open mineral gate", "turn away from lander", robot.openMineralGateCallable()));
        addState(new PathState("lower lift", "turn to wall", robot.autonMoveMineralLiftToCollectPositionSequenceCallable()));
        addState(new PathState("stop drive to wall", "large drive to wall", robot.autonDriveToWallSequenceCallable()));
        addState(new PathState("finish driving", "turn to crater", () -> {
            while (robot.getPitch() > -6);
            RobotState.currentPath.nextSegment();
        }));
        addState(new PathState("intake", "drive into crater", () -> {
            RobotState.currentPath.pause();
            robot.beginIntaking();
            delay(1500);
            robot.reverseIntake();
            delay(1500);
            robot.finishIntaking();
            RobotState.currentPath.resume();
        }));
    }

    @Override
    public void Init() {

        telemetry.putBoolean("teleop_position", true);

        //Init robot
        robot = new Robot();
    }

    @Override
    public void InitLoop(int loop) {
        //Update the sample position using tensorflow
        robot.updateSamplePosition(loop);
    }

    @Override
    public void Run() {

        //Stop object recognition
        robot.stopTensorFlow();

        //Lower robot
        robot.moveRobotLiftToBottom();

        //Collect gold mineral
        switch (RobotState.currentSamplePosition) {
            case RIGHT:
                robot.runDrivePath(Constants.collectDepotDumpRightMineral);
                break;
            case LEFT:
                robot.runDrivePath(Constants.collectDepotDumpLeftMineral);
                break;
            case CENTER:
                robot.runDrivePath(Constants.collectDepotDumpCenterMineral);
                break;
            default:
                robot.runDrivePath(Constants.collectDepotDumpRightMineral);
                break;
        }

        //Deposit team marker and drive to crater
        robot.runDrivePath(Constants.depotSideToCraterDump);

        //Drives into crater and intakes minerals
        robot.runDrivePath(Constants.pickupMinerals);
    }

    @Override
    public void Stop() {
        robot.stop();

        //Start Teleop mode
        Dashboard.startOpMode(Constants.OPMODE_TO_START_AFTER_AUTON);
    }
}
