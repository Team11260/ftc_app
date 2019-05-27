package org.firstinspires.ftc.teamcode.bogiebase.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants;
import org.firstinspires.ftc.teamcode.bogiebase.hardware.Robot;
import org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAutonNew;
import org.firstinspires.ftc.teamcode.framework.util.PathState;
import org.firstinspires.ftc.teamcode.framework.util.State;
import org.upacreekrobotics.dashboard.Dashboard;

@Autonomous(name = "Bogie Auton Crater Dump", group = "New")
//@Disabled

public class BogieAutonCraterDump extends AbstractAutonNew {

    Robot robot;

    @Override
    public void RegisterStates() {
        addState(new State("telemetry", "start", () ->{
            while (opModeIsActive()) {
                robot.updateAll();
            }
        }));
        addState(new State("auton release wheels sequence", "start", robot.autonReleaseWheelsSequenceCallable()));
        addState(new State("auton mineral lift zero sequence", "start", robot.autonLowerMineralLiftSequenceCallable()));
        addState(new PathState("finish lowering robot lift", "turn to gold mineral", robot.finishRobotLiftToBottomSequenceCallable()));
        addState(new PathState("intaking pause", "drive to minerals", () -> {
            RobotState.currentPath.pause();
            delay(Constants.DUMP_ROUTE_INTAKING_DELAY);
            RobotState.currentPath.resume();
        }));
        addState(new PathState("begin intaking", "turn to gold mineral", robot.beginIntakingCallable()));
        addState(new PathState("finish intaking", "back up from minerals", robot.finishIntakingCallable()));
        addState(new PathState("raise lift", "back up from minerals", robot.autonMoveMineralLiftToDumpPositionSequenceCallable()));
        addState(new PathState("dump pause", "drive to lander", () -> {
            RobotState.currentPath.pause();
            delay(Constants.DUMP_MINERAL_DELAY);
            RobotState.currentPath.resume();
        }));
        addState(new PathState("open mineral gate", "drive to lander", robot.openMineralGateCallable()));
        addState(new PathState("lower lift", "turn to wall", robot.autonMoveMineralLiftToCollectPositionSequenceCallable()));
        addState(new PathState("stop drive to wall", "large drive to wall", robot.autonDriveToWallSequenceCallable()));
        addState(new PathState("drop marker", "drive to depot", robot.dropMarkerCallable()));
        addState(new PathState("stop robot on crater","drive to depot",() -> {
            while (robot.getPitch() < 6);
            RobotState.currentPath.nextSegment();
        }));
    }

    @Override
    public void Init() {

        telemetry.putBoolean("teleop_position", false);

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
                robot.runDrivePath(Constants.collectRightMineralDump);
                break;
            case LEFT:
                robot.runDrivePath(Constants.collectLeftMineralDump);
                break;
            case CENTER:
                robot.runDrivePath(Constants.collectCenterMineralDump);
                break;
            default:
                robot.runDrivePath(Constants.collectRightMineralDump);
                break;
        }

        //Dump mineral
        robot.runDrivePath(Constants.dumpMineral);

        //Deposit team marker and drive to crater
        robot.runDrivePath(Constants.craterSideToCrater);
    }

    @Override
    public void Stop() {
        robot.stop();

        //Start Teleop mode
        Dashboard.startOpMode(Constants.OPMODE_TO_START_AFTER_AUTON);
    }
}
