package org.firstinspires.ftc.teamcode.boogiewheel_base.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.boogiewheel_base.hardware.Constants;
import org.firstinspires.ftc.teamcode.boogiewheel_base.hardware.Robot;
import org.firstinspires.ftc.teamcode.boogiewheel_base.hardware.RobotState;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAutonNew;
import org.firstinspires.ftc.teamcode.framework.userHardware.DoubleTelemetry;
import org.firstinspires.ftc.teamcode.framework.userHardware.inputs.sensors.vision.SamplePosition;
import org.firstinspires.ftc.teamcode.framework.userHardware.inputs.sensors.vision.TensorFlow;
import org.firstinspires.ftc.teamcode.framework.util.PathState;
import org.firstinspires.ftc.teamcode.framework.util.State;
import org.upacreekrobotics.dashboard.Dashboard;

import static org.firstinspires.ftc.teamcode.framework.userHardware.inputs.sensors.vision.SamplePosition.UNKNOWN;

@Autonomous(name = "Boogie Auton Double Sample Dump", group = "New")
//@Disabled

public class BoogieAutonDoubleSampleDump extends AbstractAutonNew {

    Robot robot;
    TensorFlow tensorFlow;

    @Override
    public void RegisterStates() {
        addState(new State("auton release wheels sequence", "start", robot.autonReleaseWheelsSequenceCallable()));
        addState(new State("auton mineral lift zero sequence", "start", robot.autonLowerMineralLiftSequenceCallable()));
        addState(new PathState("finish lowering robot lift", "turn to gold mineral", robot.finishRobotLiftToBottomSequenceCallable()));
        addState(new PathState("begin intaking", "turn to gold mineral", robot.beginIntakingCallable()));
        addState(new PathState("intaking pause", "drive to minerals", ()->{
            while (!RobotState.currentPath.getCurrentSegment().getName().equals("back up from minerals"));
            RobotState.currentPath.pause();
            delay(750);
            RobotState.currentPath.resume();
            return true;
        }));
        addState(new PathState("finish intaking", "turn to wall", robot.finishIntakingCallable()));
        addState(new PathState("finish intaking", "orient at depot", robot.finishIntakingCallable()));
        addState(new PathState("stop drive to wall", "large drive to wall", robot.autonDriveToWallSequenceCallable()));
        addState(new PathState("drop marker", "orient at depot", robot.dropMarkerCallable()));
        addState(new PathState("raise lift", "drive to crater", robot.moveMineralLiftToDumpPositionCallable()));
        addState(new PathState("open mineral gate", "drive to lander", robot.openMineralGateCallable()));
        addState(new PathState("dump pause", "drive to lander", ()->{
            while (!RobotState.currentPath.getCurrentSegment().getName().equals("drive away from lander"));
            RobotState.currentPath.pause();
            delay(1000);
            RobotState.currentPath.resume();
            return true;
        }));
        addState(new PathState("lower lift", "drive away from lander", robot.autonMoveMineralLiftToCollectPositionSequenceCallable()));
    }

    @Override
    public void Init() {
        robot = new Robot();
        tensorFlow = new TensorFlow(TensorFlow.CameraOrientation.VERTICAL, "Webcam 1", false);

        RobotState.currentSamplePosition = UNKNOWN;
        telemetry.addData(DoubleTelemetry.LogMode.INFO, "Current Sample Position: " + RobotState.currentSamplePosition);
        telemetry.update();
    }

    @Override
    public void InitLoop(int loop) {

        //Object recognition loop
        if (loop % 5 == 0) tensorFlow.restart();

        //Init object recognition
        SamplePosition currentPosition = tensorFlow.getSamplePosition();

        if (currentPosition != RobotState.currentSamplePosition && currentPosition != UNKNOWN) {
            RobotState.currentSamplePosition = currentPosition;

            telemetry.addData(DoubleTelemetry.LogMode.INFO, "Current Sample Position: " + currentPosition.toString());
            telemetry.update();
        }
    }

    @Override
    public void Run() {
        //Stop object recognition
        tensorFlow.stop();

        //Lower robot
        robot.moveRobotLiftToBottom();

        //Collect first gold mineral
        switch (RobotState.currentSamplePosition) {
            case RIGHT:
                robot.runDrivePath(Constants.collectRightMineral);
                break;
            case LEFT:
                robot.runDrivePath(Constants.collectLeftMineral);
                break;
            case CENTER:
                robot.runDrivePath(Constants.collectCenterMineral);
                break;
            default:
                robot.runDrivePath(Constants.collectCenterMineral);
                break;
        }

        //Drive to depot and prepare for second sample
        robot.runDrivePath(Constants.craterSideToDepotDoubleSample);

        //Collect second gold mineral
        switch (RobotState.currentSamplePosition) {
            case RIGHT:
                robot.runDrivePath(Constants.collectRightMineralDoubleSample);
                break;
            case LEFT:
                robot.runDrivePath(Constants.collectLeftMineralDoubleSample);
                break;
            case CENTER:
                robot.runDrivePath(Constants.collectCenterMineralDoubleSample);
                break;
            default:
                robot.runDrivePath(Constants.collectCenterMineralDoubleSample);
                break;
        }

        //Deposit team marker and drive to crater
        robot.runDrivePath(Constants.doubleSampleDepotToLander);

        //Drive to crater
        switch (RobotState.currentSamplePosition) {
            case RIGHT:
                robot.runDrivePath(Constants.doubleSampleLanderToCraterRight);
                break;
            case LEFT:
                robot.runDrivePath(Constants.doubleSampleLanderToCraterLeft);
                break;
            case CENTER:
                robot.runDrivePath(Constants.doubleSampleLanderToCraterCenter);
                break;
            default:
                robot.runDrivePath(Constants.doubleSampleLanderToCraterCenter);
                break;
        }
    }

    @Override
    public void Stop() {
        tensorFlow.stop();
        robot.stop();

        //Start Teleop mode
        Dashboard.startOpMode(Constants.OPMODE_TO_START_AFTER_AUTON);
    }
}