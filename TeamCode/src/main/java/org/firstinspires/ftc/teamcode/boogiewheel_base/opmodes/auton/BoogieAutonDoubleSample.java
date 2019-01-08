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

import static org.firstinspires.ftc.teamcode.framework.userHardware.inputs.sensors.vision.SamplePosition.UNKNOWN;

@Autonomous(name = "Boogie Auton Double Sample", group = "New")
//@Disabled

public class BoogieAutonDoubleSample extends AbstractAutonNew {

    Robot robot;
    TensorFlow tensorFlow;

    @Override
    public void RegisterStates() {
        addState(new State("auton release wheels sequence", "start", robot.autonReleaseWheelsSequenceCallable()));
        addState(new State("auton mineral lift zero sequence", "start", robot.autonLowerMineralLiftSequenceCallable()));
        addState(new PathState("finish lowering robot lift", "turn to gold mineral", robot.finishRobotLiftToBottomSequenceCallable()));
        addState(new PathState("begin intaking", "turn to gold mineral", robot.beginIntakingCallable()));
        addState(new PathState("intaking pause", "drive to minerals", ()->{
            while (RobotState.currentPath.getCurrentSegment().getName().equals("drive to minerals"));
            RobotState.currentPath.pause();
            delay(1000);
            RobotState.currentPath.resume();
            return true;
        }));
        addState(new PathState("finish intaking", "back up from minerals", robot.finishIntakingCallable()));
        addState(new PathState("drop marker", "small drive to crater", robot.dropMarkerCallable()));
        addState(new PathState("drive to wall with distance", "large drive to wall", robot.autonDriveToWallSequenceCallable()));
        addState(new PathState("drive to wall with distance", "large drive to depot double sample", robot.autonDriveToWallSequenceCallable()));
        //addState(new PathState("raise mineral lift", "turn from wall", robot.moveMineralLiftToDumpPositionCallable()));
        //addState(new PathState("dump minerals", "drive to lander", robot.toggleMineralGateCallable()));
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
        if (loop % 5 == 0) tensorFlow.restart();

        SamplePosition currentPosition = tensorFlow.getSamplePosition();

        if (currentPosition != RobotState.currentSamplePosition && currentPosition != UNKNOWN) {
            RobotState.currentSamplePosition = currentPosition;

            telemetry.addData(DoubleTelemetry.LogMode.INFO, "Current Sample Position: " + currentPosition.toString());
            telemetry.update();
        }
    }

    @Override
    public void Run() {
        tensorFlow.stop();

        robot.moveRobotLiftToBottom();

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

        robot.runDrivePath(Constants.craterSideToDepotDoubleSample);

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

        robot.runDrivePath(Constants.depotToCraterDoubleSample);
    }

    @Override
    public void Stop() {
        tensorFlow.stop();
        robot.stop();
        //Dashboard.startOpMode("TwoGamepad Boogie Teleop Tankdrive");
    }
}
