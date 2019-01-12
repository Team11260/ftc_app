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

@Autonomous(name = "Boogie Auton Depot", group = "New")
//@Disabled

public class BoogieAutonDepot extends AbstractAutonNew {

    Robot robot;
    TensorFlow tensorFlow;

    @Override
    public void RegisterStates() {
        addState(new State("auton release wheels sequence", "start", robot.autonReleaseWheelsSequenceCallable()));
        addState(new State("auton mineral lift zero sequence", "start", robot.autonLowerMineralLiftSequenceCallable()));
        addState(new PathState("finish lowering robot lift", "turn to gold mineral", robot.finishRobotLiftToBottomSequenceCallable()));
        addState(new PathState("intaking pause", "drive to minerals", ()->{
            while (RobotState.currentPath.getCurrentSegment().getName().equals("drive to minerals"));
            RobotState.currentPath.pause();
            delay(1000);
            RobotState.currentPath.resume();
            return true;
        }));
        addState(new PathState("begin intaking", "turn to gold mineral", robot.beginIntakingCallable()));
        addState(new PathState("finish intaking", "turn to depot", robot.finishIntakingCallable()));
        addState(new PathState("drop marker", "drive to depot", robot.dropMarkerCallable()));
        //addState(new PathState("drive to wall with distance", "turn to wall", robot.autonDriveToWallSequenceCallable()));
    }

    @Override
    public void Init() {
        //Init robot
        robot = new Robot();

        //Init object recognition
        tensorFlow = new TensorFlow(TensorFlow.CameraOrientation.VERTICAL, "Webcam 1", false);

        RobotState.currentSamplePosition = UNKNOWN;
        telemetry.addData(DoubleTelemetry.LogMode.INFO, "Current Sample Position: " + RobotState.currentSamplePosition);
        telemetry.update();
    }

    @Override
    public void InitLoop(int loop) {

        //Object recognition loop
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
        //Stop object recognition
        tensorFlow.stop();

        //Lower robot
        robot.moveRobotLiftToBottom();

        //Collect gold mineral
        switch (RobotState.currentSamplePosition) {
            case RIGHT:
                robot.runDrivePath(Constants.collectDepotRightMineral);
                break;
            case LEFT:
                robot.runDrivePath(Constants.collectDepotLeftMineral);
                break;
            case CENTER:
                robot.runDrivePath(Constants.collectDepotCenterMineral);
                break;
            default:
                robot.runDrivePath(Constants.collectDepotCenterMineral);
                break;
        }

        //Deposit team marker and drive to crater
        robot.runDrivePath(Constants.depotSideToCrater);
    }

    @Override
    public void Stop() {
        tensorFlow.stop();
        robot.stop();

        //Start Teleop mode
        Dashboard.startOpMode("TwoGamepad Boogie Teleop Tankdrive");
    }
}
