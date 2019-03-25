package org.firstinspires.ftc.teamcode.bogiebase.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants;
import org.firstinspires.ftc.teamcode.bogiebase.hardware.Robot;
import org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAutonNew;
import org.firstinspires.ftc.teamcode.framework.util.PathState;
import org.firstinspires.ftc.teamcode.framework.util.State;
import org.upacreekrobotics.dashboard.Dashboard;

@Autonomous(name = "Bogie Auton Crater", group = "New")
//@Disabled

public class BogieAutonCrater extends AbstractAutonNew {
    private AnalogInput pot;
    Robot robot;

    @Override
    public void RegisterStates() {
        addState(new State("auton release wheels sequence", "start", robot.autonReleaseWheelsSequenceCallable()));
        addState(new State("auton mineral lift zero sequence", "start", robot.autonLowerMineralLiftSequenceCallable()));
        addState(new PathState("finish lowering robot lift", "turn to gold mineral", robot.finishRobotLiftToBottomSequenceCallable()));
        addState(new PathState("intaking pause", "drive to minerals", () -> {
            RobotState.currentPath.pause();
            delay(Constants.NORMAL_INTAKING_DELAY);
            RobotState.currentPath.resume();
            return true;
        }));
        addState(new PathState("begin intaking", "turn to gold mineral", robot.beginIntakingCallable()));
        addState(new PathState("finish intaking", "turn to wall", robot.finishIntakingCallable()));
        addState(new PathState("stop drive to wall", "large drive to wall", robot.autonDriveToWallSequenceCallable()));
        addState(new PathState("drop marker", "drive to depot", robot.dropMarkerCallable()));
    }

    @Override
    public void Init() {

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

        delay(Constants.AUTON_CRATER_SIDE_PARTNER_DELAY);

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
