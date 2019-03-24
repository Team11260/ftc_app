package org.firstinspires.ftc.teamcode.bogiebase.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants;
import org.firstinspires.ftc.teamcode.bogiebase.hardware.Robot;
import org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAutonNew;
import org.firstinspires.ftc.teamcode.framework.util.PathState;
import org.firstinspires.ftc.teamcode.framework.util.State;
import org.upacreekrobotics.dashboard.Dashboard;

@Autonomous(name = "test", group = "util")
//@Disabled

public class Test extends AbstractAutonNew {

    Robot robot;

    @Override
    public void RegisterStates() {
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
        //Collect gold mineral
        robot.resetDriveEncoders();
        switch (RobotState.currentSamplePosition) {
            case RIGHT:
                robot.runDrivePath(Constants.testRightTurn);
                break;
            case LEFT:
                robot.runDrivePath(Constants.testLeftTurn);
                break;
            case CENTER:
                robot.runDrivePath(Constants.testCenterTurn);
                break;
            default:
                robot.runDrivePath(Constants.testCenterTurn);
                break;
        }
    }

    @Override
    public void Stop() {
        robot.stop();
    }
}
