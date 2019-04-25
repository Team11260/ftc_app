package org.firstinspires.ftc.teamcode.bogiebase.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants;
import org.firstinspires.ftc.teamcode.bogiebase.hardware.Robot;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAutonNew;

@Autonomous(name = "Bogie Auton Spline Test", group = "New")
//@Disabled

public class BogieAutonSplineTest extends AbstractAutonNew {

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
        robot.stopTensorFlow();

        robot.runDrivePath(Constants.splineTest);
    }

    @Override
    public void Stop() {
        robot.stop();
    }
}
