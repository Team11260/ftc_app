package org.firstinspires.ftc.teamcode.bogiebase.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants;
import org.firstinspires.ftc.teamcode.bogiebase.hardware.Robot;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAutonNew;

@Autonomous(name = "Test Splines", group = "New")
@Disabled

public class TestSplines extends AbstractAutonNew {

    Robot robot;

    @Override
    public void RegisterStates() {

    }

    @Override
    public void Init() {
        robot = new Robot();
        robot.stopTensorFlow();
    }

    @Override
    public void Run() {
        robot.runDrivePath(Constants.splineToDepot);
    }

    @Override
    public void Stop() {
        robot.stop();
    }
}
