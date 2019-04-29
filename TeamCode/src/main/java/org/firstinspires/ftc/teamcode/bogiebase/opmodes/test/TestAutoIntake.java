package org.firstinspires.ftc.teamcode.bogiebase.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants;
import org.firstinspires.ftc.teamcode.bogiebase.hardware.Robot;
import org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAutonNew;
import org.firstinspires.ftc.teamcode.framework.util.PathState;
import org.firstinspires.ftc.teamcode.framework.util.State;

@Autonomous(name = "Auto Intake Test", group = "New")
@Disabled

public class TestAutoIntake extends AbstractAutonNew {

    Robot robot;

    @Override
    public void RegisterStates() {
        addState(new State("pitch", "start", () -> {
            while (opModeIsActive())
                telemetry.getSmartdashboard().putValue("Pitch", robot.getPitch());
            }));
        addState(new State("finish driving", "start", () -> {
            while (robot.getPitch() > -6);
            RobotState.currentPath.nextSegment();
        }));
        addState(new PathState("intake", "drive into crater", () -> {
            RobotState.currentPath.pause();
            robot.beginIntaking();
            delay(2000);
            robot.reverseIntake();
            delay(1000);
            robot.finishIntaking();
            RobotState.currentPath.resume();
        }));
        addState(new State("drive into crater", "start", () -> {
            while (robot.getPitch() < 6);
            while (robot.getPitch() > -6);
            while (robot.getPitch() < 2);
            RobotState.currentPath.nextSegment();
        }));
    }

    @Override
    public void Init() {
        robot = new Robot();
        robot.stopTensorFlow();
    }

    @Override
    public void Run() {

        robot.autonLowerMineralLiftSequence();

        robot.runDrivePath(Constants.pickupMinerals);
    }
}
