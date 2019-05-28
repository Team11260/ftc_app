package org.firstinspires.ftc.teamcode.examplebase.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.examplebase.hardware.Robot;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAuton;
import org.firstinspires.ftc.teamcode.framework.util.State;

@Autonomous(name = "Example Auton", group = "Example")
//@Disabled

public class ExampleAuton extends AbstractAuton {

    private Robot robot;

    @Override
    public void RegisterStates() {
        //addState(new State("{state name}", "{previous state name}", {state callable}));
        addState(new State("update", "start", () -> {
            while (opModeIsActive()) robot.updateAll();
        }));
    }

    @Override
    public void Init() {
        //Init robot
        robot = new Robot();
    }

    @Override
    public void InitLoop(int loop) {

    }

    @Override
    public void Run() {

        robot.driveInches(8);
        robot.turnLeft();
        robot.driveInches(8);

    }

    @Override
    public void Stop() {
        robot.stop();
    }
}
