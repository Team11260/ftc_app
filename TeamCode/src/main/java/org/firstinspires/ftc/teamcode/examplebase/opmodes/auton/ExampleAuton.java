package org.firstinspires.ftc.teamcode.examplebase.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.examplebase.hardware.Robot;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAuton;

@Autonomous(name = "Example Auton", group = "Example")
@Disabled

public class ExampleAuton extends AbstractAuton {

    private Robot robot;

    @Override
    public void RegisterStates() {
        //addState(new State("{state name}", "{previous state name}", {state callable}));
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

    }

    @Override
    public void Stop() {
        robot.stop();
    }
}
