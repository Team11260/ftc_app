package org.firstinspires.ftc.teamcode.examplebase.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.examplebase.hardware.Robot;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractTeleop;

@TeleOp(name = "Example Teleop", group = "Example")
@Disabled

public class ExampleTeleop extends AbstractTeleop {

    private Robot robot;

    @Override
    public void RegisterEvents() {
        //addEventHandler("{event name}", {callable event handler});
    }

    @Override
    public void UpdateEvents() {
        //NEVER EVER PUT BLOCKING CODE HERE!!!
        //checkBooleanInput("{event name}", {current value});
    }

    @Override
    public void Init() {
        robot = new Robot();
    }

    @Override
    public void Loop() {
        robot.updateAll();
    }

    @Override
    public void Stop() {
        robot.stop();
    }
}