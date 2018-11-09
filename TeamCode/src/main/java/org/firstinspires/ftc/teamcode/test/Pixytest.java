package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.framework.AbstractAuton;
import org.firstinspires.ftc.teamcode.framework.userHardware.inputs.sensors.vision.Pixycam;
import org.firstinspires.ftc.teamcode.test.hardware.Robot;

@Autonomous(name="Pixy_Auto_New", group="New")
//@Disabled

public class Pixytest extends AbstractAuton {

    public Pixycam myPixy;
    public Robot myRobot;

    @Override
    public void Init() {

        myPixy = hardwareMap.get(Pixycam.class, "pixy");
        myRobot = new Robot();

    }

    @Override
    public void Run() {

        while (true) {
            telemetry.addDataPhone(myPixy.getLargestBlock().getX());
            telemetry.update();
        }

    }
}
