package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.framework.AbstractAuton;

@Autonomous(name="HardwareTest_Auto_New", group="New")
//@Disabled

public class HardwareTest extends AbstractAuton {

    @Override
    public void Init() {
        hardwareMap.dcMotor.get("Hi");
    }

    @Override
    public void Run() {

    }
}