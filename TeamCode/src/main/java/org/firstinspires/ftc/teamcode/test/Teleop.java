package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.framework.AbstractTeleop;
import org.firstinspires.ftc.teamcode.framework.userHardware.inputs.sensors.vision.vuforia.VuforiaTest;
import org.firstinspires.ftc.teamcode.test.hardware.Robot;

@TeleOp(name="Test_TeleOp_New", group="New")
//@Disabled

public class Teleop extends AbstractTeleop {

    private Robot robot;

    private DcMotor motor;

    VuforiaTest vuforia;

    @Override
    public void Init() {
        robot = new Robot();
        //motor = hardwareMap.dcMotor.get("left_motor");
        vuforia = new VuforiaTest();
    }

    @Override
    public void Start(){
        telemetry.addData("Start");
    }

    @Override
    public void Loop() {
        vuforia.getPixel();
        //robot.delay(1000);
    }

    @Override
    public void Stop() {
        robot.stop();
    }

    @Override
    public void a_down(){
        telemetry.addData("a down");
        robot.delay(10000);
        telemetry.addData("a done");
    }

    @Override
    public void b_down(){
        telemetry.addData("b down");
        robot.delay(10000);
        telemetry.addData("b done");
    }
}