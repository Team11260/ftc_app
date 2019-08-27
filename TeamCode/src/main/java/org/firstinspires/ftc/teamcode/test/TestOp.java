package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.framework.AbstractTeleop;
import org.firstinspires.ftc.teamcode.framework.userhardware.outputs.SlewDcMotor;

@TeleOp(name="Test_TestOp_New", group="New")
//@Disabled

public class TestOp extends AbstractTeleop {

    double x;

    SlewDcMotor motor;

    ElapsedTime runtime;

    @Override
    public void Init() {
        runtime = new ElapsedTime();
        telemetry.addData("hi");
        telemetry.addData("it's Matthew");
        telemetry.update();
    }

    @Override
    public void Start(){
        runtime.reset();
    }

    @Override
    public void Loop() {
        telemetry.addDataDS(runtime.milliseconds());
        telemetry.update();
    }
}