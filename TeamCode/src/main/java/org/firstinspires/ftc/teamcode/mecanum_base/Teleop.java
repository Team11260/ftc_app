package org.firstinspires.ftc.teamcode.mecanum_base;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.framework.AbstractTeleop;
import org.firstinspires.ftc.teamcode.mecanum_base.hardware.HardwareDevices;

@TeleOp(name="Mecanum_TeleOp_Stable", group="New")
//@Disabled

public class Teleop extends AbstractTeleop {

    private HardwareDevices robot;
    DcMotor intakeMotor;
    CRServo vexMotor;

    @Override
    public void Init()
    {
        intakeMotor = hardwareMap.dcMotor.get("intake");
        vexMotor = hardwareMap.crservo.get("vex");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vexMotor.setPower(0);
        intakeMotor.setPower(0);

        robot = new HardwareDevices();

    }

    @Override
    public void Loop() {
        robot.updateDrive();
        telemetry.update();
    }

    @Override
    public void Stop() {
        robot.stop();
    }

    @Override
    public void lsx_change(double lsx) {
        robot.setDriveX(-lsx);
    }

    @Override
    public void lsy_change(double lsy) {
        robot.setDriveY(lsy);
    }

    @Override
    public void rsx_change(double rsx) {
        robot.setDriveZ(rsx);
    }

    @Override
    public void a_down()
    {
        intakeMotor.setPower(0.5);
    }

    @Override
    public void b_down()
    {
        intakeMotor.setPower(-0.5);
    }

    @Override
    public void x_down()
    {
        vexMotor.setPower(0.5);
    }

    @Override
    public void y_down()
    {
        vexMotor.setPower(-0.5);
    }

    @Override
    public void a_up()
    {
        intakeMotor.setPower(0);
    }

    @Override
    public void b_up()
    {
        intakeMotor.setPower(0);
    }

    @Override
    public void x_up()
    {
        vexMotor.setPower(0);
    }

    @Override
    public void y_up()
    {
        vexMotor.setPower(0);
    }


}