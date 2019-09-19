package org.firstinspires.ftc.teamcode.rishiandmilan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractTeleop;

@Autonomous(name = "RishiMilanTeleOpMode", group = "Example")
//@Disabled

public class RishiMilanTeleOpMode extends AbstractTeleop {

    private DcMotor dcMotorFrontLeft;
    private DcMotor dcMotorBackLeft;
    private DcMotor dcMotorFrontRight;
    private DcMotor dcMotorBackRight;

    @Override
    public void RegisterEvents() {
        //addEventHandler("{event name}", {event handler callable});
    }

    @Override
    public void UpdateEvents() {
        //NEVER EVER PUT BLOCKING CODE HERE!!!
    }

    @Override
    public void Init() {

        dcMotorFrontLeft=hardwareMap.dcMotor.get("front_left");
        dcMotorBackLeft=hardwareMap.dcMotor.get("back_left");
        dcMotorFrontRight=hardwareMap.dcMotor.get("front_right");
        dcMotorBackRight=hardwareMap.dcMotor.get("back_right");
        dcMotorBackRight.setDirection(DcMotor.Direction.REVERSE);
        dcMotorFrontRight.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void Loop() {

        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

        dcMotorBackLeft.setPower(leftPower);
        dcMotorFrontLeft.setPower(leftPower);

        dcMotorBackRight.setPower(rightPower);
        dcMotorFrontRight.setPower(rightPower);

    }

    @Override
    public void Stop() {

    }




}

