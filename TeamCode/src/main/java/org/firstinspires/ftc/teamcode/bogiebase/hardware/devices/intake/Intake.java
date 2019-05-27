package org.firstinspires.ftc.teamcode.bogiebase.hardware.devices.intake;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants;

public class Intake {

    private DcMotorSimple intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorSimple.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setPower(0);
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }


    public void stop() {

    }
}