package org.firstinspires.ftc.teamcode.skidsteerbase.hardware.devices.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.skidsteerbase.hardware.Constants;
import org.firstinspires.ftc.teamcode.framework.userhardware.outputs.SlewDcMotor;

public class Arm {

    private SlewDcMotor rotateMotor;
    private Servo angleServo, intakeServo, gateServo;

    public Arm(HardwareMap hardwareMap){
        rotateMotor = new SlewDcMotor(hardwareMap.dcMotor.get("rotate_motor"));
        rotateMotor.setSlewSpeed(Constants.ARM_SLEW_SPEED);
        rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateMotor.setDirection(DcMotor.Direction.REVERSE);
        rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotateMotor.setPower(0);

        angleServo = hardwareMap.servo.get("angle_servo");
        angleServo.setDirection(Servo.Direction.FORWARD);
        angleServo.setPosition(Constants.ARM_ANGLE_FLAT);

        intakeServo = hardwareMap.servo.get("intake_servo");
        intakeServo.setDirection(Servo.Direction.FORWARD);
        intakeServo.setPosition(Constants.INTAKE_STOP_POWER);

        gateServo = hardwareMap.servo.get("gate_servo");
        gateServo.setDirection(Servo.Direction.FORWARD);
        gateServo.setPosition(0);
    }

    public void setRotateMotorPower(double power){
        rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateMotor.setPower(power);
    }

    public void setRotateMotorPosition(int position){
        rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotateMotor.setPower(0.5);
        rotateMotor.setTargetPosition(position);
    }

    public void setRotateMotorPostionP(int p){
        rotateMotor.setPositionPIDFCoefficients(p);
    }

    public int getRotateMotorPosition(){
        return rotateMotor.getCurrentPosition();
    }

    public void setAngleServoPosition(double position){
        angleServo.setPosition(position);
    }

    public void setIntakeServoPosition(double position){
        intakeServo.setPosition(position);
    }

    public void setGateServoPosition(double position){
        gateServo.setPosition(position);
    }

    public void stop(){
        rotateMotor.stop();
    }
}
