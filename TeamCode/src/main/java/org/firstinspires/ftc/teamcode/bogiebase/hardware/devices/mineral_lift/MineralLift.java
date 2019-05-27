package org.firstinspires.ftc.teamcode.bogiebase.hardware.devices.mineral_lift;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants;
import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.DistanceSensor2m;
import org.firstinspires.ftc.teamcode.framework.userhardware.outputs.SlewDcMotor;

public class MineralLift {

    private SlewDcMotor liftMotor;

    private Servo gateServo, angleServo;

    private DistanceSensor2m distanceSensor;

    private AnalogInput bottomLimitSwitch;

    public MineralLift(HardwareMap hardwareMap) {
        liftMotor = new SlewDcMotor(hardwareMap.dcMotor.get("mineral_lift"));
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(0);
        liftMotor.setPower(0);

        liftMotor.setSlewSpeed(2);

        gateServo = hardwareMap.servo.get("mineral_gate");
        gateServo.setDirection(Servo.Direction.FORWARD);
        gateServo.setPosition(Constants.MINERAL_LIFT_GATE_CLOSED_POSITION);

        angleServo = hardwareMap.servo.get("sorter_angle");
        angleServo.setDirection(Servo.Direction.REVERSE);
        //angleServo.setPosition(RobotState.currentMatchState == RobotState.MatchState.AUTONOMOUS ? Constants.MINERAL_LIFT_ANGLE_SERVO_VERTICAL_POSITION : Constants.MINERAL_LIFT_ANGLE_SERVO_HORIZONTAL_POSITION);

        distanceSensor = new DistanceSensor2m("Distance1");

        bottomLimitSwitch = hardwareMap.analogInput.get("lift_bottom_limit");
    }

    public double getDistance() {
        return distanceSensor.getDistanceIN();
    }

    public void setTargetPosition(int position) {
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(Constants.MINERAL_LIFT_FULL_SPEED);
        liftMotor.setTargetPosition(position);
    }

    public void setCurrentPosition(int position) {
        liftMotor.setCurrentPosition(position);
    }

    public int getCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }

    public double getPower() {
        return liftMotor.getPower();
    }

    public boolean isLiftInProgress() {
        return liftMotor.isBusy();
    }

    public void setLiftMotorPower(double power) {
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setPower(power);
    }

    public double getMotorCurrentDraw(){ return liftMotor.getCurrentDraw();}

    public void setLiftMotorPowerNoEncoder(double power) {
        liftMotor.setPower(power);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetPosition() {
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setGateServoPosition(double position) {
        gateServo.setPosition(position);
    }

    public double getGateServoPosition() {
        return gateServo.getPosition();
    }

    public void setAngleServoPosition(double position) {
        angleServo.setPosition(position);
    }

    public boolean getBottomLimitSwitchPressed() {
        return bottomLimitSwitch.getVoltage() > 1;
    }

    public void stop() {
        liftMotor.stop();
    }
}
