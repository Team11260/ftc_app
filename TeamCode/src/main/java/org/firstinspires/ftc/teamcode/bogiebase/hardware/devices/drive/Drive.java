package org.firstinspires.ftc.teamcode.bogiebase.hardware.devices.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.Constants;
import org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState;
import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.IMU;
import org.firstinspires.ftc.teamcode.framework.userhardware.outputs.SlewDcMotor;
import org.firstinspires.ftc.teamcode.framework.userhardware.purepursuit.PurePursuitController;

import static org.firstinspires.ftc.teamcode.bogiebase.hardware.devices.drive.DriveController.PATH_F;
import static org.firstinspires.ftc.teamcode.bogiebase.hardware.devices.drive.DriveController.PATH_P;

public class Drive extends PurePursuitController {

    private SlewDcMotor leftMotor, rightMotor;
    private DcMotorSimple light;
    private Servo servoMarker;

    private IMU imu;

    public Drive(HardwareMap hardwareMap) {
        super(Constants.TRACK_WIDTH);

        imu = new IMU(hardwareMap);

        //Motors
        leftMotor = new SlewDcMotor(hardwareMap.dcMotor.get("left"));
        rightMotor = new SlewDcMotor(hardwareMap.dcMotor.get("right"));

        //Motor Set Up
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setSlewSpeed(Constants.DRIVE_SLEW_SPEED);
        rightMotor.setSlewSpeed(Constants.DRIVE_SLEW_SPEED);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Encoder Set Up
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        light = hardwareMap.get(DcMotorSimple.class, "light");
        light.setPower(0);

        servoMarker = hardwareMap.servo.get("servo_marker");
        servoMarker.setPosition(RobotState.currentMatchState == RobotState.MatchState.AUTONOMOUS ? Constants.DRIVE_TEAM_MARKER_RETRACTED : Constants.DRIVE_TEAM_MARKER_TELEOP_RETRACTED);
    }

    public void setSlewSpeed(double ss) {
        leftMotor.setSlewSpeed(ss);
        rightMotor.setSlewSpeed(ss);
    }

    public void setPower(double l, double r) {
        leftMotor.setPower(l);
        rightMotor.setPower(r);
    }

    public void resetAngleToZero() {
        imu.resetAngleToZero();

    }

    public void setTargetPosition(int position) {
        leftMotor.setTargetPosition(position);
        rightMotor.setTargetPosition(position);
    }

    public void setTargetPosition(int leftPosition, int rightPosition) {
        leftMotor.setTargetPosition(leftPosition);
        rightMotor.setTargetPosition(rightPosition);
    }

    public void setMode(DcMotor.RunMode mode) {
        leftMotor.setMode(mode);
        rightMotor.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftMotor.setZeroPowerBehavior(behavior);
        rightMotor.setZeroPowerBehavior(behavior);
    }

    public PIDFCoefficients getSpeedPIDF() {
        return leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setSpeedPIDF(PIDFCoefficients c) {
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
    }

    public void setPositionP(double p) {
        //leftMotor.setPositionPIDFCoefficients(p);
        //rightMotor.setPositionPIDFCoefficients(p);
    }

    public int getLeftPosition() {
        return leftMotor.getCurrentPosition();
    }

    public int getRightPosition() {
        return rightMotor.getCurrentPosition();
    }

    @Override
    public double getActualHeadingDegrees() {
        return getHeading();
    }

    @Override
    public double getLeftActualPositionInches() {
        return getRightPosition() / Constants.DRIVE_COUNTS_PER_INCH;
    }

    @Override
    public double getRightActualPositionInches() {
        return getLeftPosition() / Constants.DRIVE_COUNTS_PER_INCH;
    }

    @Override
    public void setPowers(double l, double r) {
        setSpeedPIDF(new PIDFCoefficients(PATH_P, 0, 0, PATH_F));
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setPower(l, r);
    }

    public double getLeftPower() {
        return leftMotor.getPower();
    }

    public double getRightPower() {
        return rightMotor.getPower();
    }

    public void setMarkerServo(double servoPosition) {
        servoMarker.setPosition(servoPosition);
    }

    public boolean isBusy() {
        return leftMotor.isBusy() || rightMotor.isBusy();
    }

    public double getLeftMotorCurrentDraw() {
        return leftMotor.getCurrentDraw();
    }

    public double getRightMotorCurrentDraw() {
        return rightMotor.getCurrentDraw();
    }

    public double getHeading() {
        if (imu == null) return 0.0;
        return imu.getHeading();
    }

    public double getAbsoluteHeading() {
        return imu.getAbsoluteHeading();
    }

    public double getPitch(){
        return imu.getPitch();
    }

    public boolean isGyroCalibrated() {
        if (imu == null) return false;
        return imu.isGyroCalibrated();
    }

    public void setLightPower(double power) {
        light.setPower(Math.abs(power));
    }

    public void stop() {
        //Stops Update Threads
        leftMotor.stop();
        rightMotor.stop();
    }

}