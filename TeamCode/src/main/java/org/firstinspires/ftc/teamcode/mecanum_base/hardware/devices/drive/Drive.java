package org.firstinspires.ftc.teamcode.mecanum_base.hardware.devices.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.framework.AbstractOpMode;
import org.firstinspires.ftc.teamcode.framework.userHardware.inputs.sensors.IMU;
import org.firstinspires.ftc.teamcode.framework.userHardware.outputs.SlewDcMotor;

import java.util.ArrayList;

public class Drive {

    private SlewDcMotor back_left, back_right, front_left, front_right;
    private ArrayList<SlewDcMotor> motors;
    private IMU imu;
    public Drive(HardwareMap hwMap){

        //Motors
        front_left = new SlewDcMotor(hwMap.dcMotor.get("front_left"));
        front_right = new SlewDcMotor(hwMap.dcMotor.get("front_right"));
        back_left = new SlewDcMotor(hwMap.dcMotor.get("back_left"));
        back_right = new SlewDcMotor(hwMap.dcMotor.get("back_right"));

        //Motor Set Up
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        this.motors = new ArrayList<>();
        motors.add(front_left);
        motors.add(front_right);
        motors.add(back_left);
        motors.add(back_right);

        double slewSpeed = 0.2;
        DcMotor.ZeroPowerBehavior zero = DcMotor.ZeroPowerBehavior.BRAKE;
        //DcMotor.RunMode encodingMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        DcMotor.RunMode encodingMode = DcMotor.RunMode.RUN_USING_ENCODER;

        for (SlewDcMotor motor: motors) {
            motor.setSlewSpeed(slewSpeed);
            motor.setZeroPowerBehavior(zero); // Can be float if required
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(encodingMode);
            motor.setPower(0);
        }
        imu = new IMU(hwMap);
    }//end of drive function

    public void setSlewSpeed(double ss){
        for (SlewDcMotor motor: this.motors) {
            motor.setSlewSpeed(ss);
        }
    }

    public void setMode(DcMotor.RunMode mode){
        front_left.setMode(mode);
        front_right.setMode(mode);
        back_left.setMode(mode);
        back_right.setMode(mode);
    }
    public double getHeading(){
        return imu.getHeading();
    }

    public void resetAngleToZero(){
        imu.resetAngleToZero();
    }

    public int getFrontLeftPosition() {
            return front_left.getCurrentPosition();
    }

    public int getFrontRightPosition() {
        return front_right.getCurrentPosition();
    }

    public int getBackLeftPosition() {
        return back_left.getCurrentPosition();
    }

    public int getBackRightPosition() {
        return back_right.getCurrentPosition();
    }

    public void setPower(double fl, double fr, double bl, double br){
        front_left.setPower(fl);
        front_right.setPower(fr);
        back_left.setPower(bl);
        back_right.setPower(br);
        /*Dashboard.getInstance().getTelemetry().write("FL "+front_left.getCurrentPosition());
        Dashboard.getInstance().getTelemetry().write("FR "+front_right.getCurrentPosition());
        Dashboard.getInstance().getTelemetry().write("BL "+back_left.getCurrentPosition());
        Dashboard.getInstance().getTelemetry().write("BR "+back_right.getCurrentPosition());*/
        AbstractOpMode.getOpModeInstance().telemetry.addData("FL ",front_left.getCurrentPosition());
        AbstractOpMode.getOpModeInstance().telemetry.addData("FR ",front_right.getCurrentPosition());
        AbstractOpMode.getOpModeInstance().telemetry.addData("BL ",back_left.getCurrentPosition());
        AbstractOpMode.getOpModeInstance().telemetry.addData("BR ",back_right.getCurrentPosition());
        AbstractOpMode.getOpModeInstance().telemetry.update();
    }

    public void stop(){
        //Stops Update Threads
        for(SlewDcMotor motor: this.motors) {
            motor.stop();
        }
    }
}