package org.firstinspires.ftc.teamcode.boogiewheel_base.hardware;

import org.firstinspires.ftc.teamcode.framework.AbstractOpMode;

import java.util.concurrent.Callable;

public class Robot {

    private HardwareDevices hardware;

    public Robot(){
        hardware = new HardwareDevices();
    }

    //Drive Methods
    public void setDriveY(double y){
        hardware.drive.setY(y);
    }

    public void setDriveZ(double z){
        hardware.drive.setZ(z);
    }

    public void setDrivePower(double l, double r){
        hardware.drive.setPower(l,r);
    }

    public void updateDrive(){
        hardware.drive.update();
    }

    public void turnTo(double angle, double speed, double error, int period){
        hardware.drive.turnTo(angle, speed, error, period);
    }

    public int[][] recordPath(int numSamples,int timeInterval) {
        return hardware.drive.recordPath(numSamples, timeInterval);
    }

    public void runPath(int[] left, int[] right, int timeInterval) {
        hardware.drive.runPath(left, right, timeInterval);
    }

    public void driveTo(double distance, double speed){
        hardware.drive.driveTo(distance, speed);
    }

    public void setPosition(int position, double power) {
        hardware.drive.setPosition(position, power);
    }

    public boolean isGyroCalibrated() {
        return hardware.drive.isGyroCalibrated();
    }

    public double getHeading(){
        return hardware.drive.getHeading();
    }

    public void stop(){
        hardware.stop();
    }

    //Intake Methods
    public void beginIntaking(){

    }

    public Callable finishIntaking(){
        return () -> {
          hardware.intake.beginIntaking();
          return true;
        };
    }

    public void delay(int time){
        AbstractOpMode.delay(time);
    }
}
