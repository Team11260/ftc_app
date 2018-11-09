package org.firstinspires.ftc.teamcode.skidsteer_base.hardware;

import org.firstinspires.ftc.teamcode.skidsteer_base.hardware.devices.drive.DriveController;

public class HardwareDevices {

    private DriveController drive;

    private MyRev2mDistanceSensor LeftDistanceSensor;

    private MyRev2mDistanceSensor RightDistanceSensor;

    public HardwareDevices(){
        drive = new DriveController();
        LeftDistanceSensor = new MyRev2mDistanceSensor("distance1");
        RightDistanceSensor = new MyRev2mDistanceSensor("distance2");
    }

    public void setDriveY(double y) {
        drive.setY(y);
    }

    public void setDriveZ(double z) {
        drive.setZ(z);
    }

    public void turnBackToZeroDeg() {
        drive.turnTo(0,.5,1,500);
    }

    public void turn90DegLeft() {
        drive.turnTo(90, .5, 1,500);
    }

    public void turn90DegRight() {
        drive.turnTo(-90, .5, 1,500);
    }

    public void driveForward() {
        drive.driveToMyDistance(3000, .5);
    }

    public void measureLeftEncoder(){
        drive.measureEncoders(0,5000);
    }

    public void measureRightEncoder(){
        drive.measureEncoders(1,5000);
    }

    public void turnToVer2(double angle, double speed, int error)
    {
        drive.turnToVer2(angle, speed, error);
    }

    public void driveToMyDistanceGyro(double speed, double distance, double target_angle, double p_multiplier)
    {
        drive.driveToMyDistanceGyro(speed, distance, target_angle, p_multiplier);
    }

    public void driveBack() {
        drive.driveToMyDistance(-3000, .5);
    }

    public void updateDrive(){
        drive.update();
    }

    public void drive(double distance){
        drive.driveDistance(distance);
    }

    public void driveToMyDistance(double distance,double speed) {
        drive.driveToMyDistance(distance,speed);
    }

    public void turnTo(double angle, double speed, int error, int period){
        drive.turnTo(angle, speed, error, period);
    }

    public void centerBack()
    {
        if(RightDistanceSensor.getDistanceIN() >= LeftDistanceSensor.getDistanceIN())
        {
            //  start turning left
            setDrivePower(-.05,.05);
            // turning is in porgress, moniitor when it is done
            while (RightDistanceSensor.getDistanceIN() >= LeftDistanceSensor.getDistanceIN())
            {
                // start to turn left
            }
            // rightdiscance <= leftdistance, so stop as fast as possible
            setDrivePower(0,0);
        }
        if(LeftDistanceSensor.getDistanceIN() >= RightDistanceSensor.getDistanceIN()) {

            //start to turn right
            setDrivePower(0.05,-0.05);
            // turning is in porgress, moniitor when it is done
            while (LeftDistanceSensor.getDistanceIN() >= RightDistanceSensor.getDistanceIN())
            {
                //wait and do nothing
            }
            // rightdiscance >= leftdistance, so stop as fast as possible
            setDrivePower(0,0);
        }
    }


    public void setDrivePosition(int position, double power){
        drive.setPosition(position,power);
    }

    public void setDrivePower(double l, double r){
        drive.setPower(l,r);
    }

    public double getDriveHeading(){
        return drive.getHeading();
    }

    public void resetAngleToZero() {
        drive.resetAngleToZero();
    }

    public boolean isDriveModeDone(){
        return drive.isModeDone();
    }

    public void stop(){
        drive.stop();
    }
}