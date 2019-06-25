package org.firstinspires.ftc.teamcode.mecanum_base.hardware;

import org.firstinspires.ftc.teamcode.framework.AbstractRobot;
import org.firstinspires.ftc.teamcode.framework.userHardware.PIDController;
import org.firstinspires.ftc.teamcode.mecanum_base.hardware.devices.drive.DriveController;
import org.firstinspires.ftc.teamcode.skidsteer_base.hardware.MyRev2mDistanceSensor;

public class Robot extends AbstractRobot {
    private DriveController drive;
    private HardwareDevices hardware;
    private PIDController drivePID;
    public Robot(){
        hardware = new HardwareDevices();
        drivePID = new PIDController();
        drive = new DriveController();
    }

    public double getFrontLeftPosition() {
        return drive.getFrontLeftPosition();
    }

    public double getFrontRightPosition() {
        return drive.getFrontRightPosition();
    }

    public double getBackLeftPosition() {
        return drive.getBackLeftPosition();
    }

    public double getBackRightPosition() {
        return drive.getBackRightPosition();
    }

    public void resetAllEncoders() {
        drive.resetAllEncoders();

    }

    public void setDriveX(double x) { hardware.setDriveX(x);
    }

    public void setDriveY(double y) {
        hardware.setDriveY(y);
    }

    public void setDriveZ(double z) {
        hardware.setDriveZ(z);
    }

    public void driveUpdate(){
        hardware.updateDrive();
    }

    public void turnTo(int angle, double speed, int error){
    }

    public void measureEncoders(int MotorSelected,double milliSecondsStop ) {
        hardware.measureEncoders(MotorSelected, milliSecondsStop);
    }

    public void strafeToMyDistance(double distance, double speed)
    {
        hardware.strafeToMyDistance(distance,speed);
    }
    public double getHeading(){
        return hardware.getHeading();
    }
    public void stop(){
        hardware.stop();
    }
}