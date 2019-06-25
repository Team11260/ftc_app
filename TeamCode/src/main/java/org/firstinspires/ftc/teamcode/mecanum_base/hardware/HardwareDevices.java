package org.firstinspires.ftc.teamcode.mecanum_base.hardware;

import org.firstinspires.ftc.teamcode.mecanum_base.hardware.devices.drive.DriveController;

public class HardwareDevices {

    private DriveController drive;

    public HardwareDevices(){
        drive = new DriveController();
    }

    public void setDriveX(double x) {
        drive.setX(x);
    }

    public void pressed_A_Down() {
        drive.pressed_A_Down();
    }
    public void pressed_A_Up() {
        drive.pressed_A_Up();
    }

    public void pressed_B_Down() {
        drive.pressed_B_Down();
    }
    public void pressed_B_Up() {
        drive.pressed_B_Up();
    }

    public void pressed_X_Down() {
        drive.pressed_X_Down();
    }
    public void pressed_X_Up() {
        drive.pressed_X_Up();
    }

    public void pressed_Y_Down() {
        drive.pressed_Y_Down();
    }
    public void pressed_Y_Up() {
        drive.pressed_Y_Up();
    }

    public void measureEncoders(int MotorSelected,double milliSecondsStop ) {
        drive.measureEncoders(MotorSelected, milliSecondsStop);
    }

    public void setDriveY(double y) {
        drive.setY(y);
    }

    public void setDriveZ(double z) {
        drive.setZ(z);
    }

    public void strafeToMyDistance(double distance, double speed) {
        drive.strafeToMyDistance(distance, speed);
    }

    public void strafeToMyDistanceGyro(double speed, double distance, double target_angle, double p_multiplier, int direction) {
        drive.strafeToMyDistanceGyro( speed,  distance, target_angle, p_multiplier, direction);
    }

    public void updateDrive(){
        drive.update();
    }

    public void driveMotors(double speed) {
        drive.setPower(speed, speed, speed, speed);
    }

    public void updateCheckMotors() {
        drive.updateCheckMotors();
    }

    public void setPower(double fl, double fr, double bl, double br){
        drive.setPower(fl,fr,bl,br);
    }
    public double getHeading(){
        return drive.getHeading();
    }

    public void resetAngleToZero() {
        drive.resetAngleToZero();
    }

    public void stop(){
        drive.stop();
    }
}
