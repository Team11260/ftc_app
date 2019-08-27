package org.firstinspires.ftc.teamcode.mecanum_base.hardware;

import org.firstinspires.ftc.teamcode.framework.AbstractRobot;
import org.firstinspires.ftc.teamcode.framework.userhardware.PIDController;
import org.firstinspires.ftc.teamcode.framework.userhardware.purepursuit.Path;
import org.firstinspires.ftc.teamcode.mecanum_base.hardware.devices.drive.DriveController;
import org.firstinspires.ftc.teamcode.skidsteer_base.hardware.MyRev2mDistanceSensor;

public class Robot extends AbstractRobot {
    private HardwareDevices hdMecunum;

    public Robot(){
        hdMecunum = new HardwareDevices();
    }

    public double getFrontLeftPosition() {
        return hdMecunum.getFrontLeftPosition();
    }


    public double getFrontRightPosition() {
        return hdMecunum.getFrontRightPosition();
    }

    public double getBackLeftPosition() {
        return hdMecunum.getBackLeftPosition();
    }

    public double getBackRightPosition() {
        return hdMecunum.getBackRightPosition();
    }

    public void resetAllEncoders() {
        hdMecunum.resetAllEncoders();

    }

    public void setDriveX(double x) { hdMecunum.setDriveX(x);
    }

    public void setDriveY(double y) {
        hdMecunum.setDriveY(y);
    }

    public void setDriveZ(double z) {
        hdMecunum.setDriveZ(z);
    }

    public void driveUpdate(){
        hdMecunum.updateDrive();
    }

    public void turnTo(int angle, double speed, int error){
    }

    public void measureEncoders(int MotorSelected,double milliSecondsStop ) {
        hdMecunum.measureEncoders(MotorSelected, milliSecondsStop);
    }

    public void strafeToMyDistance(double distance, double speed)
    {
        hdMecunum.strafeToMyDistance(distance,speed);
    }

    public void encoderStraight( double speed, double distance, double multiplier )
    {
        hdMecunum.encoderStraight(speed,distance, multiplier);
    }

    public void gyroStraight( double speed, double distance)
    {
        hdMecunum.gyroStraight( speed, distance);
    }


    public double getHeading(){
        return hdMecunum.getHeading();
    }

    public void setAutonPower(double left, double right)
    {
        hdMecunum.setAutonPower(left,right);
    }

    public void setPower(double fl, double fr, double bl, double br)
    {
        hdMecunum.setPower(fl,fr,bl,br);
    }

    public void stop(){
        hdMecunum.stop();
    }
}