package org.firstinspires.ftc.teamcode.mecanum_base.hardware;

import org.firstinspires.ftc.teamcode.framework.AbstractRobot;
import org.firstinspires.ftc.teamcode.framework.userHardware.PIDController;

public class Robot extends AbstractRobot {

    private HardwareDevices hardware;
    private PIDController drivePID;

    public Robot(){
        hardware = new HardwareDevices();
        drivePID = new PIDController();
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