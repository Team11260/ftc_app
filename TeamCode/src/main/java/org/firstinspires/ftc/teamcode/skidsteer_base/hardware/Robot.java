package org.firstinspires.ftc.teamcode.skidsteer_base.hardware;

import org.firstinspires.ftc.teamcode.framework.AbstractAuton;
import org.firstinspires.ftc.teamcode.framework.AbstractRobot;
import org.firstinspires.ftc.teamcode.framework.userHardware.PIDController;
import org.upacreekrobotics.dashboard.Dashboard;

import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.sqrt;

public class Robot extends AbstractRobot {

    private HardwareDevices hardware;
    private PIDController drivePID;
    Dashboard.dashboardtelemetry telem;


    public Robot(){
        hardware = new HardwareDevices();
        drivePID = new PIDController();
        Dashboard dashboard = Dashboard.getInstance();
        telem = dashboard.getTelemetry();
    }

    public void setDriveY(double y) {
        hardware.setDriveY(y);
    }

    public void setDriveZ(double z) {
        hardware.setDriveZ(z);
    }

    public void updateDrive(){
        hardware.updateDrive();
    }

    public void centerBack()
    {
        hardware.centerBack();
    }

    public void setLeftDrive(boolean on) {
        //hardware.setLeftDrive(on);
    }

    public void setRightDrive(boolean on) {
        //hardware.setRightDrive(on);
    }

    public void drive(double distance){
        hardware.drive(distance);
    }

    public void turnTo(double angle, double speed, int error,int period){
        hardware.turnTo(angle,speed,error,period);
    }

    public void moveTo(double x, double y, double finalAngle){
        double angle;
        int distance;
        distance = (int) sqrt((x*x)+(y*y));
        angle = ((atan(abs(y/x))*57.3)); //57.3 = 1 radian
        if ((x<0)&& (y<0))angle=angle-180;
        if ((x<0)&& (y>0))angle=180-angle;
        if ((x>0)&& (y<0))angle=-angle;
        telem.write("Angle: "+angle);
        telem.write("Distance: "+distance);
        AbstractAuton.getOpModeInstance().telemetry.addData("Angle",angle);
        AbstractAuton.getOpModeInstance().telemetry.update();
        delay(1000);
        hardware.turnTo(angle,0,1,500);
        telem.write("First angle: "+hardware.getDriveHeading());
        delay(1000);
        hardware.drive(distance);
        delay(1000);
        hardware.turnTo(finalAngle,0,1,500);
        delay(1000);
        telem.write("Final angle: "+hardware.getDriveHeading());
    }

    public void driveToMyDistance(double distance,double speed)
    {
        hardware.driveToMyDistance(distance,speed);
    }

    public void driveToMyDistanceGyro(double speed, double distance, double target_angle, double p_multiplier)
    {
        hardware.driveToMyDistanceGyro(speed, distance, target_angle, p_multiplier);
    }

    public void turnToVer2(double angle, double speed, int error)
    {
        hardware.turnToVer2(angle, speed, error);
    }


    public void stop(){
        hardware.stop();
    }
}