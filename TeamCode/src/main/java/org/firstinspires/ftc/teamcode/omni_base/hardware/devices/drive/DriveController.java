package org.firstinspires.ftc.teamcode.omni_base.hardware.devices.drive;

import org.firstinspires.ftc.teamcode.framework.SubsystemController;

import static java.lang.Math.pow;

public class DriveController extends SubsystemController{

    private Drive drive;

    private double turn_x=0, turn_y=0, turn_z=0, fl=0, fr=0, bl=0, br=0, Drive_Power = 1;

    public DriveController(){
        init();
    }

    public void init(){
        opModeSetup();

        //Put general setup here
        drive = new Drive(hwMap);

        if(Mode==mode.Auton){
            //Put Auton setup here
            drive.setSlewSpeed(1);

        }

        else if(Mode==mode.Teleop){
            //Put Teleop setup here
            drive.setSlewSpeed(0.1);

        }
    }

    public void setX(double x){
        turn_x = x;
        turn_x = (float) scaleInput(turn_x);
    }

    public void setY(double y){
        turn_y = y;
        turn_y = (float) scaleInput(turn_y);
    }

    public void setZ(double z){
        turn_z = z;
        turn_z = (float) scaleInput(turn_z);
    }

    public void update(){
        fl = range((turn_y + turn_x + turn_z) * Drive_Power);
        fr = range((turn_y - turn_x - turn_z) * Drive_Power);
        bl = range((turn_y - turn_x + turn_z) * Drive_Power);
        br = range((turn_y + turn_x - turn_z) * Drive_Power);
        telemetry.addData("fl",fl);
        telemetry.addData("fr",fr);
        telemetry.addData("bl",bl);
        telemetry.addData("br",br);
        drive.setPower(fl, fr, bl, br);
    }

    public void setPower(double fl, double fr, double bl, double br){
        drive.setPower(range(fl),range(fr),range(bl),range(br));
    }

    public void stop(){
        drive.stop();
    }

    private double scaleInput(double val) {
        return range(pow(val, 3));
    }

    private double range(double val){
        if(val<-1) val=-1;
        if(val>1) val=1;
        return val;
    }
}