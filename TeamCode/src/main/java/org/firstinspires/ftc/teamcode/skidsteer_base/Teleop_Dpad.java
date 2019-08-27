package org.firstinspires.ftc.teamcode.skidsteer_base;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.framework.AbstractTeleop;
import org.firstinspires.ftc.teamcode.framework.userhardware.MyNumberRound;
import org.firstinspires.ftc.teamcode.skidsteer_base.hardware.HardwareDevices;

@TeleOp(name="SkidSteer_Dpad_Encoders", group="New")
//@Disabled

public class Teleop_Dpad extends AbstractTeleop {

    private org.firstinspires.ftc.teamcode.mecanum_base.hardware.HardwareDevices robot;

    private HardwareDevices hardware;

    MyNumberRound myNumberRound;

    final double ENCODER_PER_INCH = 105;

    @Override
    public void Init()
    {
        myNumberRound = new MyNumberRound();
        hardware = new HardwareDevices();
    }

    @Override
    public void Loop() {

       // hardware.updateDrive();
        telemetry.addData("dpad -90 0 +90");
        telemetry.addData("a_down angle=0");
        telemetry.addData("x = measure  left motor 5 sec");
        telemetry.addData("y = measure right motor 5 sec");
        telemetry.addData("lb = forward  3000");
        telemetry.addData("rb = backward 3000");
        telemetry.addData("ANGLE = " + myNumberRound.roundDouble( hardware.getDriveHeading(),2));
        telemetry.update();
    }

    @Override
    public void Stop() {
        hardware.stop();

    }

    @Override
    public void lsy_change(double lsy) {
        hardware.setDriveY(lsy);
    }

    @Override
    public void rsx_change(double rsx) {
        hardware.setDriveZ(rsx);
    }



    @Override
    public void a_down() {//path 1
        //3000 ticks is 28.5 inches
        //105 ticks is 1 inch
        //lower level of gyro needs to be protected otherwise the robot will go around in a circle
        hardware.resetAngleToZero();
        hardware.driveToMyDistanceGyro(1,(49*ENCODER_PER_INCH),0,1);
        hardware.turnToVer2(-135,1,1);
        hardware.driveToMyDistanceGyro(1,(ENCODER_PER_INCH*78),-135,1);
    }


    @Override
    public void dpu_down()
    {
        hardware.turnToVer2(0,1,1);
    }
    @Override
    public void dpl_down()
    {
        hardware.driveToMyDistanceGyro(.5,(ENCODER_PER_INCH*-30),0,1);

    }
    @Override
    public void dpr_down()
    {//path 3 right mineral
        hardware.resetAngleToZero();
        hardware.turnToVer2(-20,.5,1);
        hardware.driveToMyDistanceGyro(.5,(50*ENCODER_PER_INCH),-20,1);
        hardware.turnToVer2(45,.5,1);
        hardware.driveToMyDistanceGyro(.5,(25*ENCODER_PER_INCH),45,1);
        hardware.turnToVer2(-135,1,1);
        hardware.driveToMyDistanceGyro(.5,(78*ENCODER_PER_INCH),-135,1);
    }

    @Override
    public void dpd_down()
    {//path 2 left mineral
        hardware.resetAngleToZero();
        hardware.turnToVer2(20,.5,1);
        hardware.driveToMyDistanceGyro(1,(ENCODER_PER_INCH*36),20,1);
        hardware.turnToVer2(-45,.5,1);
        hardware.driveToMyDistanceGyro(1,(ENCODER_PER_INCH*15),-45,1);
        hardware.turnToVer2(-135,1,1);
        hardware.driveToMyDistanceGyro(1,(ENCODER_PER_INCH*80),-135,1);
    }

    @Override
    public void lb_down() {
        hardware.driveForward();
    }


    @Override
    public void rb_down() {
        hardware.driveBack();
    }

    @Override
    public void x_down()
    {//path 4 left mineral
        //hardware.measureLeftEncoder();
        hardware.resetAngleToZero();
        hardware.turnToVer2(20,1,1);
        hardware.driveToMyDistanceGyro(1,(36*ENCODER_PER_INCH),20,1);
        hardware.turnToVer2(-45,1,1);
        hardware.driveToMyDistanceGyro(1,(10*ENCODER_PER_INCH),-45,1);
        hardware.driveToMyDistanceGyro(1,(ENCODER_PER_INCH*-78),-45,1);
    }

    @Override
    public void y_down()
    {//path 5 middle mineral
        //hardware.measureRightEncoder();
        hardware.resetAngleToZero();
        hardware.driveToMyDistanceGyro(1,(49*ENCODER_PER_INCH),0,1);
        hardware.turnToVer2(135,1,1);
        hardware.driveToMyDistanceGyro(1,(ENCODER_PER_INCH*78),135,1);


    }



    @Override
    public void b_down()
    {
        //hardware.resetAngleToZero();
        hardware.resetAngleToZero();
        hardware.turnToVer2(-20,.5,1);
        hardware.driveToMyDistanceGyro(.5,(50*ENCODER_PER_INCH),-20,1);
        hardware.turnToVer2(45,.5,1);
        hardware.driveToMyDistanceGyro(.5,(25*ENCODER_PER_INCH),45,1);
        hardware.turnToVer2(135,1,1);
        hardware.driveToMyDistanceGyro(.5,(78*ENCODER_PER_INCH),135,1);
    }

}