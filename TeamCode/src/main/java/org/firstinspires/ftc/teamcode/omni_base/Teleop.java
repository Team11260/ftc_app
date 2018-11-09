package org.firstinspires.ftc.teamcode.omni_base;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.framework.AbstractTeleop;
import org.firstinspires.ftc.teamcode.omni_base.hardware.Robot;

@TeleOp(name="Omni_TeleOp_New", group="New")
//@Disabled

public class Teleop extends AbstractTeleop {

    private Robot robot;

    boolean gamepadMode = false;

    @Override
    public void Init() {
        robot = new Robot();
    }

    @Override
    public void Loop() {
        robot.driveUpdate();
        telemetry.update();
    }

    @Override
    public void Stop() {
        robot.stop();
    }

    @Override
    public  void a_down(){
        if(!gamepadMode){
            robot.glyphClawOpen();
        }
        else {

        }
    }

    @Override
    public void b_down(){
        if(!gamepadMode){
            robot.glyphClawClose();
        }
        else {

        }
    }

    @Override
    public  void x_down(){
        if(!gamepadMode){
        }
        else {
            robot.relicFullRetract();
        }
    }

    @Override
    public void y_down(){
        if(!gamepadMode){
        }
        else {
            robot.relicFullExtend();
        }
    }

    @Override
    public void lb_down(){
        if(!gamepadMode){
            robot.glyphLiftDown();
        }
        else {

        }
    }

    @Override
    public void rb_down(){
        if(!gamepadMode){
            robot.glyphLiftUp();
        }
        else {
            robot.relicFullUp();
        }
    }

    @Override
    public void dpu_down(){
        if(!gamepadMode){
        }
        else {
            robot.relicSmallUp();
        }
    }

    @Override
    public void dpd_down(){
        if(!gamepadMode){
        }
        else {
            robot.relicSmallDown();
        }
    }

    @Override
    public void lt_change(double lt){
        if(!gamepadMode){
        }
        else {
            if(lt>0.5){
                robot.relicSmallRetract();
            }
        }
    }

    @Override
    public void rt_change(double rt){
        if(!gamepadMode){
        }
        else {
            if(rt>0.5){
                robot.relicSmallExtend();
            }
        }
    }

    @Override
    public void lsx_change(double lsx) {
        robot.setDriveX(lsx);
    }

    @Override
    public void lsy_change(double lsy) {
        robot.setDriveY(lsy);
    }

    @Override
    public void rsx_change(double rsx) {
        robot.setDriveZ(rsx);
    }

    @Override
    public void back_down(){
        gamepadMode = !gamepadMode;
        telemetry.addData("Gamepad Mode: "+gamepadMode);
    }
}