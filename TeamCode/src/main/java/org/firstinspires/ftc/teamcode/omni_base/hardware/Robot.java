package org.firstinspires.ftc.teamcode.omni_base.hardware;

import org.firstinspires.ftc.teamcode.framework.AbstractRobot;
import org.firstinspires.ftc.teamcode.framework.userhardware.PIDController;

public class Robot extends AbstractRobot {

    private HardwareDevices hardware;
    private PIDController drivePID;

    public Robot(){
        hardware = new HardwareDevices();
        drivePID = new PIDController();
    }

    public void speak(String text){
        hardware.speak(text);
    }

    public boolean isSpeaking(){
        return hardware.isSpeaking();
    }

    public void glyphClawOpen() {
        hardware.glyphClawOpen();
    }

    public void glyphClawClose() {
        hardware.glyphClawClose();
    }

    public void glyphLiftUp(){
        hardware.glyphLiftUp();
    }

    public void glyphLiftDown(){
        hardware.glyphLiftDown();
    }

    public void relicFullUp(){
        hardware.relicFullUp();
    }

    public void relicSmallUp(){
        hardware.relicSmallUp();
    }

    public void relicSmallDown(){
        hardware.relicSmallDown();
    }

    public void relicFullExtend(){
        hardware.relicFullExtend();
    }

    public void relicFullRetract(){
        hardware.relicFullRetract();
    }

    public void relicSmallExtend(){
        hardware.relicSmallExtend();
    }

    public void relicSmallRetract(){
        hardware.relicSmallRetract();
    }

    public void jewelArmUp(){
        hardware.jewelArmUp();
    }

    public void jewelArmDown(){
        hardware.jewelArmDown();
    }

    public void setDriveX(double x) {
        hardware.setDriveX(x);
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

    public void stop(){
        hardware.stop();
    }
}