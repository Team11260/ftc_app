package org.firstinspires.ftc.teamcode.skidsteerbase.hardware;

import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.ExpansionHubMonitor;
import org.firstinspires.ftc.teamcode.framework.util.AbstractRobot;

import java.util.concurrent.Callable;

public class Robot extends AbstractRobot {

    private HardwareDevices hardware;
    private ExpansionHubMonitor hub;

    //Robot Methods
    public Robot() {
        hardware = new HardwareDevices();

        hub = new ExpansionHubMonitor("Expansion Hub 1");
    }

    @Override
    public void stop() {

    }

    public void updateAll(){
        hardware.drive.update();
        hardware.arm.update();
    }

    public void moveToDumpPosition(){
        hardware.arm.moveToDumpPosition();
    }

    public Callable moveToDumpPositionCallable(){
        return ()->{
            moveToDumpPosition();
            return true;
        };
    }

    public void moveToCollectPosition(){
        hardware.arm.moveToCollectPosition();
    }

    public Callable moveToCollectPositionCallable(){
        return ()->{
            moveToCollectPosition();
            return true;
        };
    }

    public void setDrivePower(double left, double right){
        hardware.drive.setPower(left,right);
    }

    public void beginIntaking(){
        hardware.arm.beginIntaking();
    }

    public Callable beginIntakingCallable(){
        return ()->{
            beginIntaking();
            return true;
        };
    }

    public void reverseIntake(){
        hardware.arm.reverseIntake();
    }

    public Callable reverseIntakeCallable(){
        return ()->{
            reverseIntake();
            return true;
        };
    }

    public void finishIntaking(){
        hardware.arm.finishIntaking();
    }

    public Callable finishIntakingCallable(){
        return ()->{
            finishIntaking();
            return true;
        };
    }

    public Callable openMineralGateCallable() {
        return () -> {
            openMineralGate();
            return true;
        };
    }

    public void openMineralGate() {
        hardware.arm.openGate();
    }

    public Callable closeMineralGateCallable() {
        return () -> {
            closeMineralGate();
            return true;
        };
    }

    public void closeMineralGate() {
        hardware.arm.closeGate();
    }

    public Callable toggleMineralGateCallable() {
        return () -> {
            hardware.arm.toggleGate();
            return true;
        };
    }
}