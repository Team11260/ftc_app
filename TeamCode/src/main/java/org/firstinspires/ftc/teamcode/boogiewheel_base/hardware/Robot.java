package org.firstinspires.ftc.teamcode.boogiewheel_base.hardware;

import org.firstinspires.ftc.teamcode.framework.util.AbstractRobot;

import java.util.concurrent.Callable;

public class Robot extends AbstractRobot {

    private HardwareDevices hardware;
    private boolean mineralGateOpen = false, driveInverted = false;

    public Robot() {
        hardware = new HardwareDevices();
    }

    //Drive Methods
    public void setDriveY(double y) {
        if(driveInverted) hardware.drive.setY(-y);
        else hardware.drive.setY(y);
    }

    public void setDriveZ(double z) {
        hardware.drive.setZ(z);
    }

    public void setDrivePower(double l, double r) {
        hardware.drive.setPower(l, r);
    }

    public void updateDrive() {
        hardware.drive.update();
    }

    public Callable toggleDriveInvertedCallable(){
        return ()-> {
            driveInverted = !driveInverted;
            hardware.drive.setInverted(driveInverted);
            return true;
        };
    }

    public void turnTo(double angle, double speed, double error, int period) {
        hardware.drive.turnTo(angle, speed, error, period);
    }

    public int[][] recordPath(int numSamples, int timeInterval) {
        return hardware.drive.recordPath(numSamples, timeInterval);
    }

    public void runPath(int[] left, int[] right, int timeInterval) {
        hardware.drive.runPath(left, right, timeInterval);
    }

    public void driveTo(double distance, double speed) {
        hardware.drive.driveTo(distance, speed);
    }

    public void driveTo(double distance, double speed, int angle) {
        hardware.drive.driveTo(distance, speed, angle);
    }

    public int[] recordPathWithHeading(int numSamples, int timeInterval) {
        return hardware.drive.recordPathWithHeading(numSamples, timeInterval);
    }

    public void runPathWithHeading(int[] values, int timeInterval, double speed) {
        hardware.drive.runPathWithHeading(values, timeInterval, speed);
    }

    public void setPosition(int position, double power) {
        hardware.drive.setPosition(position, power);
    }

    public boolean isGyroCalibrated() {
        return hardware.drive.isGyroCalibrated();
    }

    public double getHeading() {
        return hardware.drive.getHeading();
    }

    public void stop() {
        hardware.stop();
    }

    //Intake Methods
    public Callable beginIntakingCallable() {
        return () -> {
            hardware.intake.beginIntaking();
            return true;
        };
    }

    public Callable finishIntakingCallable() {
        return () -> {
            hardware.intake.finishIntaking();
            driveInverted = true;
            hardware.drive.setInverted(driveInverted);
            return true;
        };
    }

    public Callable reverseIntakeCallable() {
        return () -> {
            hardware.intake.reverseIntake();
            return true;
        };
    }

    //Mineral Lift Methods
    public Callable moveMineralLiftToCollectPositionCallable() {
        return () -> {
            moveMineralLiftToCollectPosition();
            return true;
        };
    }

    public void moveMineralLiftToCollectPosition() {
        hardware.mineralLift.moveToCollectPosition();
        hardware.mineralLift.closeGate();
        driveInverted = false;
        hardware.drive.setInverted(driveInverted);
    }

    public Callable moveMineralLiftToDumpPositionCallable() {
        return () -> {
            moveMineralLiftToDumpPosition();
            return true;
        };
    }

    public void moveMineralLiftToDumpPosition() {
        hardware.mineralLift.moveToDumpPosition();
    }

    public Callable openMineralGateCallable() {
        return () -> {
            openMineralGate();
            return true;
        };
    }

    public void openMineralGate() {
        hardware.mineralLift.openGate();
    }

    public Callable closeMineralGateCallable() {
        return () -> {
            closeMineralGate();
            return true;
        };
    }

    public void closeMineralGate() {
        hardware.mineralLift.closeGate();
    }

    public Callable toggleMineralGateCallable() {
        return () -> {
            if (mineralGateOpen) {
                closeMineralGate();
                mineralGateOpen = false;
            } else {
                openMineralGate();
                mineralGateOpen = true;
            }
            return true;
        };
    }

    public Callable robotLiftUpCallable() {
        return () -> {
            hardware.robotLift.robotLiftUp();
            return true;
        };

    }

    public Callable robotLiftDownCallable() {
        return () -> {
            hardware.robotLift.robotLiftDown();
            return true;
        };
    }

    public Callable robotLiftStopCallable() {
        return () -> {
            hardware.robotLift.robotLiftStop();
            return true;
        };
    }

    public Callable moveRobotLiftToTopCallable() {
        return () -> {
            hardware.robotLift.robotLiftTop();
            return true;
        };
    }

    public Callable moveRobotLiftToBottomCallable() {
        return () -> {
            hardware.robotLift.robotLiftBottom();
            return true;
        };
    }
}



