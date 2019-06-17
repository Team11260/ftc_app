package org.firstinspires.ftc.teamcode.examplebase.hardware;

import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.ExpansionHubMonitor;
import org.firstinspires.ftc.teamcode.framework.util.AbstractRobot;

public class Robot extends AbstractRobot {

    private HardwareDevices hardware;
    private ExpansionHubMonitor hub;


    //Robot Methods
    public Robot() {
        hardware = new HardwareDevices();
        hub = new ExpansionHubMonitor("Expansion Hub 1");
    }

    public void updateAll() {
        //if (RobotState.currentMatchState == RobotState.MatchState.TELEOP) {
            hardware.drive.update();
        //}

        updateTelemetry();
    }


    public void updateTelemetry() {
        telemetry.update();
    }

    public double getVoltage() {
        return hub.getVoltage();
    }

    public double getCurrent() {
        return hub.getTotalCurrentDraw();
    }

    public void driveInches(int inches){
        hardware.drive.driveInches(inches);
    }

    public void turnLeft(int angle) {
        hardware.drive.turnLeft(angle);
    }

    public void turnRight(int angle) {
        hardware.drive.turnRight(angle);
    }

    public void stop() {
        hardware.stop();
    }
}