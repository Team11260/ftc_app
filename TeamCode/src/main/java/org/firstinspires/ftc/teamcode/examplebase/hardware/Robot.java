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
        updateTelemetry();
    }

    public void updateTelemetry() {
        telemetry.update();
    }

    public void stop() {
        hardware.stop();
    }
}