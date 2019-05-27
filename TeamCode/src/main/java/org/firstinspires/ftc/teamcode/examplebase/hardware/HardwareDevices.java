package org.firstinspires.ftc.teamcode.examplebase.hardware;

import org.firstinspires.ftc.teamcode.examplebase.hardware.devices.subsystem.ExampleSubsystemController;

public class HardwareDevices {

    public ExampleSubsystemController subsystem = null;

    public HardwareDevices() {
        subsystem = new ExampleSubsystemController();
    }

    public void stop() {
        if (subsystem != null) subsystem.stop();
    }
}
