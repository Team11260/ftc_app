package org.firstinspires.ftc.teamcode.examplebase.hardware;

import org.firstinspires.ftc.teamcode.examplebase.hardware.devices.subsystem.SubsystemController;

public class HardwareDevices {

    public SubsystemController subsystem = null;

    public HardwareDevices() {
        subsystem = new SubsystemController();
    }

    public void stop() {
        if(subsystem != null) subsystem.stop();
    }
}
