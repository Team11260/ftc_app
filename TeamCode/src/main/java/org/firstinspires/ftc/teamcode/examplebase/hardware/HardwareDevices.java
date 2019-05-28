package org.firstinspires.ftc.teamcode.examplebase.hardware;

import org.firstinspires.ftc.teamcode.examplebase.hardware.devices.subsystem.DriveController;

public class HardwareDevices {

    public DriveController drive = null;

    public HardwareDevices() {
        drive = new DriveController();
    }

    public void stop() {
        if (drive != null) drive.stop();
    }
}
