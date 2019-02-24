package org.firstinspires.ftc.teamcode.skidsteerbase.hardware;

import org.firstinspires.ftc.teamcode.skidsteerbase.hardware.devices.arm.ArmController;
import org.firstinspires.ftc.teamcode.skidsteerbase.hardware.devices.drive.DriveController;

public class HardwareDevices {

    public DriveController drive = null;
    public ArmController arm = null;

    public HardwareDevices() {
        drive = new DriveController();
        arm = new ArmController();
    }

    public void stop() {
        if (drive != null) drive.stop();
        if (arm != null) arm.stop();
    }
}
