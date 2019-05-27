package org.firstinspires.ftc.teamcode.examplebase.hardware.devices.subsystem;

public class SubsystemController extends org.firstinspires.ftc.teamcode.framework.util.SubsystemController {

    private Subsystem subsystem;

    @Override
    public void init() {
        subsystem = new Subsystem();
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
        subsystem.stop();
    }
}
