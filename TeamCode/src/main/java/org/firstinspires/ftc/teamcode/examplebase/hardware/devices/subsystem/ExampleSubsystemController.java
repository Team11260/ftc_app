package org.firstinspires.ftc.teamcode.examplebase.hardware.devices.subsystem;

import org.firstinspires.ftc.teamcode.framework.util.SubsystemController;

public class ExampleSubsystemController extends SubsystemController {

    private ExampleSubsystem subsystem;

    //Utility Methods
    public ExampleSubsystemController() {
        init();
    }

    public synchronized void init() {
        subsystem = new ExampleSubsystem(hardwareMap);
    }

    public synchronized void update() {

    }

    public synchronized void stop() {
        subsystem.stop();
    }
}