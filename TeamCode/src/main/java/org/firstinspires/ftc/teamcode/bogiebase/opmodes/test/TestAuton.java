package org.firstinspires.ftc.teamcode.bogiebase.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAutonNew;
import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.vuforia.VuforiaImpl;

@Autonomous(name = "Test Auton", group = "New")
@Disabled

public class TestAuton extends AbstractAutonNew {

    VuforiaImpl vuforia;

    @Override
    public void RegisterStates() {

    }

    @Override
    public void Init() {
        vuforia = new VuforiaImpl(true);
    }

    @Override
    public void Run() {
        vuforia.getImage();
    }

    @Override
    public void Stop() {

    }
}
