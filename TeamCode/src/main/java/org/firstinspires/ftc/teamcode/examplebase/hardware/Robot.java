package org.firstinspires.ftc.teamcode.examplebase.hardware;

import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.ExpansionHubMonitor;
import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.SamplePosition;
import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.tensorflow.TensorFlowImpl;
import org.firstinspires.ftc.teamcode.framework.userhardware.paths.Path;
import org.firstinspires.ftc.teamcode.framework.util.AbstractRobot;
import org.firstinspires.ftc.teamcode.framework.util.RobotCallable;

import static org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry.LogMode.INFO;
import static org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry.LogMode.TRACE;
import static org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.SamplePosition.UNKNOWN;

public class Robot extends AbstractRobot {

    private HardwareDevices hardware;
    private ExpansionHubMonitor hub;

    //Robot Methods
    public Robot() {
        hardware = new HardwareDevices();
        hub = new ExpansionHubMonitor("Expansion Hub 1");
    }

    public void updateAll() {
        if (RobotState.currentMatchState == RobotState.MatchState.TELEOP) {
            hardware.subsystem.update();
        }

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

    public void stop() {
        if (RobotState.currentMatchState == RobotState.MatchState.AUTONOMOUS) stopTensorFlow();
        hardware.stop();
    }
}