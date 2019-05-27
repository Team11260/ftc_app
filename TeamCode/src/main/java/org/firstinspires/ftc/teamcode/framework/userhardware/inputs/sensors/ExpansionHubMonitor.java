package org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractOpMode;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

public class ExpansionHubMonitor {

    private ExpansionHubEx hub;
    private ExpansionHubMotor[] motors = new ExpansionHubMotor[4];

    public ExpansionHubMonitor(String hubName) {
        hub = AbstractOpMode.getHardwareMap().get(ExpansionHubEx.class, hubName);

        //TODO test this
        //hub.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.HIGH_3_4M);
    }

    public double getVoltage() {
        return hub.read12vMonitor() / 1000.0;
    }

    public double getTotalCurrentDraw() {
        return hub.getTotalModuleCurrentDraw() / 1000.0;
    }

    public double getCurrentDrawMotor0() {
        return hub.getMotorCurrentDraw(0) / 1000.0;
    }

    public double getCurrentDrawMotor1() {
        return hub.getMotorCurrentDraw(1) / 1000.0;
    }

    public double getCurrentDrawMotor2() {
        return hub.getMotorCurrentDraw(2) / 1000.0;
    }

    public double getCurrentDrawMotor3() {
        return hub.getMotorCurrentDraw(3) / 1000.0;
    }
}
