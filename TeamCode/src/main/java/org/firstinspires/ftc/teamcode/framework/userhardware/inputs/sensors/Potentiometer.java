package org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractOpMode;

public class Potentiometer {

    private AnalogInput pot;

    public Potentiometer(String name) {
      //  pot = AbstractOpMode.getHwMap().analogInput.get(name);
        pot = AbstractOpMode.getHardwareMap().analogInput.get(name);
    }

    public double getVoltage() {
        return pot.getVoltage();
    }
}
