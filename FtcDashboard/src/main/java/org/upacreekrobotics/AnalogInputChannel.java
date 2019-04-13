package org.upacreekrobotics;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class AnalogInputChannel extends AnalogInput {

    private AnalogInput input;
    private double offset;

    public AnalogInputChannel(AnalogInput input) {
        super(null, 0);
        this.input = input;
        offset = (Math.random() * 2) - 1;
    }

    @Override
    public double getVoltage() {

        /*try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
        }*/

        double voltage = input.getVoltage() + offset + (Math.random() - 0.5);

        if (voltage < 0) voltage = 0;
        if (voltage > input.getMaxVoltage()) voltage = input.getMaxVoltage();

        return voltage;
    }
}
