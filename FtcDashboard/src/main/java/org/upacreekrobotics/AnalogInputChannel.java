package org.upacreekrobotics;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class AnalogInputChannel extends AnalogInput {

    private AnalogInput input;
    private double offset;

    public AnalogInputChannel(AnalogInput input) {
        super(null, 0);
        this.input = input;
        offset = (Math.random() * 2) - 1;
        new Thread(this::test).start();
    }

    public void test() {
        try {
            Thread.sleep(2);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        test();
    }

    @Override
    public double getVoltage() {

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
        }

        double voltage = super.getVoltage() + offset + (Math.random() - 0.5);

        if (voltage < 0) voltage = 0;
        if (voltage > getMaxVoltage()) voltage = getMaxVoltage();

        return voltage;
    }
}
