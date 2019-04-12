package org.upacreekrobotics;

public class HardwareMap {

    private com.qualcomm.robotcore.hardware.HardwareMap hardwareMap;

    public AnalogInputFactory analogInput;

    public HardwareMap(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        analogInput = new AnalogInputFactory();
    }

    public class AnalogInputFactory {

        public AnalogInputChannel get(String name) {
            return new AnalogInputChannel(hardwareMap.analogInput.get(name));
        }
    }
}
