package org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractOpMode;

public class MRTouchSen {

    //private com.qualcomm.hardware.rev.Rev2mDistanceSensor timeOfFlightSensor;
    private AnalogInput MR_Switch;

    public MRTouchSen(String name) {
        MR_Switch = AbstractOpMode.getHardwareMap().get(AnalogInput.class,name); // Put this string on phone
    }

    public boolean getPressed() {
        if (MR_Switch.getVoltage() > 2.5) {
            return true;
        } else {
            return false;
        }
    }
}
