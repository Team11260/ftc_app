package org.firstinspires.ftc.teamcode.omni_base.hardware.devices.jewel;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Jewel {
    Servo jewelservo;
    public Jewel(HardwareMap hwMap){
        jewelservo = hwMap.servo.get("jewelservo");
        jewelservo.setPosition(0.04);
    }
    public void armUp(){
        jewelservo.setPosition(0.04);
    }
    public void armDown(){
        jewelservo.setPosition(0.42);
    }
}
