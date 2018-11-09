package org.firstinspires.ftc.teamcode.test.hardware;


import org.firstinspires.ftc.teamcode.framework.AbstractRobot;

public class Robot extends AbstractRobot {

    private HardwareDevices hardware;

    public Robot(){
        hardware = new HardwareDevices();
    }

    public void stop(){
        hardware.stop();
    }
}