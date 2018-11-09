package org.firstinspires.ftc.teamcode.omni_base.hardware.devices.jewel;

import org.firstinspires.ftc.teamcode.framework.SubsystemController;

public class JewelController extends SubsystemController {
    Jewel jewel;
    public JewelController(){
        init();
    }
    @Override
    public void init() {
        opModeSetup();
        jewel = new Jewel(hwMap);
    }

    @Override
    public void stop() {

    }
    public void armUp(){
        jewel.armUp();
    }
    public void armDown(){
        jewel.armDown();
    }
}
