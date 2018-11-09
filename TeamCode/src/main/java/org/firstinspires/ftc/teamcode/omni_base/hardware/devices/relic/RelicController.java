package org.firstinspires.ftc.teamcode.omni_base.hardware.devices.relic;

import org.firstinspires.ftc.teamcode.framework.SubsystemController;

public class RelicController extends SubsystemController{

    private Relic relic;

    public RelicController(){
        init();
    }

    @Override
    public void init() {
        opModeSetup();
        relic = new Relic(hwMap);
    }

    @Override
    public void stop() {

    }

    public void fullUp(){
        relic.setLiftPosition(750,0.7);
    }

    public void smallUp(){
        relic.setLiftPosition(relic.getLiftTargetPosition()+20,1);
    }

    public void smallDown(){
        relic.setLiftPosition(relic.getLiftTargetPosition()-10,0.4);
    }

    public void fullExtend(){
        if(relic.getExtendTargetPosition()>2950) relic.setExtendPosition(4000,0.8);
        else if(relic.getExtendTargetPosition()>1950) relic.setExtendPosition(3000,0.8);
        else relic.setExtendPosition(2000,0.8);
    }

    public void fullRetract(){
        if(relic.getExtendTargetPosition()>1950) relic.setExtendPosition(1900,0.8);
        else relic.setExtendPosition(100,0.8);
    }

    public void smallExtend(){
        relic.setExtendPosition(relic.getExtendTargetPosition()+50,0.5);
    }

    public void smallRetract(){
        relic.setExtendPosition(relic.getExtendTargetPosition()-50,0.5);
    }
}
