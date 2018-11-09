package org.firstinspires.ftc.teamcode.omni_base.hardware.devices.glyph;

import org.firstinspires.ftc.teamcode.framework.SubsystemController;

public class GlyphController extends SubsystemController {

    private Glyph glyph;

    private Glyph.liftHeights liftHeight;

    public GlyphController(){
        init();
    }

    @Override
    public void init() {
        opModeSetup();
        glyph = new Glyph(hwMap);
        liftHeight = Glyph.liftHeights.BOTTOM;
    }

    @Override
    public void stop() {

    }

    public void clawOpen(){
        glyph.setClawServoPosition(0.4);
        glyph.setClawPower(-0.8);
        try {
            Thread.sleep(800);
        } catch (InterruptedException e){
        }
        glyph.setClawPower(0);
    }

    public void clawClose(){
        glyph.setClawPower(0.9);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e){
        }
        glyph.setClawPower(0.7);
        glyph.setClawServoPosition(0.2);
    }

    public void liftArm(){
        liftHeight = Glyph.liftHeights.up(liftHeight);
        glyph.setLiftPosition(liftHeight.getHeight(),0.8);
    }

    public void lowerArm(){
        liftHeight = Glyph.liftHeights.down(liftHeight);
        glyph.setLiftPosition(liftHeight.getHeight(),0.8);
    }
}
