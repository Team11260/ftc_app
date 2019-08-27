package org.firstinspires.ftc.teamcode.omni_base.hardware;

import org.firstinspires.ftc.teamcode.framework.AbstractAuton;
import org.firstinspires.ftc.teamcode.framework.AbstractTeleop;
import org.firstinspires.ftc.teamcode.framework.userhardware.outputs.Speech;
import org.firstinspires.ftc.teamcode.omni_base.hardware.devices.drive.DriveController;
import org.firstinspires.ftc.teamcode.omni_base.hardware.devices.glyph.GlyphController;
import org.firstinspires.ftc.teamcode.omni_base.hardware.devices.jewel.JewelController;
import org.firstinspires.ftc.teamcode.omni_base.hardware.devices.relic.RelicController;

public class HardwareDevices {

    private Speech speech;

    private GlyphController glyph;

    private RelicController relic;

    private JewelController jewel;

    private DriveController drive;

    public HardwareDevices(){
        try {
            speech = new Speech(AbstractAuton.getOpModeInstance().hardwareMap);
        } catch (NullPointerException e){
            speech = new Speech(AbstractTeleop.getOpModeInstance().hardwareMap);
        }
        glyph = new GlyphController();
        relic = new RelicController();
        jewel = new JewelController();
        drive = new DriveController();
    }

    public void speak(String text){
        speech.speak(text);
    }

    public boolean isSpeaking(){
        return speech.isSpeaking();
    }

    //Glyph
    public void glyphClawOpen() {
        glyph.clawOpen();
    }

    public void glyphClawClose() {
        glyph.clawClose();
    }

    public void glyphLiftUp(){
        glyph.liftArm();
    }

    public void glyphLiftDown(){
        glyph.lowerArm();
    }

    //Relic
    public void relicFullUp(){
        relic.fullUp();
    }

    public void relicSmallUp(){
        relic.smallUp();
    }

    public void relicSmallDown(){
        relic.smallDown();
    }

    public void relicFullExtend(){
        relic.fullExtend();
    }

    public void relicFullRetract(){
        relic.fullRetract();
    }

    public void relicSmallExtend(){
        relic.smallExtend();
    }

    public void relicSmallRetract(){
        relic.smallRetract();
    }

    //Jewel
    public void jewelArmUp(){
        jewel.armUp();
    }

    public void jewelArmDown(){
        jewel.armDown();
    }

    //Drive
    public void setDriveX(double x) {
        drive.setX(x);
    }

    public void setDriveY(double y) {
        drive.setY(y);
    }

    public void setDriveZ(double z) {
        drive.setZ(z);
    }

    public void updateDrive(){
        drive.update();
    }

    public void setPower(double fl, double fr, double bl, double br){
        drive.setPower(fl,fr,bl,br);
    }

    public void stop(){
        drive.stop();
    }
}