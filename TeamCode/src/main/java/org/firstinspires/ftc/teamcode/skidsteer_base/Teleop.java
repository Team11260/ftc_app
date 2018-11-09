package org.firstinspires.ftc.teamcode.skidsteer_base;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.framework.AbstractTeleop;
import org.firstinspires.ftc.teamcode.skidsteer_base.hardware.Robot;

@TeleOp(name="SkidSteer_TeleOp_Stable", group="New")
//@Disabled

public class Teleop extends AbstractTeleop {

    private Robot robot;

    @Override
    public void Init() {
        robot = new Robot();
    }

    @Override
    public void Loop() {
        robot.updateDrive();
        telemetry.update();
    }

    @Override
    public void Stop() {
        robot.stop();
    }



    @Override
    public void lsy_change(double lsy) {
        robot.setDriveY(lsy);
    }

    @Override
    public void rsx_change(double rsx) {
        robot.setDriveZ(rsx);
    }
}