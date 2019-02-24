package org.firstinspires.ftc.teamcode.skidsteerbase.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractTeleop;
import org.firstinspires.ftc.teamcode.skidsteerbase.hardware.Robot;

@TeleOp(name = "Skid Steer Teleop Two Driver", group = "New")
//@Disabled

public class SkidSteerTeleopTwoDriver extends AbstractTeleop {

    private Robot robot;

    @Override
    public void RegisterEvents() {
        ////////////////Gamepad 1////////////////

        ////////////////Gamepad 2////////////////
        addEventHandler("2_rb_down",robot.moveToCollectPositionCallable());
        addEventHandler("2_rt_down",robot.moveToDumpPositionCallable());
        addEventHandler("2_b_down",robot.beginIntakingCallable());
        addEventHandler("2_b_up",robot.finishIntakingCallable());
        addEventHandler("2_x_down",robot.reverseIntakeCallable());
        addEventHandler("2_x_up",robot.finishIntakingCallable());
        addEventHandler("2_y_down",robot.toggleMineralGateCallable());
    }

    @Override
    public void UpdateEvents() {
        //NEVER EVER PUT BLOCKING CODE HERE!!!
        checkBooleanInput("2_lt", gamepad2.left_trigger > 0.5);
        checkBooleanInput("2_rt", gamepad2.right_trigger > 0.5);

        robot.setDrivePower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
    }

    @Override
    public void Init() {
        robot = new Robot();
    }

    @Override
    public void Loop() {
        robot.updateAll();
    }

    @Override
    public void Stop() {
        robot.stop();
    }
}