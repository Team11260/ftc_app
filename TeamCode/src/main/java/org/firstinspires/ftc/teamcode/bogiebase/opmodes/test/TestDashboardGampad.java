package org.firstinspires.ftc.teamcode.bogiebase.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractTeleop;

@TeleOp(name = "Test Dashboard Gamepad", group = "New")
@Disabled

public class TestDashboardGampad extends AbstractTeleop {

    @Override
    public void RegisterEvents() {

    }

    @Override
    public void UpdateEvents() {

    }

    @Override
    public void Init() {

    }

    @Override
    public void Loop() {
        telemetry.getSmartdashboard().putValue("1 left_stick_x", gamepad1.left_stick_x);
        telemetry.getSmartdashboard().putValue("1 left_stick_y", gamepad1.left_stick_y);
        telemetry.getSmartdashboard().putValue("1 right_stick_x", gamepad1.right_stick_x);
        telemetry.getSmartdashboard().putValue("1 right_stick_y", gamepad1.right_stick_y);
        telemetry.getSmartdashboard().putValue("1 right_trigger", gamepad1.right_trigger);
        telemetry.getSmartdashboard().putValue("1 left_trigger", gamepad1.left_trigger);
        telemetry.getSmartdashboard().putBoolean("1 x", gamepad1.x);
        telemetry.getSmartdashboard().putBoolean("1 y", gamepad1.y);
        telemetry.getSmartdashboard().putBoolean("1 a", gamepad1.a);
        telemetry.getSmartdashboard().putBoolean("1 b", gamepad1.b);
        telemetry.getSmartdashboard().putBoolean("1 start", gamepad1.start);
        telemetry.getSmartdashboard().putBoolean("1 back", gamepad1.back);
        telemetry.getSmartdashboard().putBoolean("1 guide", gamepad1.guide);
        telemetry.getSmartdashboard().putBoolean("1 left_stick_button", gamepad1.left_stick_button);
        telemetry.getSmartdashboard().putBoolean("1 right_stick_button", gamepad1.right_stick_button);
        telemetry.getSmartdashboard().putBoolean("1 right_bumper", gamepad1.right_bumper);
        telemetry.getSmartdashboard().putBoolean("1 left_bumper", gamepad1.left_bumper);
        telemetry.getSmartdashboard().putBoolean("1 dpad_up", gamepad1.dpad_up);
        telemetry.getSmartdashboard().putBoolean("1 dpad_down", gamepad1.dpad_down);
        telemetry.getSmartdashboard().putBoolean("1 dpad_left", gamepad1.dpad_left);
        telemetry.getSmartdashboard().putBoolean("1 dpad_right", gamepad1.dpad_right);

        telemetry.getSmartdashboard().putValue("2 left_stick_x", gamepad2.left_stick_x);
        telemetry.getSmartdashboard().putValue("2 left_stick_y", gamepad2.left_stick_y);
        telemetry.getSmartdashboard().putValue("2 right_stick_x", gamepad2.right_stick_x);
        telemetry.getSmartdashboard().putValue("2 right_stick_y", gamepad2.right_stick_y);
        telemetry.getSmartdashboard().putValue("2 right_trigger", gamepad2.right_trigger);
        telemetry.getSmartdashboard().putValue("2 left_trigger", gamepad2.left_trigger);
        telemetry.getSmartdashboard().putBoolean("2 x", gamepad2.x);
        telemetry.getSmartdashboard().putBoolean("2 y", gamepad2.y);
        telemetry.getSmartdashboard().putBoolean("2 a", gamepad2.a);
        telemetry.getSmartdashboard().putBoolean("2 b", gamepad2.b);
        telemetry.getSmartdashboard().putBoolean("2 start", gamepad2.start);
        telemetry.getSmartdashboard().putBoolean("2 back", gamepad2.back);
        telemetry.getSmartdashboard().putBoolean("2 guide", gamepad2.guide);
        telemetry.getSmartdashboard().putBoolean("2 left_stick_button", gamepad2.left_stick_button);
        telemetry.getSmartdashboard().putBoolean("2 right_stick_button", gamepad2.right_stick_button);
        telemetry.getSmartdashboard().putBoolean("2 right_bumper", gamepad2.right_bumper);
        telemetry.getSmartdashboard().putBoolean("2 left_bumper", gamepad2.left_bumper);
        telemetry.getSmartdashboard().putBoolean("2 dpad_up", gamepad2.dpad_up);
        telemetry.getSmartdashboard().putBoolean("2 dpad_down", gamepad2.dpad_down);
        telemetry.getSmartdashboard().putBoolean("2 dpad_left", gamepad2.dpad_left);
        telemetry.getSmartdashboard().putBoolean("2 dpad_right", gamepad2.dpad_right);
    }

    @Override
    public void Stop() {

    }
}
