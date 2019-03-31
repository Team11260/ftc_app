package org.firstinspires.ftc.teamcode.bogiebase.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractTeleop;
import org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry;

@TeleOp(name = "OpMode Setup", group = "New")
//@Disabled

public class OpModeSetup extends AbstractTeleop {

    @Override
    public void RegisterEvents() {
        addEventHandler("1_dpu_down", () -> {
            int value = telemetry.getInt("delay", 0);
            if(value < 10) telemetry.putInt("delay", value + 1);
            return true;
        });
        addEventHandler("1_dpd_down", () -> {
            int value = telemetry.getInt("delay", 0);
            if(value > 0) telemetry.putInt("delay", value - 1);
            return true;
        });
        addEventHandler("1_x_down", () -> {
            telemetry.putBoolean("teleop_position", !telemetry.getBoolean("teleop_position", false));
            return true;
        });
        addEventHandler("1_b_down", () -> {
            telemetry.putInt("delay", 0);
            telemetry.putBoolean("teleop_position", false);
            return true;
        });
    }

    @Override
    public void UpdateEvents() {

    }

    @Override
    public void Init() {

    }

    @Override
    public void Loop() {
        telemetry.getSmartdashboard().putValue("Auton Delay", telemetry.getInt("delay", 0));
        telemetry.getSmartdashboard().putValue("Teleop Position", telemetry.getBoolean("teleop_position", false) ? "Depot Side" : "Crater Side");

        telemetry.addDataPhone(DoubleTelemetry.LogMode.INFO, "Auton Delay", telemetry.getInt("delay", 0));
        telemetry.addDataPhone(DoubleTelemetry.LogMode.INFO, "Teleop Position", telemetry.getBoolean("teleop_position", false) ? "Depot Side" : "Crater Side");
        telemetry.update();
    }

    @Override
    public void Stop() {

    }
}
