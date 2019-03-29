package org.firstinspires.ftc.teamcode.bogiebase.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractTeleop;

@TeleOp(name = "Auton Delay Editor", group = "New")
//@Disabled

public class DelayEditor extends AbstractTeleop {

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
    }

    @Override
    public void UpdateEvents() {

    }

    @Override
    public void Init() {

    }

    @Override
    public void Loop() {
        telemetry.getSmartdashboard().putValue("Delay", telemetry.getInt("delay", 0));
    }

    @Override
    public void Stop() {

    }
}
