package org.firstinspires.ftc.teamcode.bogiebase.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractTeleop;

@TeleOp(name = "Restart Robot", group = "New")

public class RestartRobot extends AbstractTeleop {

    @Override
    public void RegisterEvents() {

    }

    @Override
    public void UpdateEvents() {

    }

    @Override
    public void Init() {
        msStuckDetectStop = 0;
        requestOpModeStop();
    }

    @Override
    public void Loop() {

    }

    @Override
    public void Stop() {
        pause();
    }

    public void pause() {
        try {
            Thread.sleep(10000);
        } catch (InterruptedException e) {
            pause();
        }
    }
}
