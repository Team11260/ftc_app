package org.firstinspires.ftc.teamcode.bogiebase.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.Robot;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractTeleop;

@TeleOp(name = "Boogie Teleop One Driver", group = "New")
@Disabled

public class BogieTeleopOneDriver extends AbstractTeleop {

    private Robot robot;

    @Override
    public void RegisterEvents() {
        addEventHandler("1_lsb_down", robot.toggleDriveInvertedCallable());

        addEventHandler("1_lb_down", robot.dropMarkerCallable());

        ////////Intake////////
        addEventHandler("1_a_down", robot.finishIntakingCallable());

        addEventHandler("1_b_down", robot.beginIntakingCallable());

        addEventHandler("1_x_down", robot.reverseIntakeCallable());

        addEventHandler("1_dpr_down", robot.liftIntakeCallable());

        addEventHandler("1_dpl_down", robot.lowerIntakeCallable());

        ///////Mineral Lift////////
        addEventHandler("1_rt_down", robot.moveMineralLiftToDumpPositionCallable());

        addEventHandler("1_lt_down", robot.moveMineralLiftToCollectPositionCallable());

        addEventHandler("1_y_down", robot.toggleMineralGateCallable());

        ////////Robot Lift////////
        addEventHandler("1_dpu_down", robot.robotLiftUpCallable());

        addEventHandler("1_dpd_down", robot.robotLiftDownCallable());

        addEventHandler("1_dpu_up", robot.robotLiftStopCallable());

        addEventHandler("1_dpd_up", robot.robotLiftStopCallable());
    }

    @Override
    public void UpdateEvents() {
        //NEVER EVER PUT BLOCKING CODE HERE!!!
        checkBooleanInput("1_lt", gamepad1.left_trigger > 0.5);
        checkBooleanInput("1_rt", gamepad1.right_trigger > 0.5);
    }

    @Override
    public void Init() {
        robot = new Robot();
    }

    @Override
    public void Loop() {
        robot.setDrivePower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        robot.updateAll();
        telemetry.update();
    }

    @Override
    public void Stop() {
        robot.stop();
    }
}