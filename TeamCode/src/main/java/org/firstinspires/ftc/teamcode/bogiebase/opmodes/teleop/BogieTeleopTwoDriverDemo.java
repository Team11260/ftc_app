package org.firstinspires.ftc.teamcode.bogiebase.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bogiebase.hardware.Robot;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractTeleop;

@TeleOp(name = "Bogie Teleop Two Driver Demo", group = "New")
//@Disabled

public class BogieTeleopTwoDriverDemo extends AbstractTeleop {

    private Robot robot;

    @Override
    public void RegisterEvents() {
        ////////////////Gamepad 1////////////////
        ////////Drive////////
        addEventHandler("1_a_down", robot.toggleAngleServoTiltAngleCallable());

        addEventHandler("1_b_down", robot.dropMarkerCallable());

        addEventHandler("1_rt_down", robot.toggleMineralGateCallable());

        ////////////////Gamepad 2////////////////
        ////////Intake////////
        addEventHandler("2_b_down", robot.beginIntakingCallable());

        addEventHandler("2_b_up", robot.finishIntakingCallable());

        addEventHandler("2_x_down", robot.reverseIntakeCallable());

        addEventHandler("2_x_up", robot.finishIntakingCallable());

        ///////Mineral Lift////////
        addEventHandler("2_rt_down", robot.moveMineralLiftToCollectPositionCallable());

        addEventHandler("2_rb_down", robot.moveMineralLiftToDumpPositionCallable());

        addEventHandler("2_dpd_down", robot.toggleAngleServoTiltAngleCallable());

        ////////Robot Lift////////
        addEventHandler("2_lb_down", robot.robotLiftUpCallable());

        addEventHandler("2_lb_up", robot.robotLiftStopCallable());

        addEventHandler("2_lt_down", robot.robotLiftDownCallable());

        addEventHandler("2_lt_up", robot.robotLiftStopCallable());
    }

    @Override
    public void UpdateEvents() {
        //NEVER EVER PUT BLOCKING CODE HERE!!!
        checkBooleanInput("1_rt",gamepad1.right_trigger > 0.5);

        checkBooleanInput("2_lt", gamepad2.left_trigger > 0.5);
        checkBooleanInput("2_rt", gamepad2.right_trigger > 0.5);

        robot.setDrivePower((-gamepad1.left_stick_y) + gamepad1.left_stick_x, (-gamepad1.left_stick_y) - gamepad1.left_stick_x);
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