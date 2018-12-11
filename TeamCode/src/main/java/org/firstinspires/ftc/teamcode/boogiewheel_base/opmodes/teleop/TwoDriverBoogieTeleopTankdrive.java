package org.firstinspires.ftc.teamcode.boogiewheel_base.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.boogiewheel_base.hardware.Robot;
import org.firstinspires.ftc.teamcode.framework.opModes.AbstractTeleop;

@TeleOp(name = "TwoDriver BoogieWheel Teleop Tankdrive", group = "New")
//@Disabled

public class TwoDriverBoogieTeleopTankdrive extends AbstractTeleop {

    private Robot robot;

    private boolean driver1 = true;

    @Override
    public void RegisterEvents() {
        twoDrivers();
    }

    private void twoDrivers() {
        ////////////////Gamepad 1////////////////
        ////////Drive////////
        //THIS CODE HAS BEEN MODIFIED FOR TANKDRIVE

        ////////Intake////////
        addEventHandler("1_a_down", robot.finishIntakingCallable());

        addEventHandler("1_x_down", robot.beginIntakingCallable());

        addEventHandler("1_b_down", robot.reverseIntakeCallable());

        ////////Robot Lift////////
        addEventHandler("1_dpu_down", robot.robotLiftUpCallable());

        addEventHandler("1_dpd_down", robot.robotLiftDownCallable());

        addEventHandler("1_dpu_up", robot.robotLiftStopCallable());

        addEventHandler("1_dpd_up", robot.robotLiftStopCallable());

        ////////Util////////
        addEventHandler("1_y_down", () -> {
            driver1 = false;
            ////Gamepad 1////
            //Drive
            pauseEvent("1_lsy_change");
            pauseEvent("1_rsx_change");

            //Intake
            pauseEvent("1_b_down");
            pauseEvent("1_a_down");
            pauseEvent("1_y_down");

            //Robot Lift
            pauseEvent("1_dpu_down");
            pauseEvent("1_dpd_down");
            pauseEvent("1_dpu_up");
            pauseEvent("1_dpd_up");

            //Util
            pauseEvent("1_y_down");

            ////Gamepad 2////
            //Drive
            resumeEvent("2_lsy_change");
            resumeEvent("2_rsx_change");

            //Mineral Lift
            resumeEvent("2_rt_down");
            resumeEvent("2_lt_down");
            resumeEvent("2_x_down");

            //Util
            resumeEvent("2_y_down");

            return true;
        });

        ////////////////Gamepad 2////////////////
        ////////Drive////////
        addEventHandler("2_lsy_change", () -> {
            robot.setDrivePower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
            return true;
        });
        pauseEvent("2_lsy_change");

        addEventHandler("2_rsx_change", () -> {
            robot.setDrivePower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
            return true;
        });
        pauseEvent("2_rsx_change");

        ///////Mineral Lift////////
        addEventHandler("2_rt_down", robot.moveMineralLiftToDumpPositionCallable());
        pauseEvent("2_rt_down");

        addEventHandler("2_lt_down", robot.moveMineralLiftToCollectPositionCallable());
        pauseEvent("2_lt_down");

        addEventHandler("2_x_down", robot.toggleMineralGateCallable());
        pauseEvent("2_x_down");

        ////////Util////////
        addEventHandler("2_y_down", () -> {
            driver1 = true;
            ////Gamepad 1////
            //Drive
            resumeEvent("1_lsy_change");
            resumeEvent("1_rsx_change");

            //Intake
            resumeEvent("1_b_down");
            resumeEvent("1_a_down");
            resumeEvent("1_y_down");

            //Robot Lift
            resumeEvent("1_dpu_down");
            resumeEvent("1_dpd_down");
            resumeEvent("1_dpu_up");
            resumeEvent("1_dpd_up");

            //Util
            resumeEvent("1_y_down");

            ////Gamepad 2////
            //Drive
            pauseEvent("2_lsy_change");
            pauseEvent("2_rsx_change");

            //Mineral Lift
            pauseEvent("2_rt_down");
            pauseEvent("2_lt_down");
            pauseEvent("2_x_down");

            //Util
            pauseEvent("2_y_down");

            return true;
        });
    }

    @Override
    public void UpdateEvents() {
        //NEVER EVER PUT BLOCKING CODE HERE!!!
        checkBooleanInput("2_lt", gamepad2.left_trigger > 0.5);
        checkBooleanInput("2_rt", gamepad2.right_trigger > 0.5);

        if(driver1) robot.setDrivePower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        else robot.setDrivePower(gamepad2.right_stick_y, gamepad2.left_stick_y);
    }

    @Override
    public void Init() {
        robot = new Robot();
    }

    @Override
    public void Loop() {
        robot.updateAll();
        telemetry.update();
    }

    @Override
    public void Stop() {
        robot.stop();
    }

}