package org.firstinspires.ftc.teamcode.mecanum_base;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.framework.AbstractTeleop;
import org.firstinspires.ftc.teamcode.mecanum_base.hardware.HardwareDevices;

@TeleOp(name="Mecanum_TeleOp_Switch", group="New")
//@Disabled

public class Teleop_switch extends AbstractTeleop {

    private HardwareDevices robot;
    DcMotor intakeMotor;
    CRServo vexMotor;

    boolean IsSwitchOn = false;
    DigitalChannel mySwitch;  // Hardware Device Object

    boolean testMotorInProgress = false;

    @Override
    public void Init()
    {


        // get a reference to our digitalTouch object.
        mySwitch = hardwareMap.get(DigitalChannel.class, "switch");

        // set the digital channel to input.
        mySwitch.setMode(DigitalChannel.Mode.INPUT);


        intakeMotor = hardwareMap.dcMotor.get("intake");
        vexMotor = hardwareMap.crservo.get("vex");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vexMotor.setPower(0);
        intakeMotor.setPower(0);

        robot = new HardwareDevices();

    }

    @Override
    public void Loop() {

        if (testMotorInProgress == false) {
            robot.updateDrive();
        }

        if (mySwitch.getState() == true)
        {
            telemetry.addData("Switch is on");
            intakeMotor.setPower(0);
            vexMotor.setPower(0);


        }
        else
        {
            telemetry.addData("Switch is off");
        }

        telemetry.update();

    }  // end of loop

    @Override
    public void Stop() {
        robot.stop();
    }

    @Override
    public void lsx_change(double lsx) {
        robot.setDriveX(lsx);
    }

    @Override
    public void lsy_change(double lsy) {
        robot.setDriveY(lsy);
    }

    @Override
    public void rsx_change(double rsx) {
        robot.setDriveZ(rsx);
    }

    @Override
    public void a_down()
    {
            intakeMotor.setPower(0.5);




    }

    @Override
    public void b_down()
    {
        intakeMotor.setPower(-0.5);
    }

    @Override
    public void x_down()
    {
        vexMotor.setPower(0.5);
    }

    @Override
    public void y_down()
    {
        vexMotor.setPower(-0.5);
    }

    @Override
    public void a_up()
    {
        intakeMotor.setPower(0);
    }

    @Override
    public void b_up()
    {
        intakeMotor.setPower(0);
    }

    @Override
    public void x_up()
    {
        vexMotor.setPower(0);
    }

    @Override
    public void dpl_down()
    {
        if(testMotorInProgress == false) {
            testMotorInProgress = true;
            robot.measureEncoders(0, 5000);
            testMotorInProgress = false;
        }
    }

    @Override
    public void dpu_down()
    {
        if(testMotorInProgress == false) {
            testMotorInProgress = true;
            robot.measureEncoders(1, 5000);
            testMotorInProgress = false;
        }
    }

    @Override
    public void dpd_down()
    {
        if(testMotorInProgress == false) {
            testMotorInProgress = true;
            robot.measureEncoders(2, 5000);
            testMotorInProgress = false;
        }
    }

    @Override
    public void dpr_down()
    {
        if(testMotorInProgress == false) {
            testMotorInProgress = true;
            robot.measureEncoders(3, 5000);
            testMotorInProgress = false;
        }
    }


    @Override
    public void y_up()
    {
        vexMotor.setPower(0);
    }



}