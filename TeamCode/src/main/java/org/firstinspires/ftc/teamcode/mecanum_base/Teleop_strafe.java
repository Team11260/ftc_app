package org.firstinspires.ftc.teamcode.mecanum_base;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.framework.AbstractTeleop;
import org.firstinspires.ftc.teamcode.mecanum_base.hardware.HardwareDevices;
import org.firstinspires.ftc.teamcode.mecanum_base.hardware.Robot;
import org.firstinspires.ftc.teamcode.skidsteer_base.hardware.MyRev2mDistanceSensor;

@TeleOp(name="Mecanum_TeleOp_strafe1", group="New")
//@Disabled

public class Teleop_strafe extends AbstractTeleop {
    private ElapsedTime runTime;
    private MyRev2mDistanceSensor dist;
    private MyRev2mDistanceSensor dist1;
    private HardwareDevices hardware;
    double myLsy = 0;
    double myLsy1 = 0;
    double counter = 0;
    DcMotor intakeMotor;
    private boolean a_button_pressed =false;
    CRServo vexMotor;
    private Robot robot;
    boolean IsSwitchOn = false;
    DigitalChannel mySwitch;  // Hardware Device Object

    @Override
    public void Init() {
        runTime = new ElapsedTime();


        // get a reference to our digitalTouch object.
//        mySwitch = hardwareMap.get(DigitalChannel.class, "switch");

        // set the digital channel to input.
//        mySwitch.setMode(DigitalChannel.Mode.INPUT);


//        intakeMotor = hardwareMap.dcMotor.get("intake");
//        vexMotor = hardwareMap.crservo.get("vex");

//        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
//        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        vexMotor.setPower(0);
//        intakeMotor.setPower(0);
        dist = new MyRev2mDistanceSensor("dist");
        dist1 = new MyRev2mDistanceSensor("dist1");
        hardware = new HardwareDevices();
        robot = new Robot();
    }

    @Override
    public void Loop() {
//        if (mySwitch.getState() == true)
//        {
//            telemetry.addData("Switch is on");
//            intakeMotor.setPower(0);
//            vexMotor.setPower(0);
//        }
//        else
//        {
//            telemetry.addData("Switch is off");
//      }
//        if (a_button_pressed == false) {
//            telemetry.addData("BackDistance = ", dist1.getDistanceIN());
//            telemetry.addData("FrontDistance = ", dist.getDistanceIN());
//            telemetry.addData("currentangle=" + hardware.getHeading());
//            telemetry.addData("Front Left Encoders =" + robot.getFrontLeftPosition());
//            telemetry.addData("Front Right Encoders =" + robot.getFrontRightPosition());
//            telemetry.addData("Back Left Encoders =" + robot.getBackLeftPosition());
//            telemetry.addData("Back Right Encoders =" + robot.getBackRightPosition());
//
//        hardware.updateDrive();
            telemetry.update();
//        }


    }  // end of loop

    @Override
    public void Stop() {
        hardware.stop();
    }

    @Override
    public void lsx_change(double lsx) {
        hardware.setDriveX(lsx);
    }

    @Override
    public void lsy_change(double lsy) {
        hardware.setDriveY(lsy);
    }

    @Override
    public void rsx_change(double rsx) {
        hardware.setDriveZ(rsx);
    }

    @Override
    public void a_down()
    {
        a_button_pressed = true;
        telemetry.addData("A Pressed");
        telemetry.addData("Front Left Encoders =" + robot.getFrontLeftPosition());
        telemetry.addData("Front Right Encoders =" + robot.getFrontRightPosition());
        telemetry.addData("Back Left Encoders =" + robot.getBackLeftPosition());
        telemetry.addData("Back Right Encoders =" + robot.getBackRightPosition());
        telemetry.update();
//        robot.resetAllEncoders();
        runTime.reset();
        hardware.driveMotors(0.5);
        while (runTime.milliseconds() <= 1000) {
        }
        hardware.driveMotors(0);
        telemetry.addData("Front Left Encoders =" + robot.getFrontLeftPosition());
        telemetry.addData("Front Right Encoders =" + robot.getFrontRightPosition());
        telemetry.addData("Back Left Encoders =" + robot.getBackLeftPosition());
        telemetry.addData("Back Right Encoders =" + robot.getBackRightPosition());
        telemetry.update();
        a_button_pressed= false;

    }
//
    @Override
    public void b_down()
    {
        a_button_pressed = true;
        telemetry.addData("B Pressed");
        telemetry.update();
        runTime.reset();
        hardware.driveMotors(-0.5);
        while (runTime.milliseconds() <= 1000) {

        }
        hardware.driveMotors(0);
        telemetry.addData("Front Left Encoders =" + robot.getFrontLeftPosition());
        telemetry.addData("Front Right Encoders =" + robot.getFrontRightPosition());
        telemetry.addData("Back Left Encoders =" + robot.getBackLeftPosition());
        telemetry.addData("Back Right Encoders =" + robot.getBackRightPosition());
        telemetry.update();
        telemetry.update();
        a_button_pressed = false;
    }
//
//    @Override
//    public void x_down()
//    {
//        vexMotor.setPower(0.5);
//    }
//
//    @Override
//    public void y_down()
//    {
//        vexMotor.setPower(-0.5);
//    }
//
//    @Override
//    public void a_up()
//    {
//        intakeMotor.setPower(0);
//    }
//
//    @Override
//    public void b_up()
//    {
//        intakeMotor.setPower(0);
//    }
//
//    @Override
//    public void x_up()
//    {
//        vexMotor.setPower(0);
//    }
//
//    @Override
//    public void y_up()
//    {
//        vexMotor.setPower(0);
//    }

    @Override
    public void dpr_down() {
        telemetry.addData("dpad right Pressed");
        telemetry.update();
        delay(5000);
        hardware.resetAngleToZero();
        hardware.strafeToMyDistanceGyro(.5,-2000,0,1, 0);
        //1st = 38.5 2nd = 40.5 3rd = 41 average = 40 Encoders = 2000
    }

    @Override
    public void dpl_down() {
        hardware.resetAngleToZero();
        hardware.strafeToMyDistanceGyro(.5,2000,0,1, 0);
        //1st = 32 in 2nd = 24 3rd =31.5 average = 32.5 Encoders = 2000

    }

    @Override
    public void dpu_down() {
        hardware.resetAngleToZero();
        hardware.strafeToMyDistanceGyro(.5,2000,0,1, 1);
        //1st = 48.5 2nd = 48.5 3rd = 47 Average = 48 Encoders = 20000
    }

    @Override
    public void dpd_down() {
        hardware.resetAngleToZero();
        hardware.strafeToMyDistanceGyro(.5,-2000,0,1, 1);
        // 1st = 49.5 2nd = 49.5 3rd = 49.5 Average = 49.5 Encoders = 2000
    }
}