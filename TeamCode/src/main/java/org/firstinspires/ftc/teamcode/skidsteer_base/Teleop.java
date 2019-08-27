package org.firstinspires.ftc.teamcode.skidsteer_base;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.framework.AbstractTeleop;
import org.firstinspires.ftc.teamcode.framework.userhardware.outputs.SlewDcMotor;
import org.firstinspires.ftc.teamcode.skidsteer_base.hardware.Robot;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.Scanner;

import static java.lang.Math.abs;

@TeleOp(name="SkidSteer_TeleOp_Stable", group="New")
//@Disabled

public class Teleop extends AbstractTeleop {

    private Robot robot;
    private SlewDcMotor armMotor;
    private int ArmPosition = 0;
    private int ArmPositionFile;
    private boolean lastButtonPressed = false;
    private boolean lastButtonPressedA = false;
    private double speedUP = 0;
    private boolean movingUP = false;
    private double slowSpeed = 0;
    private int SlowDownTarget;
    private Double SpeedDownFile;

    @Override
    public void Init() {
        armMotor = new SlewDcMotor(hardwareMap.dcMotor.get("arm"));
        armMotor.setDirection(SlewDcMotor.Direction.FORWARD);
        armMotor.setMode(SlewDcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(SlewDcMotor.RunMode.RUN_TO_POSITION);
        robot = new Robot();

        armMotor.setSlewSpeed(1);


        try{
            File file =
                    new File("/sdcard/FIRST/speed.txt");
            Scanner sc = new Scanner(file);
            speedUP = sc.nextDouble();

            File file1 =
                    new File("/sdcard/FIRST/slowSpeed.txt");
            Scanner sc1 = new Scanner(file1);
            slowSpeed = sc1.nextDouble();

            File file2 =
                    new File("/sdcard/FIRST/ArmPosition.txt");
            Scanner sc2 = new Scanner(file2);
            ArmPositionFile = sc2.nextInt();

            File file3 =
                    new File("/sdcard/FIRST/SlowDownTarget.txt");
            Scanner sc3 = new Scanner(file3);
            SlowDownTarget = sc3.nextInt();

            File file4 =
                    new File("/sdcard/FIRST/SpeedDownFile.txt");
            Scanner sc4 = new Scanner(file4);
            SpeedDownFile = sc4.nextDouble();


        } catch (IOException e){
            Log.e("Exception", "File init failed " + e.toString());
        }


    }

    @Override
    public void Loop() {
        if (gamepad1.y && lastButtonPressed  == false) {
            movingUP = false;
            armMotor.setPower(SpeedDownFile);//arm down
            ArmPosition = 0;
            armMotor.setTargetPosition(ArmPosition);
        }
        lastButtonPressed = gamepad1.y;

        if (gamepad1.a && lastButtonPressedA  == false) {
            movingUP = true;
            armMotor.setPower(speedUP);//arm up
           ArmPosition = ArmPositionFile;
            armMotor.setTargetPosition(ArmPosition);
        }
        if (movingUP == true) {
            if (abs(armMotor.getCurrentPosition() - ArmPosition) < SlowDownTarget) {
                armMotor.setPower(slowSpeed);
            }

        }
        lastButtonPressedA = gamepad1.a;
        robot.updateDrive();
        telemetry.addData("Arm Position : " + ArmPosition);
        telemetry.addData("Arm Speed " + armMotor.getPower());
        telemetry.addData("actual Motor : " + armMotor.getCurrentPosition());
        telemetry.addData("Speed is " + speedUP);
        telemetry.addData("Slow Speed is " + slowSpeed);
        telemetry.addData("ArmUpPosition is " + ArmPositionFile);
        telemetry.addData("Slow Down Target is " + SlowDownTarget);
        telemetry.addData("Speed Down is " + SpeedDownFile);
        telemetry.update();
    }

    @Override
    public void Stop() {
        robot.stop();
    }



    @Override
    public void lsy_change(double lsy) {
        robot.setDriveY(lsy);
    }

    @Override
    public void rsx_change(double rsx) {
        robot.setDriveZ(rsx);
    }
}