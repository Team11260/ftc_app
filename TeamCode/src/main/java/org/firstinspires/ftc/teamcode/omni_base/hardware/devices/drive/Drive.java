package org.firstinspires.ftc.teamcode.omni_base.hardware.devices.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.framework.userhardware.outputs.SlewDcMotor;

public class Drive {

    private SlewDcMotor back_left, back_right, front_left, front_right;

    public Drive(HardwareMap hwMap){

        //Motors
        front_left = new SlewDcMotor(hwMap.dcMotor.get("front_left"));
        front_right = new SlewDcMotor(hwMap.dcMotor.get("front_right"));
        back_left = new SlewDcMotor(hwMap.dcMotor.get("back_left"));
        back_right = new SlewDcMotor(hwMap.dcMotor.get("back_right"));

        //Motor Set Up
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        front_left.setSlewSpeed(0.2);
        front_right.setSlewSpeed(0.2);
        back_left.setSlewSpeed(0.2);
        back_right.setSlewSpeed(0.2);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //can be.FLOAT if required
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Encoders
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);

        /*
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */
    }

    public void setSlewSpeed(double ss){
        front_left.setSlewSpeed(ss);
        front_right.setSlewSpeed(ss);
        back_left.setSlewSpeed(ss);
        back_right.setSlewSpeed(ss);
    }

    public void setPower(double fl, double fr, double bl, double br){
        front_left.setPower(fl);
        front_right.setPower(fr);
        back_left.setPower(bl);
        back_right.setPower(br);
    }

    public void stop(){
        //Stops Update Threads
        front_left.stop();
        front_right.stop();
        back_left.stop();
        back_right.stop();
    }
}