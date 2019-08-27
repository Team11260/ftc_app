package org.firstinspires.ftc.teamcode.omni_base.hardware.devices.relic;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.framework.userhardware.outputs.SlewDcMotor;

public class Relic {

    private Servo claw;
    private SlewDcMotor lift;
    private SlewDcMotor extend;

    public Relic(HardwareMap hwMap){
        claw = hwMap.servo.get("relic_claw_servo");

        lift = new SlewDcMotor(hwMap.dcMotor.get("relic_lift_motor"));
        extend = new SlewDcMotor(hwMap.dcMotor.get("relic_extend_motor"));

        claw.setPosition(0.7);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setLiftPosition(int position,double power){
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(position);
        lift.setPower(power);
    }

    public int getLiftTargetPosition(){
        return lift.getTargetPosition();
    }

    public void setExtendPosition(int position,double power){
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setTargetPosition(position);
        extend.setPower(power);
    }

    public int getExtendTargetPosition(){
        return extend.getTargetPosition();
    }

    public void setClawPosition(double position){
        claw.setPosition(position);
    }
}
