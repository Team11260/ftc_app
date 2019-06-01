package org.firstinspires.ftc.teamcode.examplebase.hardware.devices.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.IMU;
import org.firstinspires.ftc.teamcode.framework.userhardware.outputs.SlewDcMotor;

public class Drive {

    private SlewDcMotor slewDcMotorLeft;
    private SlewDcMotor slewDcMotorRight;
    private IMU imu;

    public Drive(HardwareMap hardwareMap) {
        slewDcMotorLeft = new SlewDcMotor(hardwareMap.dcMotor.get("left"));
        slewDcMotorRight = new SlewDcMotor(hardwareMap.dcMotor.get("right"));

        slewDcMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = new IMU(hardwareMap);
    }

    public void setPower(double leftPower, double rightPower) {
        slewDcMotorLeft.setPower(leftPower);
        slewDcMotorRight.setPower(rightPower);
    }

    public double getHeading(){
        return imu.getHeading();
    }

    public void resetHeading () {
        imu.resetAngleToZero();
    }

    public double getLeftCurrentPosition(){
        return slewDcMotorLeft.getCurrentPosition();
    }

    public double getRightCurrentPosition(){
        return slewDcMotorRight.getCurrentPosition();
    }

    public void setMode(DcMotor.RunMode mode){
        slewDcMotorLeft.setMode(mode);
        slewDcMotorRight.setMode(mode);
    }

    public void stop() {

    }
}