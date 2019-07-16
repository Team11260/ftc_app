package org.firstinspires.ftc.teamcode.framework.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.imu.IMU;
import org.firstinspires.ftc.teamcode.framework.userhardware.outputs.SlewDcMotor;

public class HardwareMapEx extends HardwareMap {

    public HardwareMapEx(HardwareMap hardwareMap) {
        super(hardwareMap.appContext);
    }

    public DcMotor getDcMotor(String name){
        return dcMotor.get(name);
    }

    public SlewDcMotor getSlewDcMotor(String name){
        return new SlewDcMotor(dcMotor.get(name));
    }

    public Servo getServo(String name){
        return servo.get(name);
    }

    public DcMotorSimple getDcMotorSimple(String name){
        return get(DcMotorSimple.class, name);
    }

    public IMU getImu(String name) {
        return new IMU(get(BNO055IMU.class, name));
    }
}
