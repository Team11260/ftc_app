package org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.framework.AbstractOpMode;

import static org.firstinspires.ftc.teamcode.framework.AbstractOpMode.delay;
import static org.firstinspires.ftc.teamcode.framework.AbstractOpMode.isOpModeActive;


public class IMU {
    private ElapsedTime GyroTimeout;
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;

    public IMU(HardwareMap hwMap){
        GyroTimeout = new ElapsedTime();

        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.mode = BNO055IMU.SensorMode.IMU;

        delay(100);
        imu = hwMap.get(BNO055IMU.class, "imu");


        AbstractOpMode.getTelemetry().addData("1) IMU is init start.");
        AbstractOpMode.getTelemetry().update();

        imu.initialize(parameters);

        AbstractOpMode.getTelemetry().addData("2) IMU is wait.");
        AbstractOpMode.getTelemetry().update();

        GyroTimeout.reset();
        while (!imu.isGyroCalibrated() && ( GyroTimeout.milliseconds() <= 1000 )&& isOpModeActive())
        {
            // do nothing, but wait
        }

        AbstractOpMode.getTelemetry().addData("3) IMU is initialized.");
        AbstractOpMode.getTelemetry().update();
    }

    public double getHeading(){
        Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angle.firstAngle;
    }

    public void resetAngleToZero() {
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()&& isOpModeActive());
    }
}
