package org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractOpMode;

public class IMU implements Runnable {

    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;

    private ElapsedTime GyroTimeOut;

    private boolean newValue = false;
    private double heading = 0;

    private double lastHeading = 0;
    private double absoluteHeadingCorrection = 0;

    private final Object lock = new Object();

    public IMU(HardwareMap hwMap) {
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.mode = BNO055IMU.SensorMode.IMU;

        imu = hwMap.get(BNO055IMU.class, "imu");

        AbstractOpMode.telemetry.addData("IMU initializing: " + imu.toString());

        ThreadPool.getDefault().submit((Runnable)() -> imu.initialize(parameters));

        new Thread(this).start();
    }

    public double getHeading() {
        while (AbstractOpMode.isOpModeActive()) {
            synchronized (lock) {
                if(newValue) {
                    newValue = false;
                    return heading;
                }
            }
        }

        return 0;
    }

    public double getAbsoluteHeading() {
        return absoluteHeadingCorrection + heading;
    }

    public double getHeadingRadians() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    public double getPitch(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
    }

    public void resetAngleToZero() {
        imu.initialize(parameters);

        while (!imu.isGyroCalibrated() && GyroTimeOut.milliseconds() <= 1000 && AbstractOpMode.isOpModeActive());
    }

    public boolean isGyroCalibrated() {
        return imu.isGyroCalibrated();
    }

    @Override
    public void run() {
        Orientation angle;
        while (AbstractOpMode.isOpModeActive()) {
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            synchronized (lock) {
                heading = angle.firstAngle;
                newValue = true;
            }

            if(heading > 90 && lastHeading < -90) absoluteHeadingCorrection -= 360;
            if(heading < -90 && lastHeading > 90) absoluteHeadingCorrection += 360;

            lastHeading = heading;
        }
    }
}