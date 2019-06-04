package org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.imu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractOpMode;

public class IMU implements Runnable {

    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;

    private ElapsedTime GyroTimeOut;

    private boolean newHeadingValue = false;
    private boolean newAccelerationValue = false;
    private boolean newVelocityValue = false;
    private double heading = 0;
    private double accelerationX = 0;
    private double velocityX = 0;

    private double lastHeading = 0;
    private double absoluteHeadingCorrection = 0;

    private final Object headingLock = new Object();
    private final Object accelerationLock = new Object();
    private final Object velocityLock = new Object();

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
            synchronized (headingLock) {
                if(newHeadingValue) {
                    newHeadingValue = false;
                    return heading;
                }
            }
        }

        return 0;
    }

    public double getXAcceleration() {
        while (AbstractOpMode.isOpModeActive()) {
            synchronized (accelerationLock) {
                if(newAccelerationValue) {
                    newAccelerationValue = false;
                    return accelerationX;
                }
            }
        }

        return 0;
    }

    public double getXVelocity() {
        while (AbstractOpMode.isOpModeActive()) {
            synchronized (velocityLock) {
                if(newVelocityValue) {
                    newVelocityValue = false;
                    return velocityX;
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
        Acceleration acceleration;
        Velocity velocity;
        while (AbstractOpMode.isOpModeActive()) {
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            acceleration = imu.getAcceleration();
            velocity = imu.getVelocity();

            synchronized (headingLock) {
                heading = angle.firstAngle;
                newHeadingValue = true;
            }

            synchronized (accelerationLock) {
                accelerationX = acceleration.yAccel;
                newAccelerationValue = true;
            }

            synchronized (velocityLock) {
                velocityX = velocity.xVeloc;
                newVelocityValue = true;
            }

            if(heading > 90 && lastHeading < -90) absoluteHeadingCorrection -= 360;
            if(heading < -90 && lastHeading > 90) absoluteHeadingCorrection += 360;

            lastHeading = heading;
        }
    }
}