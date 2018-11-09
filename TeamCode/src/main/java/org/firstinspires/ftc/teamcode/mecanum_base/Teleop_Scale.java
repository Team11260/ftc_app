package org.firstinspires.ftc.teamcode.mecanum_base;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.framework.AbstractTeleop;
import org.firstinspires.ftc.teamcode.mecanum_base.hardware.HardwareDevices;

@TeleOp(name="Mecanum_TeleOp_ColorSensor", group="Utility")
@Disabled

public class Teleop_Scale extends AbstractTeleop {

    private HardwareDevices robot;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    int relativeLayoutId;
    View relativeLayout;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;


    @Override
    public void Init() {
        robot = new HardwareDevices();

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());

        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    }

    @Override
    public void Loop() {


            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            if (gamepad1.a==true){
                sensorColor.enableLed(true);

            }
            if (gamepad1.b==true){
                sensorColor.enableLed(false);
            }
            telemetry.addData("Alpha= " + sensorColor.alpha());
            telemetry.addData("Red=  " + sensorColor.red());
            telemetry.addData("Green= " + sensorColor.green());
            telemetry.addData("Blue= " + sensorColor.blue());
            telemetry.addData("Hue= " + hsvValues[0]);
            telemetry.addData("Distance= " + sensorDistance.getDistance(DistanceUnit.CM));
            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });







        robot.updateDrive();
        telemetry.update();


    }

    @Override
    public void Stop() {
        robot.stop();

	// Set the panel back to the default color
		relativeLayout.post(new Runnable() {
		    public void run() {
		        relativeLayout.setBackgroundColor(Color.WHITE);
		    }
		});
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
}
