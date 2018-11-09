package org.firstinspires.ftc.teamcode.skidsteer_base;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.framework.AbstractTeleop;
import org.firstinspires.ftc.teamcode.framework.userHardware.MyNumberRound;
import org.firstinspires.ftc.teamcode.skidsteer_base.hardware.HardwareDevices;
import org.firstinspires.ftc.teamcode.skidsteer_base.hardware.MyPotentiometer;

import java.text.DecimalFormat;
import java.util.Locale;

@TeleOp(name="SkidSteer_Pot_0236pm", group="New")
//@Disabled

public class Teleop_Pot extends AbstractTeleop {

    private HardwareDevices hardware;

    DecimalFormat decfmt1;

    // **************************
    //   Gamepad1.A with counter
    // **************************
    int counterA;
    boolean gamepadMode = false;
    boolean lastgampadAPressed = false;

    // ************************
    //   Potentiometer Readings
    // ************************
    MyPotentiometer myLocalPot;
    double myDoublePotValue;
    double myDoublePotValue2;
    double myRangeUpToTen;
    double myRangeUpToTen_1dec;
    // ********************************
    //   Methods (functions) are below
    // ********************************

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;


    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array. QUESTION HOW IS THE REFERENCE USED HERE???????
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    int relativeLayoutId;
    View relativeLayout;


    MyNumberRound myLocalRound;



    @Override
    public void Init() {
        hardware = new HardwareDevices();

        myLocalRound = new MyNumberRound();
        myLocalPot = new MyPotentiometer();

        counterA = 0;
        decfmt1 = new DecimalFormat("#.#");

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        //This needs to be here not in the declaration, or else you will get an INIT error when you
        //start the program
        relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    }

    @Override
    public void Loop() {

        //if((gamepad1.a == true) && (lastgampadAPressed == false)) {
            //counterA = counterA + 1;
        //}
        //lastgampadAPressed = gamepad1.a;

        hardware.updateDrive();

        myDoublePotValue = myLocalPot.getMyPotVoltageRaw();  // will return any value from 0 to 3.3
        myDoublePotValue2 = myLocalPot.getMyPotVoltageFrom0To33By1DecBy1Dec();     //myLocalRound.round(myDoublePotValue, 1);
        myRangeUpToTen = myLocalPot.getMyPotVoltageFrom0To10();          //myLocalRound.round((10/3.3) * (myDoublePotValue2),0);
  //      myRangeUpToTen_1dec = myLocalRound.roundDouble((10/3.3) * (myDoublePotValue2),1);
        myRangeUpToTen_1dec = myLocalPot.getMyPotVoltageFrom0To10By1Dec();

        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);


        // send the info back to driver station using telemetry function.

        telemetry.addData("Distance (cm)" +
                String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha "+ sensorColor.alpha());
        telemetry.addData("Red  "+ sensorColor.red());
        telemetry.addData("Green "+ sensorColor.green());
        telemetry.addData("Blue "+ sensorColor.blue());
        telemetry.addData("Hue "+ hsvValues[0]);

        telemetry.addData("myPot = " + myDoublePotValue);
        telemetry.addData("myPot fixed = " + decfmt1.format( myDoublePotValue));
        telemetry.addData("myPot Rounded = " + myDoublePotValue2);
        telemetry.addData("myPot up to 10 = " + myRangeUpToTen);
        telemetry.addData("myPot up to 10 with decimal = " + myRangeUpToTen_1dec);

        telemetry.addData("counterA = " + counterA);// displays counterA value but not the string,
        // instead there is a colon
        telemetry.update();


        /*
        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });

        */


    } // End of the loop section

    @Override
    public void Stop() {
        hardware.stop();

        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });

    }

    @Override
    public void lsy_change(double lsy) {
        //forward and backward
        hardware.setDriveY(lsy);
    }

    @Override
    public void rsx_change(double rsx) {
        //left and right turn

        hardware.setDriveZ(rsx);
    }

    @Override
    public  void a_down(){
        counterA = counterA + 1;

        // display doesnt work
        //telemetry.addData("counter = " + counterA);
       // telemetry.update();
    }

}