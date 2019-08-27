package org.firstinspires.ftc.teamcode.skidsteer_base.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.framework.AbstractTeleop;
import org.firstinspires.ftc.teamcode.framework.userhardware.MyNumberRound;

/**
 * Created by user on 9/30/2018.
 */

public class MyPotentiometer {

    // ************************
    //   Potentiometer Readings
    // ************************
    private AnalogInput myPot;
    private MyNumberRound myLocalRound;

    private HardwareMap hwMap;

    //The initializer
    public MyPotentiometer(){
        myLocalRound = new MyNumberRound();
        // This is an odd way to fix the hardware map issue
        //    where the normal hardware map getter does not work.
        hwMap = AbstractTeleop.getOpModeInstance().hardwareMap;
        myPot = hwMap.analogInput.get("Pot"); // Put this string on phone
    }

    public double getMyPotVoltageRaw(){

        return ( myPot.getVoltage()); // 0-3.3
    }

    public double getMyPotVoltageFrom0To33By1DecBy1Dec(){
        double myRange0To33; // 0 - 3.3 by 1 decimal place
        myRange0To33 = myLocalRound.roundDouble(( myPot.getVoltage()),1);
        return (myRange0To33);

    }

    public double getMyPotVoltageFrom0To10(){
        double myRangeUpToTen; // 0 - 10
        myRangeUpToTen = myLocalRound.roundDouble((10/3.3) * ( myPot.getVoltage()),0);
        return (myRangeUpToTen);

    }

    public double getMyPotVoltageFrom0To10By1Dec(){
        double myRangeUpToTen_1dec; // 0.0 - 10.0
        myRangeUpToTen_1dec = myLocalRound.roundDouble((10/3.3) * ( myPot.getVoltage()),1);
        return ( myRangeUpToTen_1dec);

    }



}
