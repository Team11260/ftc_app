package org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractTeleop;
import org.firstinspires.ftc.teamcode.framework.userhardware.RoundNumber;

public class MiniPotentiometer {

    // ************************
    //   Potentiometer Readings
    // ************************
    private AnalogInput myPot;
    private RoundNumber myLocalRound;

    private HardwareMap hwMap;

    //The initializer
    public MiniPotentiometer(){
        myLocalRound = new RoundNumber();
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
