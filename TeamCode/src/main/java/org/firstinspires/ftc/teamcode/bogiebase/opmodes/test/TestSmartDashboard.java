package org.firstinspires.ftc.teamcode.bogiebase.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAutonNew;
import org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry;
import org.firstinspires.ftc.teamcode.framework.util.State;
import org.upacreekrobotics.dashboard.Dashboard;

@Autonomous(name = "Test Smart Dashboard", group = "New")
@Disabled

public class TestSmartDashboard extends AbstractAutonNew {

    Dashboard.smartdashboard smartdashboard;

    @Override
    public void RegisterStates() {
        addState(new State("button", "start", () ->{
            while (opModeIsActive()) smartdashboard.putBoolean("Button Pressed", smartdashboard.getButton("Run"));
        }));
        addState(new State("slider", "start", () ->{
            while (opModeIsActive()) smartdashboard.putValue("Slider Value", smartdashboard.getSlider("P"));
        }));
    }

    @Override
    public void Init() {
        smartdashboard = Dashboard.getInstance().getSmartDashboard();

        smartdashboard.putValue("Test", 1234);
        smartdashboard.putButton("Run");
        smartdashboard.putInput("Value");
        smartdashboard.putSlider("P", 0, 100);
    }

    @Override
    public void Run() {
        telemetry.addData(DoubleTelemetry.LogMode.INFO, smartdashboard.getValue("Test"));
        telemetry.addData(DoubleTelemetry.LogMode.INFO, smartdashboard.getInput("Value"));
        telemetry.update();

        for(int i=0; i<100; i++) {
            smartdashboard.putValue("Test", i);
            smartdashboard.putBoolean("Even", i%2==0);
            telemetry.addData(DoubleTelemetry.LogMode.INFO, i + " " + smartdashboard.getBoolean("Even"));
            delay(500);
            if(!opModeIsActive()) break;
        }
    }
}
