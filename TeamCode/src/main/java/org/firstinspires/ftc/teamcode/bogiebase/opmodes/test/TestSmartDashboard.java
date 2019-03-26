package org.firstinspires.ftc.teamcode.bogiebase.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAutonNew;
import org.upacreekrobotics.dashboard.Dashboard;

@Autonomous(name = "Test Smart Dashboard", group = "New")
//@Disabled

public class TestSmartDashboard extends AbstractAutonNew {


    @Override
    public void RegisterStates() {

    }

    @Override
    public void Init() {
        Dashboard.getInstance().getSmartDashboard().putValue("Test", 1234);
    }

    @Override
    public void Run() {

    }
}
