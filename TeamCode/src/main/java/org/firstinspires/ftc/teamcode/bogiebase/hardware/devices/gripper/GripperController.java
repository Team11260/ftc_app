package org.firstinspires.ftc.teamcode.bogiebase.hardware.devices.gripper;

import org.firstinspires.ftc.teamcode.framework.util.SubsystemController;

public class GripperController extends SubsystemController {

    Gripper gripper;

    private boolean gripped = false,extended = false;

    @Override
    public void update() throws Exception {

    }

    public GripperController() {

        gripper = new Gripper();
    }

    @Override
    public void stop() {

    }
}
