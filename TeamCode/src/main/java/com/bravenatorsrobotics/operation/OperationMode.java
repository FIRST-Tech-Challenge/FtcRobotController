package com.bravenatorsrobotics.operation;

import com.bravenatorsrobotics.core.Robot;
import com.bravenatorsrobotics.core.RobotSpecifications;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class OperationMode extends LinearOpMode {

    protected Robot robot = null;

    protected final RobotSpecifications specifications;

    public OperationMode(RobotSpecifications specifications) {
        this.specifications = specifications;
    }

    @Override
    public void runOpMode() {
        // TODO: Salvage the robot class
//        if(OperationMode.robot == null)
        this.robot = new Robot(this, specifications);
    }
}