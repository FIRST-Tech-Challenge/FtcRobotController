package com.bravenatorsrobotics.operation;

import com.bravenatorsrobotics.core.Robot;
import com.bravenatorsrobotics.core.RobotSpecifications;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class OperationMode extends LinearOpMode {

    protected static Robot robot = null;

    protected final RobotSpecifications specifications;

    public OperationMode(RobotSpecifications specifications) {
        this.specifications = specifications;

        if(OperationMode.robot == null)
            OperationMode.robot = new Robot(hardwareMap, specifications);
    }

}