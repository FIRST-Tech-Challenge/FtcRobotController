package com.bravenatorsrobotics.operation;

import com.bravenatorsrobotics.core.Robot;
import com.bravenatorsrobotics.core.RobotSpecifications;
import com.bravenatorsrobotics.drive.AbstractDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class OperationMode<T extends AbstractDrive> extends LinearOpMode {

    protected Robot<T> robot = null;

    protected final RobotSpecifications specifications;

    public OperationMode(RobotSpecifications specifications) {
        this.specifications = specifications;
    }

    @Override
    public void runOpMode() {
        // TODO: Salvage the robot class
//        if(OperationMode.robot == null)
        this.robot = new Robot<T>(this, specifications);
    }
}