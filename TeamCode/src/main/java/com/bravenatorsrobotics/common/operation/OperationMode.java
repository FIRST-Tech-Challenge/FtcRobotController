package com.bravenatorsrobotics.common.operation;

import com.bravenatorsrobotics.common.core.BravenatorRuntimeException;
import com.bravenatorsrobotics.common.core.Robot;
import com.bravenatorsrobotics.common.core.RobotSpecifications;
import com.bravenatorsrobotics.common.drive.AbstractDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class OperationMode<T extends AbstractDrive> extends LinearOpMode {

    protected Robot<T> robot = null;

    public final RobotSpecifications specifications;

    public OperationMode(RobotSpecifications specifications) {
        this.specifications = specifications;
    }

    private void RunFrameworkChecks() {
        // Max Velocity Check
        if (this.specifications.useVelocity) {
            if(this.specifications.maxVelocity == -1) {
                throw new BravenatorRuntimeException("You must specify the robot's velocity in the robot specifications " +
                        "if \"useVelocity\" is enabled!");
            } else if(this.specifications.maxVelocity <= 0) {
                throw new BravenatorRuntimeException("You must specify a robot's velocity to an integer greater than 0!");
            }
        }
    }

    @Override
    public void runOpMode() {

        if(specifications.debugModeEnabled) {
            telemetry.addLine("Debug Mode Enabled!");
            telemetry.update();
        }

        // Make sure the robot specifications are set-up correctly.
        RunFrameworkChecks();

        // TODO: Salvage the robot class
//        if(OperationMode.robot == null)
        this.robot = new Robot<>(this, specifications);
    }
}