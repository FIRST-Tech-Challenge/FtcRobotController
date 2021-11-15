package com.bravenatorsrobotics.common.operation;

import com.bravenatorsrobotics.common.core.RobotSpecifications;
import com.bravenatorsrobotics.common.drive.AbstractDrive;

public abstract class AutonomousMode<T extends AbstractDrive> extends OperationMode<T> {

    public AutonomousMode(RobotSpecifications specifications) { super(specifications); }

    public abstract void OnInitialize();
    public abstract void OnStart();
    public abstract void OnStop();

    @Override
    public void runOpMode() {
        super.runOpMode();

        telemetry.addData("Operation Mode", "AutonomousMode");
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        super.robot.Reset();

        OnInitialize();

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running...");
        telemetry.update();

        OnStart();

        telemetry.addData("Status", "Stopping...");
        telemetry.update();

        OnStop();

        telemetry.addData("Status", "Stopped!");
        telemetry.update();
    }
}
