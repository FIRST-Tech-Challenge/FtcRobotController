package com.bravenatorsrobotics.operation;

import com.bravenatorsrobotics.core.RobotSpecifications;

public abstract class AutonomousMode extends OperationMode {

    public AutonomousMode(RobotSpecifications specifications) { super(specifications); }

    public abstract void OnInitialize();
    public abstract void OnStart();
    public abstract void OnStop();

    @Override
    public void runOpMode() {
        super.runOpMode();

        OnInitialize();

        waitForStart();

        OnStart();
        OnStop();
    }
}
