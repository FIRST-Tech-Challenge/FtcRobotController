package com.bravenatorsrobotics.operation;

import com.bravenatorsrobotics.core.RobotSpecifications;

public abstract class TeleopMode extends OperationMode {

    public TeleopMode(RobotSpecifications specifications) { super(specifications); }

    public abstract void OnInitialize();
    public abstract void OnUpdate();
    public abstract void OnEnd();

    @Override
    public void runOpMode() {
        OnInitialize();

        waitForStart();

        while(opModeIsActive()) {
            OnUpdate();
        }

        OnEnd();
    }

}
