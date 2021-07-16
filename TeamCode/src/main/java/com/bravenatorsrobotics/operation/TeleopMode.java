package com.bravenatorsrobotics.operation;

import com.bravenatorsrobotics.core.RobotSpecifications;

public abstract class TeleopMode extends OperationMode {

    public TeleopMode(RobotSpecifications specifications) { super(specifications); }

    public abstract void OnInitialize();
    public abstract void OnUpdate();
    public abstract void OnStop();

    @Override
    public void runOpMode() {
        super.runOpMode();

        telemetry.addData("Operation Mode", "TeleopMode");
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        OnInitialize();

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running...");
        telemetry.update();

        while(opModeIsActive()) {
            OnUpdate();
        }

        telemetry.addData("Status", "Stopping...");
        telemetry.update();

        OnStop();

        telemetry.addData("Status", "Stopped!");
        telemetry.update();
    }

}
