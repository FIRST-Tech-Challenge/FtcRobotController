package com.bravenatorsrobotics.common.operation;

import com.bravenatorsrobotics.common.core.FtcGamePad;
import com.bravenatorsrobotics.common.core.RobotSpecifications;
import com.bravenatorsrobotics.common.drive.AbstractDrive;

public abstract class TeleopMode<T extends AbstractDrive> extends OperationMode<T> {

    protected final FtcGamePad driverGamePad;
    protected final FtcGamePad operatorGamePad;

    private double startTime = 0.0, endTime = 0.0;
    private double deltaTime = 0.0;

    public TeleopMode(RobotSpecifications specifications) {
        super(specifications);

        driverGamePad = new FtcGamePad();
        operatorGamePad = new FtcGamePad();
    }

    public abstract void OnInitialize();
    public abstract void OnStart();
    public abstract void OnUpdate();
    public abstract void OnStop();

    protected abstract void OnDriverGamePadChange(FtcGamePad gamePad, int button, boolean pressed);
    protected abstract void OnOperatorGamePadChange(FtcGamePad gamePad, int button, boolean pressed);

    public double GetDeltaTime() { return deltaTime; }

    @Override
    public void runOpMode() {
        super.runOpMode();

        telemetry.addData("Operation Mode", "TeleopMode");
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        driverGamePad.initialize("Driver GamePad", gamepad1, this::OnDriverGamePadChange);
        operatorGamePad.initialize("Operator GamePad", gamepad2, this::OnOperatorGamePadChange);

        OnInitialize();

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running...");
        telemetry.update();

        OnStart();

        while(opModeIsActive()) {
            deltaTime = (endTime - startTime) / 1000000000.0; // Convert to accurate millis
            startTime = System.nanoTime();

            // Update
            driverGamePad.update();
            operatorGamePad.update();

            OnUpdate();

            endTime = System.nanoTime();
        }

        telemetry.addData("Status", "Stopping...");
        telemetry.update();

        OnStop();

        telemetry.addData("Status", "Stopped!");
        telemetry.update();
    }

}
