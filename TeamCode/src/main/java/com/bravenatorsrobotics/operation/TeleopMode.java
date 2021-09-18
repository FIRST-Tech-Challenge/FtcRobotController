package com.bravenatorsrobotics.operation;

import com.bravenatorsrobotics.core.FtcGamePad;
import com.bravenatorsrobotics.core.RobotSpecifications;
import com.bravenatorsrobotics.drive.AbstractDrive;

public abstract class TeleopMode<T extends AbstractDrive> extends OperationMode<T> {

    protected final FtcGamePad driverGamePad;
    protected final FtcGamePad operatorGamePad;

    public TeleopMode(RobotSpecifications specifications) {
        super(specifications);

        driverGamePad = new FtcGamePad();
        operatorGamePad = new FtcGamePad();
    }

    public abstract void OnInitialize();
    public abstract void OnUpdate();
    public abstract void OnStop();

    protected abstract void OnDriverGamePadChange(FtcGamePad gamePad, int button, boolean pressed);
    protected abstract void OnOperatorGamePadChange(FtcGamePad gamePad, int button, boolean pressed);

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

        while(opModeIsActive()) {
            driverGamePad.update();
            operatorGamePad.update();

            OnUpdate();
        }

        telemetry.addData("Status", "Stopping...");
        telemetry.update();

        OnStop();

        telemetry.addData("Status", "Stopped!");
        telemetry.update();
    }

}
