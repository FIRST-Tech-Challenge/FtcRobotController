package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;

public class WhileSamplePresent extends SounderBotCommandBase {

    public WhileSamplePresent(RollingIntake intake, Telemetry telemetry, int timeout) {
        super(timeout);

        this.rollingIntake = intake;
        this.telemetry = telemetry;
    }

    RollingIntake rollingIntake;
    Telemetry telemetry;

    public void initialize() {
    }

    @Override
    protected boolean isTargetReached() {
        return !rollingIntake.IsSampleIntaken();
    }

    @Override
    public void doExecute() {
    }
}
