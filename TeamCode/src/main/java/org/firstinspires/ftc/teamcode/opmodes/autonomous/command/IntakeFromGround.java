package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;

public class IntakeFromGround extends SounderBotCommandBase {

    public IntakeFromGround(RollingIntake intake, DeliveryPivot pivot, Telemetry telemetry) {
        super(2000);
        this.intake = intake;
        this.pivot = pivot;
    }

    RollingIntake intake;

    DeliveryPivot pivot;

    Telemetry telemetry;

    @Override
    public void initialize() {
        intake.IntakeInAuto();
        sleep(200);
    }

    @Override
    protected boolean isTargetReached() {
        return intake.IsSampleIntaken();
    }

    @Override
    public void doExecute() {
        pivot.getMotor().set(-.4);

        if (isTargetReached()) {
            intake.HoldInAuto();
            finished = true;

            pivot.getMotor().set(0);
        }
    }
}
