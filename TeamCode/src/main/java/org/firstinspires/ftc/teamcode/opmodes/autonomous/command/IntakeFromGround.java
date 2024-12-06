package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;

public class IntakeFromGround extends SounderBotCommandBase {

    public IntakeFromGround(RollingIntake intake, DeliveryPivot pivot) {
        this.intake = intake;
        this.pivot = pivot;
    }

    RollingIntake intake;

    DeliveryPivot pivot;

    @Override
    public void initialize() {
        intake.IntakeInAuto();
        sleep(300);
        pivot.getMotor().set(-.3);
    }

    @Override
    protected boolean isTargetReached() {
        return intake.IsSampleIntaken() || DeliveryPivot.recordedPosition < DeliveryPivot.IntakePositionFromStart - 500;
    }

    @Override
    public void doExecute() {
        if (isTargetReached()) {
            intake.HoldInAuto();
            finished = true;
        }
    }
}
