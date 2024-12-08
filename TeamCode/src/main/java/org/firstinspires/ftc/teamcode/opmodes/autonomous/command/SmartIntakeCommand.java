package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;

import java.util.Timer;
import java.util.TimerTask;

public class SmartIntakeCommand extends SounderBotCommandBase {

    public SmartIntakeCommand(RollingIntake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    RollingIntake intake;

    @Override
    public void initialize() {
    }

    @Override
    protected boolean isTargetReached() {
        return intake.IsSampleIntaken();
    }

    @Override
    public void doExecute() {
        intake.IntakeInAuto();
        if (isTargetReached()) {
            finished = true;
        }
    }

}
