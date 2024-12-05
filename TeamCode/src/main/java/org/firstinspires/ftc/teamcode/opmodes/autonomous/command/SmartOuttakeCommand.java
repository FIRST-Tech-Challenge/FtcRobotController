package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;

public class SmartOuttakeCommand extends SounderBotCommandBase {

    public SmartOuttakeCommand(RollingIntake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    RollingIntake intake;

    @Override
    public void initialize() {
    }

    @Override
    protected boolean isTargetReached() {
        return !intake.IsSampleIntaken();
    }

    @Override
    public void doExecute() {
        intake.Intake();
    }

    @Override
    public boolean isFinished() {
        if(!intake.IsSampleIntaken()) {
            intake.Hold();
            return true;
        }

        return false;
    }
}
