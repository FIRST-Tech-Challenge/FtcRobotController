package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

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
        Log.i(CommonConstants.DEBUG_TAG, "Out take running");
        Log.i(CommonConstants.DEBUG_TAG, "Sample taken: " + intake.IsSampleIntaken());
        intake.IntakeInAuto();
        if (isTargetReached()) {
            intake.Hold();
            finished = true;
        }
    }
}
