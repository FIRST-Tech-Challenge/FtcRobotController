package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;

import java.util.Timer;
import java.util.TimerTask;

public class OuttakeCommand extends SounderBotCommandBase {

    private static final long TIME_OUT = 1000; // 1 sec
    boolean expired = false;
    Timer timer = new Timer();
    public OuttakeCommand(RollingIntake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    RollingIntake intake;

    @Override
    protected boolean isTargetReached() {
        return expired;
    }

    @Override
    public void initialize() {
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                expired = true;
            }
        }, TIME_OUT);
    }

    @Override
    public void doExecute() {
        if (isTargetReached()) {
            intake.HoldInAuto();
            expired = true;
            finished = true;
        } else {
            intake.OuttakeInAuto();
        }
    }
}
