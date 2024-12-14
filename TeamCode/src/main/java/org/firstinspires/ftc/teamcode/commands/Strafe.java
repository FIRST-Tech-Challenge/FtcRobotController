package org.firstinspires.ftc.teamcode.commands;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.concurrent.TimeUnit;

public class Strafe extends CommandBase {
    private Drivetrain drivetrain;
    private Timing.Timer timer;
    private double strafePower;
    private boolean isDone = false;

    public Strafe(Drivetrain drivetrain, double strafePower, long dt) {
        this.drivetrain = drivetrain;
        timer = new Timing.Timer(dt, TimeUnit.MILLISECONDS);
        this.strafePower = strafePower;
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        drivetrain.driveArcade(0, strafePower, 0);
        if (timer.done()) {
            drivetrain.driveArcade(0,0,0);
            isDone = true;
        }
    }

    @Override
    public boolean isFinished() {
        return this.isDone;
    }

}
