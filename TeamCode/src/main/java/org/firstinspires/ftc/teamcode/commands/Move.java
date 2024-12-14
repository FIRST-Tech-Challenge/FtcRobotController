package org.firstinspires.ftc.teamcode.commands;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.concurrent.TimeUnit;

public class Move extends CommandBase {
    private Drivetrain drivetrain;
    private Timing.Timer timer;
    private boolean isDone = false;
    private double movePower;

    public Move(Drivetrain drivetrain,double movePower ,long dt) {
        this.drivetrain = drivetrain;
        this.movePower = movePower;
        timer = new Timing.Timer(dt, TimeUnit.MILLISECONDS);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        drivetrain.driveArcade(movePower, 0, 0);

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
