package org.firstinspires.ftc.teamcode.commandBased.commands._rr;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandBased.subsystems.AutoDrivetrainSubsystem;

public class TurnCommand extends CommandBase {

    private final AutoDrivetrainSubsystem drive;
    private final double angle;

    public TurnCommand(AutoDrivetrainSubsystem drive, double angle) {
        this.drive = drive;
        this.angle = angle;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.turn(angle);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }

}