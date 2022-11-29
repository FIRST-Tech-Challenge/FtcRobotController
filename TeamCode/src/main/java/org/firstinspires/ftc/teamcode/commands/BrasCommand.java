package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class BrasCommand extends CommandBase {

    private final BrasSubsystem mBrasSubsystem;
    private final Telemetry mTelemetry;
    private final Gamepad mGamepad;

    public BrasCommand(Telemetry telemetry, BrasSubsystem brasSubsystem, Gamepad gamepad){
        mTelemetry = telemetry;
        mBrasSubsystem = brasSubsystem;
        mGamepad = gamepad;

        addRequirements(brasSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        mBrasSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
