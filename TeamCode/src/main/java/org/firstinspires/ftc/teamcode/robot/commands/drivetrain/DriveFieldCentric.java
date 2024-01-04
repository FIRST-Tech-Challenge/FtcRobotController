package org.firstinspires.ftc.teamcode.robot.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveFieldCentric extends CommandBase {
    private final DriveSubsystem drive;
    private DoubleSupplier y;
    private DoubleSupplier x;
    private DoubleSupplier rx;
    private BooleanSupplier slowMode;

    private final double SLOW_MODE_SPEED = 0.5;

    public DriveFieldCentric(
            DriveSubsystem subsystem,
            DoubleSupplier y,
            DoubleSupplier x,
            DoubleSupplier rx,
            BooleanSupplier slowMode)
    {
        this.y = y;
        this.x = x;
        this.rx = rx;
        this.slowMode = slowMode;
        drive = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if(slowMode.getAsBoolean())
            drive.runFeildCentric(y.getAsDouble(),x.getAsDouble(),rx.getAsDouble(), SLOW_MODE_SPEED);
        else
            drive.runFeildCentric(y.getAsDouble(),x.getAsDouble(),rx.getAsDouble(), 1.0);
    }
}