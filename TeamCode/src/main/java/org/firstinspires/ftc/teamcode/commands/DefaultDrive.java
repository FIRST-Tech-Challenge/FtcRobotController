package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {
    private Drivetrain drivetrain;

    private DoubleSupplier strafe, fwd, rot;
    public DefaultDrive(Drivetrain drivetrain, DoubleSupplier strafe, DoubleSupplier fwd, DoubleSupplier rot) {
        this.drivetrain = drivetrain;

        this.fwd = fwd;
        this.rot = rot;
        this.strafe = strafe;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.driveFieldCentric(strafe.getAsDouble() * 0.7, fwd.getAsDouble() * 0.7, rot.getAsDouble() * 0.5);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
