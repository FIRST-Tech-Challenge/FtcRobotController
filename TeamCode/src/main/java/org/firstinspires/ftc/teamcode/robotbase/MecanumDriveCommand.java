package org.firstinspires.ftc.teamcode.robotbase;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class MecanumDriveCommand extends CommandBase {
    private final MecanumDrivePPV2 drivetrain;
    private final DoubleSupplier forwardSpeed, strafeSpeed, turnSpeed, heading, maxSpeed;

    public MecanumDriveCommand(MecanumDrivePPV2 drivetrain, DoubleSupplier forwardSpeed,
                                DoubleSupplier strafeSpeed, DoubleSupplier turnSpeed,
                                DoubleSupplier heading, DoubleSupplier maxSpeed) {
        this.drivetrain = drivetrain;
        this.forwardSpeed = forwardSpeed;
        this.strafeSpeed = strafeSpeed;
        this.turnSpeed = turnSpeed;
        this.heading = heading;
        this.maxSpeed = maxSpeed;
        addRequirements(this.drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(),
                turnSpeed.getAsDouble(), heading.getAsDouble(), maxSpeed.getAsDouble());
    }
}
