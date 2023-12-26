package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import java.util.Objects;
import java.util.function.DoubleSupplier;

public class MoveArmCommand extends CommandBase {

    ArmSubsystem subsystem;
    DoubleSupplier frontwardSupplier, backwardSupplier;

    public MoveArmCommand(ArmSubsystem subsystem, DoubleSupplier frontwardSupplier, DoubleSupplier backwardSupplier) {
        Objects.requireNonNull(subsystem);
        this.subsystem = subsystem;
        this.frontwardSupplier = frontwardSupplier;
        this.backwardSupplier = backwardSupplier;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {

        double power = frontwardSupplier.getAsDouble()-backwardSupplier.getAsDouble();

        if(power == 0) {
            subsystem.haltArm();
        } else {
            subsystem.manualMoveArm(power);
        }
    }
}
