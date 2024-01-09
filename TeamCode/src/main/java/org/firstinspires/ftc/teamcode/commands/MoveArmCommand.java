package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import java.util.Objects;
import java.util.function.DoubleSupplier;
import static org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem.ArmPosition.*;

public class MoveArmCommand extends CommandBase {

    ArmSubsystem subsystem;
    DoubleSupplier frontwardSupplier, backwardSupplier, moveArmToZero, moveArmToBoard;

    public MoveArmCommand(ArmSubsystem subsystem, DoubleSupplier frontwardSupplier,
                          DoubleSupplier backwardSupplier) {
        Objects.requireNonNull(subsystem);
        this.subsystem = subsystem;
        this.frontwardSupplier = frontwardSupplier;
        this.backwardSupplier = backwardSupplier;

        final DoubleSupplier ZERO = () -> 0.0;

        this.moveArmToZero = ZERO;
        this.moveArmToBoard = ZERO;

        addRequirements(subsystem);
    }

    public MoveArmCommand(ArmSubsystem subsystem, DoubleSupplier frontwardSupplier,
                          DoubleSupplier backwardSupplier, DoubleSupplier moveArmToZero,
                          DoubleSupplier moveArmToBoard) {
        Objects.requireNonNull(subsystem);

        this.subsystem = subsystem;
        this.frontwardSupplier = frontwardSupplier;
        this.backwardSupplier = backwardSupplier;
        this.moveArmToZero = moveArmToZero;
        this.moveArmToBoard = moveArmToBoard;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {

        double power = frontwardSupplier.getAsDouble()-backwardSupplier.getAsDouble();

        if (moveArmToZero.getAsDouble() != 0) {
            subsystem.positionMoveArm(ZERO);
            return;
        } else if (moveArmToBoard.getAsDouble() != 0) {
            subsystem.positionMoveArm(BOARD);
            return;
        }

        if(power == 0) {
            subsystem.haltArm();
        } else {
            subsystem.manualMoveArm(power);
        }
    }
}
