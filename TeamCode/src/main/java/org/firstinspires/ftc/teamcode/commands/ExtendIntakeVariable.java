package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.function.DoubleSupplier;

public class ExtendIntakeVariable extends CommandBase {
    private Intake intake;
    private DoubleSupplier supplier;

    public ExtendIntakeVariable(Intake intake, DoubleSupplier extension) {
        this.intake = intake;
        this.supplier = extension;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        this.intake.setExt(this.supplier.getAsDouble() + 0.3);
    }

}
