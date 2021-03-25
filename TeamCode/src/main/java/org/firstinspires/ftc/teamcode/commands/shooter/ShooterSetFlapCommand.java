package org.firstinspires.ftc.teamcode.commands.shooter;

import com.technototes.library.command.Command;
import com.technototes.library.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class ShooterSetFlapCommand extends WaitCommand {
    public ShooterSubsystem subsystem;
    public DoubleSupplier supplier;
    public ShooterSetFlapCommand(ShooterSubsystem sub, DoubleSupplier sup) {
        super(0.1);
        subsystem = sub;
        supplier = sup;
    }

    @Override
    public void execute() {
       subsystem.setFlapPosition(supplier.getAsDouble());
    }
}
