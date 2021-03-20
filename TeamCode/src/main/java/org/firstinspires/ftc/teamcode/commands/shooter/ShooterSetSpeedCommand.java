package org.firstinspires.ftc.teamcode.commands.shooter;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;
public class ShooterSetSpeedCommand extends Command {
    public ShooterSubsystem subsystem;
    public DoubleSupplier supplier;
    public ShooterSetSpeedCommand(ShooterSubsystem sub, DoubleSupplier sup){
        addRequirements(sub);
        subsystem = sub;
        supplier = sup;
    }

    @Override
    public void execute() {
        subsystem.setVelocity(supplier.getAsDouble()*100);
    }

    @Override
    public void end(boolean cancel) {
        if(cancel) subsystem.stop();
    }
}
