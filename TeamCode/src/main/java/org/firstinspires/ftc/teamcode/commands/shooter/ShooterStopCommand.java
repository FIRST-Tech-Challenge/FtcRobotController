package org.firstinspires.ftc.teamcode.commands.shooter;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;
public class ShooterStopCommand extends Command {
    public ShooterSubsystem subsystem;
    public ShooterStopCommand(ShooterSubsystem sub){
        addRequirements(sub);
        subsystem = sub;
    }


    @Override
    public void execute() {
        System.out.print("stopshooter");
        subsystem.setVelocity(0);
    }

}
