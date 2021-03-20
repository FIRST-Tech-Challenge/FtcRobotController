package org.firstinspires.ftc.teamcode.commands.index;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.IndexSubsystem;

public class IndexPivotUpCommand extends WaitCommand {
    public IndexSubsystem indexSubsystem;
    public IndexPivotUpCommand(IndexSubsystem subsystem){
        //COOLDOWN
        super(1);
        addRequirements(subsystem);
        indexSubsystem = subsystem;
    }

    @Override
    public void execute() {
        indexSubsystem.raiseToShooter();
    }
}
