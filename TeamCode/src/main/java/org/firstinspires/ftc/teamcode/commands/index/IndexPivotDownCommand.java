package org.firstinspires.ftc.teamcode.commands.index;

import com.technototes.library.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.IndexSubsystem;

public class IndexPivotDownCommand extends WaitCommand {
    public IndexSubsystem indexSubsystem;
    public IndexPivotDownCommand(IndexSubsystem subsystem){
        //COOLDOWN
        super(1);
        //addRequirements(subsystem);
        indexSubsystem = subsystem;
    }

    @Override
    public void init() {
        indexSubsystem.lowerToIntake();
    }
}
