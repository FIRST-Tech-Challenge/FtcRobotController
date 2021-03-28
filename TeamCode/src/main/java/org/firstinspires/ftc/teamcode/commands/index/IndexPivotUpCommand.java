package org.firstinspires.ftc.teamcode.commands.index;

import com.technototes.library.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.IndexSubsystem;

public class IndexPivotUpCommand extends WaitCommand {
    public IndexSubsystem indexSubsystem;
    public IndexPivotUpCommand(IndexSubsystem subsystem){
        //COOLDOWN
        super(0.2);
        addRequirements(subsystem);
        indexSubsystem = subsystem;
    }

    @Override
    public void init() {
        indexSubsystem.raiseToShooter();
    }
}
