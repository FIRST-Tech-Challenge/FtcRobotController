package org.firstinspires.ftc.teamcode.commands.index;

import org.firstinspires.ftc.teamcode.commands.WaitCommand;
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
    public void execute() {
        indexSubsystem.lowerToIntake();
    }
}
