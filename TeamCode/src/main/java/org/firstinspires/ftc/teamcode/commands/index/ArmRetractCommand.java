package org.firstinspires.ftc.teamcode.commands.index;

import com.technototes.library.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.IndexSubsystem;

public class ArmRetractCommand extends WaitCommand {
    public IndexSubsystem indexSubsystem;
    public ArmRetractCommand(IndexSubsystem subsystem){
        //COOLDOWN
        super(0.1);
        //addRequirements(subsystem);
        indexSubsystem = subsystem;
    }

    @Override
    public void init() {
        indexSubsystem.retractArm();
    }

    @Override
    public void execute() {
        System.out.print("armretract");

    }
}

