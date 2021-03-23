package org.firstinspires.ftc.teamcode.commands.index;

import com.technototes.library.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.IndexSubsystem;

public class SendRingToShooterCommand extends SequentialCommandGroup {
    public IndexSubsystem indexSubsystem;
    public SendRingToShooterCommand(IndexSubsystem subsystem){
        //addRequirements(subsystem);
        indexSubsystem = subsystem;
        addCommand(new ArmExtendCommand(subsystem)).addCommand(new ArmRetractCommand(subsystem));
    }
}
