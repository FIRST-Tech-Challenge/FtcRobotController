package org.firstinspires.ftc.teamcode.commandBased.commands._groups;

import org.firstinspires.ftc.teamcode.classes.triggers.TriggerCommandBase;
import org.firstinspires.ftc.teamcode.classes.triggers.TriggerCommandGroup;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

public class LiftMoveRotateArmCmd extends TriggerCommandGroup {

    public LiftMoveRotateArmCmd(
            ElevatorSubsystem eleSS,
            ArmSubsystem armSS,
            RotatorSubsystem rotSS,
            TriggerCommandBase eleCommand,
            TriggerCommandBase armCommand,
            TriggerCommandBase rotCommand
    ) {
        addCommands(
                eleCommand,
                armCommand,
                rotCommand
        );
        addRequirements(
                eleSS,
                armSS,
                rotSS
        );

    }
}
