package org.firstinspires.ftc.teamcode.commandBased.commands._groups.tele;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;

public class ResetArm extends SequentialCommandGroup {

    public ResetArm(ArmSubsystem armSubsystem) {
        addCommands(
                new InstantCommand(() -> armSubsystem.setDisabled(true)),
                new WaitCommand(1000),
                new InstantCommand(armSubsystem::resetEncoder),
                new InstantCommand(() -> armSubsystem.setDisabled(false))
        );
    }
}
