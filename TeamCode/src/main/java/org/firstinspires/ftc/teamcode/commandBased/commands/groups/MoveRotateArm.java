package org.firstinspires.ftc.teamcode.commandBased.commands.groups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

public class MoveRotateArm extends SequentialCommandGroup {

    public MoveRotateArm(
            ArmSubsystem arm,
            IntakeSubsystem intake,
            double armAngle,
            double velo,
            double accel,
            double intakePower
    ) {
        addCommands(
                new TriggerCommandGroup(
                    new MoveArmToAngle(arm, armAngle, velo, accel)
                ),
                new SetIntakePower(intake, intakePower)
        );
    }
}
