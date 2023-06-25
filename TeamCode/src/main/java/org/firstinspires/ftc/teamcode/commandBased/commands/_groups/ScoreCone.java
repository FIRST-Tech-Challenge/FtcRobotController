package org.firstinspires.ftc.teamcode.commandBased.commands._groups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

public class ScoreCone extends SequentialCommandGroup {

    public ScoreCone(
            ArmSubsystem arm,
            RotatorSubsystem rot,
            IntakeSubsystem intake
    ) {
        addCommands(
                new SetIntakePower(intake, Constants.INTAKE_OUT),
                new MoveArmToAngle(arm, Constants.ARM_ANGLE_IDLE),
                new MoveRotatorToPosition(rot, Constants.ROTATOR_FRONT),
                new SetIntakePower(intake, Constants.INTAKE_IDLE)
        );
    }

}
