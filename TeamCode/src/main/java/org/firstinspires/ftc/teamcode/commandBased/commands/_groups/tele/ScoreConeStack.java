package org.firstinspires.ftc.teamcode.commandBased.commands._groups.tele;

import static org.firstinspires.ftc.teamcode.commandBased.Constants.ARM_ANGLE_IDLE;
import static org.firstinspires.ftc.teamcode.commandBased.Constants.ARM_ANGLE_STACK;
import static org.firstinspires.ftc.teamcode.commandBased.Constants.INTAKE_IDLE;
import static org.firstinspires.ftc.teamcode.commandBased.Constants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.commandBased.Constants.ROTATOR_FRONT;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

public class ScoreConeStack extends SequentialCommandGroup {

    public ScoreConeStack(
            ArmSubsystem arm,
            RotatorSubsystem rot,
            IntakeSubsystem intake
    ) {
        addCommands(
                new ParallelCommandGroup(
                        new SetIntakePower(intake, INTAKE_OUT),
                        new WaitCommand(250)
                ),
                new MoveArmToAngle(arm, ARM_ANGLE_STACK),
                new MoveRotatorToPosition(rot, ROTATOR_FRONT),
                new SetIntakePower(intake, INTAKE_IDLE)
        );
    }

}
