package org.firstinspires.ftc.teamcode.commandBased.commands._groups.auto.general;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandBased.classes.enums.Stack;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

import static org.firstinspires.ftc.teamcode.commandBased.Constants.*;

public class GrabConeStackAuto extends SequentialCommandGroup {

    public GrabConeStackAuto(
            ElevatorSubsystem ele,
            ArmSubsystem arm,
            RotatorSubsystem rot,
            IntakeSubsystem intake,
            Stack.Cone height
    ) {
        addCommands(
                new MoveArmToAngle(arm, ARM_ANGLE_STACK),
                new MoveRotatorToPosition(rot, ROTATOR_FRONT),
                new MoveElevatorToPosition(ele, height.getValue()),
                new SetIntakePower(intake, INTAKE_IN).withTimeout(2000),
                new SetIntakePower(intake, INTAKE_IDLE)
        );
    }
}
