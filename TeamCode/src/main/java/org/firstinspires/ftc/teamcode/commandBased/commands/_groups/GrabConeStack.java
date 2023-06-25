package org.firstinspires.ftc.teamcode.commandBased.commands._groups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.classes.enums.Stack;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

public class GrabConeStack extends SequentialCommandGroup {

    public GrabConeStack(
            ElevatorSubsystem ele,
            ArmSubsystem arm,
            RotatorSubsystem rot,
            IntakeSubsystem intake,
            Stack.Cone height
    ) {
        addCommands(
                new MoveArmToAngle(arm, Constants.ARM_ANGLE_IDLE),
                new MoveRotatorToPosition(rot, Constants.ROTATOR_FRONT),
                new MoveElevatorToPosition(ele, height.getValue()),
                new SetIntakePower(intake, Constants.INTAKE_IN).withTimeout(4000),
                new SetIntakePower(intake, Constants.INTAKE_IDLE),
                new MoveElevatorToPosition(ele, Constants.ELE_IDLE)
        );
    }
}
