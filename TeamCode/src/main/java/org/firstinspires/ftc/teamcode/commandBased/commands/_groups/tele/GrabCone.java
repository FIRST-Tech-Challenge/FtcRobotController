package org.firstinspires.ftc.teamcode.commandBased.commands._groups.tele;

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

import static org.firstinspires.ftc.teamcode.commandBased.Constants.*;

public class GrabCone extends SequentialCommandGroup {

    public GrabCone(
            ElevatorSubsystem ele,
            ArmSubsystem arm,
            RotatorSubsystem rot,
            IntakeSubsystem intake
    ) {
        addCommands(
                new MoveArmToAngle(arm, ARM_ANGLE_STACK),
                new MoveRotatorToPosition(rot, ROTATOR_FRONT),
                new MoveElevatorToPosition(ele, ELE_LOW),
                new SetIntakePower(intake, INTAKE_IN).withTimeout(2000),
                new SetIntakePower(intake, INTAKE_IDLE),
                new MoveElevatorToPosition(ele, ELE_MID)
        );
    }
}
