package org.firstinspires.ftc.teamcode.commandBased.commands._groups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

public class ComeFinished extends SequentialCommandGroup {

    public ComeFinished(
            ElevatorSubsystem ele,
            ArmSubsystem arm,
            RotatorSubsystem rot
    ) {
        addCommands(
                new MoveArmToAngle(arm, 55),
                new MoveRotatorToPosition(rot, Constants.ROTATOR_FRONT),
                new MoveElevatorToPosition(ele, Constants.ELE_LOW)
        );
    }
}
