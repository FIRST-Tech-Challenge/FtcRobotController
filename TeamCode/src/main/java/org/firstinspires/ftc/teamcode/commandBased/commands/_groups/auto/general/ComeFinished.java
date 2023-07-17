package org.firstinspires.ftc.teamcode.commandBased.commands._groups.auto.general;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

import static org.firstinspires.ftc.teamcode.commandBased.Constants.*;

public class ComeFinished extends SequentialCommandGroup {

    public ComeFinished(
            ElevatorSubsystem ele,
            ArmSubsystem arm,
            RotatorSubsystem rot
    ) {
        addCommands(
                new MoveArmToAngle(arm, ARM_ANGLE_IDLE),
                new WaitCommand(2000),
                new MoveRotatorToPosition(rot, ROTATOR_FRONT),
                new MoveElevatorToPosition(ele, 0)
        );
    }
}
