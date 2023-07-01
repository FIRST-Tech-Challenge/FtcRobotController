package org.firstinspires.ftc.teamcode.commandBased.commands._groups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

import static org.firstinspires.ftc.teamcode.commandBased.Constants.*;

public class LiftMoveRotateArm extends ParallelCommandGroup {

    public LiftMoveRotateArm(
            ElevatorSubsystem ele,
            ArmSubsystem arm,
            RotatorSubsystem rot,
            double armAngle,
            double eleTarget,
            double rotAngle
    ) {

        addCommands(
                new MoveElevatorToPosition(ele, eleTarget).withTimeout(1250),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> ele.getEleProfileTarget() > ELE_TRIGGER),
                        new SequentialCommandGroup(
                                new MoveRotatorToPosition(rot, rotAngle),
                                new WaitCommand(100),
                                new MoveArmToAngle(arm, armAngle)
                        )
                )
        );
        addRequirements(ele, arm, rot);
    }
}
