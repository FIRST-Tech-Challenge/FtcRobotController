package org.firstinspires.ftc.teamcode.commandBased.commands._groups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commandBased.classes.triggers.TriggerCommandGroup;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

import java.util.function.BooleanSupplier;

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
                new MoveElevatorToPosition(ele, eleTarget),
                new MoveArmToAngle(arm, armAngle),
                new MoveRotatorToPosition(rot, rotAngle)

        );
        addRequirements(ele, arm, rot);
    }
}
