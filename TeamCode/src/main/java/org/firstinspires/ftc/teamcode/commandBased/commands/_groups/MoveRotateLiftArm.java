package org.firstinspires.ftc.teamcode.commandBased.commands._groups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

public class MoveRotateLiftArm extends SequentialCommandGroup {

    public MoveRotateLiftArm(
            ElevatorSubsystem ele,
            ArmSubsystem arm,
            RotatorSubsystem rot,
            double armAngle,
            double eleTarget,
            double rotAngle
    ) {
        addCommands(
                new MoveArmToAngle(arm, armAngle),
                new MoveRotatorToPosition(rot, rotAngle),
                new MoveElevatorToPosition(ele, eleTarget)
        );
        addRequirements(ele, arm, rot);
    }
}
