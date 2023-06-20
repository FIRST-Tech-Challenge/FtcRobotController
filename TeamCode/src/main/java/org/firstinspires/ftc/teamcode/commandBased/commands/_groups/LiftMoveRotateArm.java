package org.firstinspires.ftc.teamcode.commandBased.commands._groups;

import org.firstinspires.ftc.teamcode.classes.triggers.TriggerCommandGroup;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

public class LiftMoveRotateArm extends TriggerCommandGroup {

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
