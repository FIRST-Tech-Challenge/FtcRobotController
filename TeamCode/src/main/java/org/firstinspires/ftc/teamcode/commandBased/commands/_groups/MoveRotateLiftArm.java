package org.firstinspires.ftc.teamcode.commandBased.commands._groups;

import org.firstinspires.ftc.teamcode.classes.triggers.TriggerCommandGroup;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

public class MoveRotateLiftArm extends TriggerCommandGroup {

    public MoveRotateLiftArm(
            ElevatorSubsystem ele,
            ArmSubsystem arm,
            RotatorSubsystem rot,
            double armAngle,
            double armVelo,
            double armAccel,
            double eleTarget,
            double rotAngle
    ) {
        addCommands(
                new MoveArmToAngle(arm, armAngle, armVelo, armAccel),
                new MoveRotatorToPosition(rot, rotAngle),
                new MoveElevatorToPosition(ele, eleTarget)
        );
        addRequirements(ele, arm, rot);
    }
}
