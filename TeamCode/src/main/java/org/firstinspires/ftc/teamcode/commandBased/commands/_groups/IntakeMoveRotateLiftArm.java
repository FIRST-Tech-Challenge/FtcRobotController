package org.firstinspires.ftc.teamcode.commandBased.commands._groups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.classes.triggers.TriggerCommandGroup;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

public class IntakeMoveRotateLiftArm extends TriggerCommandGroup {

    public IntakeMoveRotateLiftArm(
            ElevatorSubsystem ele,
            ArmSubsystem arm,
            RotatorSubsystem rot,
            IntakeSubsystem intake,
            double armAngle,
            double armVelo,
            double armAccel,
            double eleTarget,
            double rotAngle,
            double intakePower
    ) {
        addCommands(
                new SetIntakePower(intake, intakePower),
                new MoveArmToAngle(arm, armAngle, armVelo, armAccel),
                new MoveRotatorToPosition(rot, rotAngle),
                new MoveElevatorToPosition(ele, eleTarget)
        );
    }
}
