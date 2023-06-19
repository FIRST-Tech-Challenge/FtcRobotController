package org.firstinspires.ftc.teamcode.commandBased.commands._groups;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.classes.triggers.TriggerCommandGroup;
import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

public class GrabCone extends TriggerCommandGroup {

    public GrabCone(
            ElevatorSubsystem ele,
            ArmSubsystem arm,
            RotatorSubsystem rot,
            IntakeSubsystem intake
    ) {
        addCommands(
                new MoveArmToAngle(arm, Constants.ARM_ANGLE_IDLE, Constants.ARM_IDLE_VELO, Constants.ARM_IDLE_ACCEL),
                new MoveRotatorToPosition(rot, Constants.ROTATOR_FORWARD),
                new MoveElevatorToPosition(ele, Constants.ELE_LOW),
                new SetIntakePower(intake, Constants.INTAKE_IN),
                new SetIntakePower(intake, Constants.INTAKE_IDLE),
                new MoveElevatorToPosition(ele, Constants.ELE_MID_LOW)
        );
    }
}
