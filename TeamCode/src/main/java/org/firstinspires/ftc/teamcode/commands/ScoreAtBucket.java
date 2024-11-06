package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

public class ScoreAtBucket extends SequentialCommandGroup {

    public ScoreAtBucket(Drivetrain drivetrain, Arm arm, Elevator elevator) {

        addCommands(
                new ParallelCommandGroup(
                    new DriveToBucket(drivetrain),
                    new ElevatorGoTo(elevator, 38)
                ),
                new SetArmPosition(arm, Arm.ArmState.SCORE).withTimeout(1500),
                new ParallelCommandGroup(
                        new ElevatorGoTo(elevator, 0),
                        new SetArmPosition(arm, Arm.ArmState.INTAKE).withTimeout(500)
                )
        );

        addRequirements(drivetrain, arm, elevator);
    }
}
