package org.firstinspires.ftc.teamcode.commandBased.commands._groups.tele;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

import static org.firstinspires.ftc.teamcode.commandBased.Constants.*;

public class ScoreToIdle extends SequentialCommandGroup {

    public ScoreToIdle(
            ElevatorSubsystem ele,
            ArmSubsystem arm,
            RotatorSubsystem rot,
            IntakeSubsystem intake
    ) {
        addCommands(
                new ScoreCone(arm, rot, intake),
                new MoveElevatorToPosition(ele, ELE_IDLE)
        );
        addRequirements(ele, arm, rot, intake);
    }
}
