package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.dragonswpilib.command.ParallelRaceGroup;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveAutoCommand;
import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class AutonomousCommandGroup extends SequentialCommandGroup {

    public AutonomousCommandGroup(Telemetry telemetry, DriveSubsystem driveSubsystem) {

        ParallelRaceGroup avancer5sec = new DriveAutoCommand(telemetry, driveSubsystem).withTimeout(5);

            addCommands(
                    avancer5sec
            );

    }

}
