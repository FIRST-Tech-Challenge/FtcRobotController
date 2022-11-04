package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveAutoCommand;
import org.firstinspires.ftc.teamcode.dragonswpilib.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class AutonomousCommandGroup extends SequentialCommandGroup {
    public AutonomousCommandGroup(Telemetry telemetry, DriveSubsystem driveSubsystem) {

        DriveAutoCommand avancer40In = new DriveAutoCommand(telemetry, driveSubsystem);

            addCommands(
                    avancer40In
            );
    }

}
