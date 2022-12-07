package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AvancerAutoCommande;
import org.firstinspires.ftc.teamcode.commands.TournerAutoCommande;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class AutonomousCommandGroupDroite extends SequentialCommandGroup {

    public AutonomousCommandGroupDroite(Telemetry telemetry, DriveSubsystem driveSubsystem) {

        AvancerAutoCommande avancer60cm = new AvancerAutoCommande(telemetry, driveSubsystem, 60, 0.8);
        TournerAutoCommande tourner90 = new TournerAutoCommande(telemetry, driveSubsystem, 90, -0.5);

            addCommands(
                    avancer60cm,
                    tourner90,
                    avancer60cm,
                    tourner90
            );

    }

}
