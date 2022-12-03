package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AvancerAutoCommande;
import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.TournerAutoCommande;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class AutonomousCommandGroupGauche extends SequentialCommandGroup {

    public AutonomousCommandGroupGauche(Telemetry telemetry, DriveSubsystem driveSubsystem) {

        AvancerAutoCommande avancer30cm = new AvancerAutoCommande(telemetry, driveSubsystem, 30, 0.5);
        TournerAutoCommande tourner90 = new TournerAutoCommande(telemetry, driveSubsystem, 90, 0.5);

            addCommands(
                    avancer30cm,
                    tourner90
            );

    }

}
