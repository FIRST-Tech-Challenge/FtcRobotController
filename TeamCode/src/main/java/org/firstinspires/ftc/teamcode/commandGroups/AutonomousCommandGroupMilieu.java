package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AvancerAutoCommande;
import org.firstinspires.ftc.teamcode.commands.TournerAutoCommande;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class AutonomousCommandGroupMilieu extends SequentialCommandGroup {

    public AutonomousCommandGroupMilieu(Telemetry telemetry, DriveSubsystem driveSubsystem) {

        AvancerAutoCommande avancer80cm = new AvancerAutoCommande(telemetry, driveSubsystem, 20, 0.6);


            addCommands(
                    avancer80cm

            );

    }

}
