package org.firstinspires.ftc.teamcode.fieldCentric;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controller.MechanicalDriveBase;

public class CentricDrive
{
    Telemetry telemetry;
    MechanicalDriveBase mechanicalDriveBase;

    public CentricDrive(MechanicalDriveBase mechanicalDriveBase, Telemetry telemetry)
    {
        this.mechanicalDriveBase = mechanicalDriveBase;
        this.telemetry = telemetry;
    }

    public void drive(double x,double y, double robot_heading, double turn)
    {
        double drive_y = y * Math.cos(Math.toRadians(robot_heading)) + x * Math.sin(Math.toRadians(robot_heading));
        double drive_x = -y * Math.sin(Math.toRadians(robot_heading)) + x * Math.cos(Math.toRadians(robot_heading));
        mechanicalDriveBase.driveMotors(drive_y, -turn, -drive_x,1);
        telemetry.update();
    }
}


