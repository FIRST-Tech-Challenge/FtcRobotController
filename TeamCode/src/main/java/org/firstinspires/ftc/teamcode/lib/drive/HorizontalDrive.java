package org.firstinspires.ftc.teamcode.lib.drive;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface HorizontalDrive extends Subsystem {
    void driveLeft(double power);
    void driveRight(double power);
    void stop();
}
