package org.firstinspires.ftc.teamcode.lib.kinematics;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface MecanumDrive extends TankDrive, HolonomicDrive, Subsystem {
    void driveLeft(double power);
    void driveRight(double power);
}
