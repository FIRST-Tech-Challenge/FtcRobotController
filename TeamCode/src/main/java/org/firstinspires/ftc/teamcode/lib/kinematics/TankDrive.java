package org.firstinspires.ftc.teamcode.lib.kinematics;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface TankDrive extends Subsystem {
    void tankDrive(double left, double right);

    default void stop() {}
}
