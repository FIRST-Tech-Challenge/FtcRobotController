package org.firstinspires.ftc.teamcode.lib.drive;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ArcadeDrive extends Subsystem {
    void arcadeDrive(double x, double y, double spin);
    void stop();
}
