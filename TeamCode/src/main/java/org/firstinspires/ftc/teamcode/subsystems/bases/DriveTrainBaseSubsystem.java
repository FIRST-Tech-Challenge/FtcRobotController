package org.firstinspires.ftc.teamcode.subsystems.bases;

import org.firstinspires.ftc.teamcode.util.subsystems.SympleSubsystem;

/**
 * Interface for {@link SympleSubsystem}s, use this interface on a subsystem to make it support the common drive train commands
 */
public interface DriveTrainBaseSubsystem extends SympleSubsystem {
    void moveSideMotors(double left, double right);
    double getForwardDistanceDriven();
    double getHeading();
}
