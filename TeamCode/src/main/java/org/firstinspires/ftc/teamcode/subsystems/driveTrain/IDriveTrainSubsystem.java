package org.firstinspires.ftc.teamcode.subsystems.driveTrain;

import org.firstinspires.ftc.teamcode.util.subsystems.SympleSubsystem;

/**
 * Interface for {@link SympleSubsystem}s, use this interface on a subsystem to make it support the common drive train commands
 */
public interface IDriveTrainSubsystem extends SympleSubsystem {
    void moveSideMotors(double left, double right);
    double getForwardDistanceDriven();
    double getHeading();
}
