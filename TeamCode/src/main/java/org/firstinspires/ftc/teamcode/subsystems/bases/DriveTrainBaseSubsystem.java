package org.firstinspires.ftc.teamcode.subsystems.bases;

import org.firstinspires.ftc.teamcode.util.subsystems.SympleSubsystem;

public interface DriveTrainBaseSubsystem extends SympleSubsystem {
    void moveSideMotors(double left, double right);
    double getForwardDistanceDriven();
    double getHeading();
}
