package org.firstinspires.ftc.teamcode.subsystems.driveTrain;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.util.LoggerSubsystem;

public interface IDriveTrainSubsystem extends LoggerSubsystem, Subsystem {
    void moveSideMotors(double left, double right);
    double getForwardDistanceDriven();
    double getHeading();
}
