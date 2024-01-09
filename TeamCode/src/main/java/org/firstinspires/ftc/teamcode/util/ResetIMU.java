package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;

public class ResetIMU extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem drive;
    public ResetIMU(DriveSubsystem subsystem) {
       drive = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    public void execute() {
        drive.resetIMU();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
