package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.DriveSub;

public class ResetIMU extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSub drive;
    public ResetIMU(DriveSub subsystem) {
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
