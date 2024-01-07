package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveRobotCentric extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSub drive;
    double y;double x;double rx;boolean slow_mode;
    public DriveRobotCentric(DriveSub subsystem,double y1,double x1,double rx1,boolean slow_mode1) {
        y=y1;x=x1;rx=rx1;slow_mode=slow_mode1;
       drive = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    public void execute() {
        drive.runRobotCentric(y,x,rx,slow_mode);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
