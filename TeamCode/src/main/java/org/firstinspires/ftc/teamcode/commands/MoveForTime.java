package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.DriveSystem;

import java.util.concurrent.TimeUnit;

public class MoveForTime extends CommandBase {
    DriveSystem driveSystem;

    ElapsedTime time;

    double end = -1;

    double forward = 0;
    double strafe = 0;

    public MoveForTime(ElapsedTime elapsedTime, DriveSystem drive,
                       double strafe, double forward, double length) {
        driveSystem = drive;
        time = elapsedTime;
        this.forward = forward;
        this.strafe = strafe;

        end = System.currentTimeMillis() + length;
    }

    @Override
    public void initialize() {
        driveSystem.setStrafe(strafe);
        driveSystem.setForward(forward);
    }

    @Override
    public void end(boolean interrupted) {
        driveSystem.stop();
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() > end;
    }
}
