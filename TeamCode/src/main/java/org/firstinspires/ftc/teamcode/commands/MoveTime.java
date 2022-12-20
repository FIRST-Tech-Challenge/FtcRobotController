package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.DriveSystem;

public class MoveTime extends CommandBase {
    DriveSystem driveSystem;
    ElapsedTime timer;
    double endTime;
    
    double strafe;
    double forward;
    double turn;
    
    public MoveTime(
        final ElapsedTime timer, DriveSystem driveSystem, 
        double strafe, double forward, double turn, double time
    ) {
        this.driveSystem = driveSystem;
        this.timer = timer;
        
        endTime = timer.milliseconds() + time;
        
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;
        
        addRequirements(driveSystem);
    }

    @Override
    public void initialize() {
        driveSystem.setAll(strafe, forward, turn);
    }

    @Override
    public void end(boolean interrupted) {
        driveSystem.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= endTime;
    }
}
