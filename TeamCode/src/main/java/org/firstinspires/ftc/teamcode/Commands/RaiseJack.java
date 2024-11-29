package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;


// command template
public class RaiseJack extends CommandBase {
    ElapsedTime timer;
    // constructor
    public RaiseJack() {
        timer = new ElapsedTime();
        addRequirements(RobotContainer.climb);
        // add subsystem requirements (if any) - for example:
        //addRequirements(RobotContainer.drivesystem);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        timer.reset();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        RobotContainer.climb.ClimbServoSpeed(0.0);
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return (timer.seconds()>=30.0) || (RobotContainer.climb.GetLowLimit());

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

        RobotContainer.climb.ClimbServoSpeed(0.5);

    }

}