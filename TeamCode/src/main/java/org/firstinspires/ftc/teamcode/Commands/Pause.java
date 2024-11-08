package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;


// command template
public class Pause extends CommandBase {
    ElapsedTime delayTimer;
    double delayTime;
    // constructor
    public Pause(double delay) {
        delayTime = delay;
        delayTimer = new ElapsedTime();
        // add subsystem requirements (if any) - for example:
        //addRequirements(RobotContainer.drivesystem);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        delayTimer.reset();

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        if(delayTimer.seconds()>=delayTime)
            return true;

        else
            return false;

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

    }

}