package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotContainer;


// command template
public class ClawSense extends CommandBase {

    // constructor
    public ClawSense() {

        // add subsystem requirements (if any) - for example:
        //addRequirements(RobotContainer.drivesystem);

        addRequirements(RobotContainer.clawTouch);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        return RobotContainer.clawTouch.ControleTouchSenser();



    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

    }

}