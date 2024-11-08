package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotContainer;


// command template
public class CloseClaw extends CommandBase {

    // constructor
    public CloseClaw() {
        // add subsystem requirements (if any) - for example:
        //addRequirements(RobotContainer.claw);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
    //RobotContainer.claw.ClawClosed();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return true;

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

    }

}