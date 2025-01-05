package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.RobotContainer;


// Command executes either cmd1 or cmd2 depending on selected operating mode of robot
public class SelectCommandOnMode extends CommandBase {

    Command cmd1, cmd2;
    Command cmd;

    // constructor
    public SelectCommandOnMode(Command cmd1, Command cmd2) {
        this.cmd1 = cmd1;
        this.cmd2 = cmd2;
        addRequirements(RobotContainer.drivesystem);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        // depending on condition, we are to run either cmd1 or cmd2
        if (!RobotContainer.operatingMode.getSelectedMode())
            cmd = cmd1;
        else
             cmd = cmd2;

        // go ahead and initialize the command. If null, do nothing.
        if (cmd!=null)
            cmd.initialize();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        if (cmd!=null)
            cmd.execute();
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        // we are finished if we have no command to do, or the command is finished
        return (cmd==null || cmd.isFinished());
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        // we are to end - end the command (if there is one)
        if (cmd!=null)
            cmd.end(interrupted);
    }

}