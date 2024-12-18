package org.firstinspires.ftc.teamcode.Commands.Claw;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Claw.ClawState;


/**Command to open the claw
 * */
public class OpenClaw extends CommandBase {

    // constructor
    public OpenClaw() {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.claw);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        RobotContainer.claw.ControlClaw(ClawState.OPEN);
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