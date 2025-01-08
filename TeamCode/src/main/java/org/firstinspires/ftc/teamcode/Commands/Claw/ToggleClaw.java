package org.firstinspires.ftc.teamcode.Commands.Claw;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Claw.ClawState;
import org.firstinspires.ftc.teamcode.RobotContainer;


/** a toggle to open and close the claw
 * */
public class ToggleClaw extends CommandBase {

    private boolean claw;
    private boolean CurrentButtonState;
    private ElapsedTime debounce;

    // constructor
    public ToggleClaw() {
        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.claw);
        debounce = new ElapsedTime();
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        claw=false;
        debounce.reset();
        CurrentButtonState = false;
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

        // get new button state
        boolean newButtonState = RobotContainer.driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER);

        // previous - was using l-trigger
        //boolean newButtonState = RobotContainer.driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5;

        if (newButtonState && !CurrentButtonState && debounce.seconds() > 0.5) {

            // alternate claw state
            claw = !claw;

            // map the appropriate commands to the buttons
            if (claw) {
                RobotContainer.claw.ControlClaw(ClawState.CLOSE);
            } else {
                RobotContainer.claw.ControlClaw(ClawState.OPEN);
            }
        }

        CurrentButtonState = newButtonState;
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return false;

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
    }

}