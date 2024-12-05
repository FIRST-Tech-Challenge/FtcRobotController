package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.RobotContainer.elbowJoint;
import static org.firstinspires.ftc.teamcode.RobotContainer.linearSlide;
import static org.firstinspires.ftc.teamcode.RobotContainer.shoulderJoint;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.ArmStowHigh;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.HighBucketDeposit;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.PlaceSpecimenAddOffset;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.WallPickUp;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.ButtonSwitcher;
import org.firstinspires.ftc.teamcode.Subsystems.ButtonSwitcherState;
import org.firstinspires.ftc.teamcode.Subsystems.ElbowPosition;
import org.firstinspires.ftc.teamcode.Subsystems.ShoulderPosition;
import org.firstinspires.ftc.teamcode.Subsystems.SlideTargetHeight;

public class PollButtonSwitcherButton extends CommandBase {
    private final ButtonSwitcher buttonSwitcher;
    private ButtonSwitcherState currentState;

    public PollButtonSwitcherButton(ButtonSwitcher buttonSwitcher) {
        this.buttonSwitcher = buttonSwitcher;
        addRequirements(buttonSwitcher);
    }

    @Override
    public void initialize() {
        // Initialization code here
        currentState = buttonSwitcher.getButtonState();
    }

    @Override
    public void execute() {
        // Code to poll the button state
        ButtonSwitcherState incomingState = buttonSwitcher.getButtonState();
        if (incomingState != currentState) {
            currentState = incomingState;
            // map the appropriate commands to the buttons
            switch (currentState) {
                case AUTO_MODE:
                    // map the auto commands
                    RobotContainer.driverOp.getGamepadButton(GamepadKeys.Button.A).whenHeld(new WallPickUp());
                    RobotContainer.driverOp.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ArmStowHigh());
                    RobotContainer.driverOp.getGamepadButton(GamepadKeys.Button.X).whenHeld(new PlaceSpecimenAddOffset());
                    RobotContainer.driverOp.getGamepadButton(GamepadKeys.Button.B).whenHeld(new HighBucketDeposit());
                    break;
                case MANUAL_MODE:
                    //map the manual commands
                    //probably don't want these specific calls, but you can
                    RobotContainer.driverOp.getGamepadButton(GamepadKeys.Button.A).whenHeld(new InstantCommand(() -> shoulderJoint.RotateTo(ShoulderPosition.FLEX.getValue())));
                    RobotContainer.driverOp.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> shoulderJoint.RotateTo(ShoulderPosition.GROUND.getValue())));
                    RobotContainer.driverOp.getGamepadButton(GamepadKeys.Button.X).whenHeld(new InstantCommand(() -> elbowJoint.RotateTo(ElbowPosition.FLEX.getValue())));
                    RobotContainer.driverOp.getGamepadButton(GamepadKeys.Button.B).whenHeld(new InstantCommand(() -> elbowJoint.RotateTo(ElbowPosition.GROUND.getValue())));
                    break;
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Return true when the command should end
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Cleanup code here
    }
}