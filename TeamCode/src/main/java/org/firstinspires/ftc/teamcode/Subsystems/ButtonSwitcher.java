package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.RobotContainer;

public class ButtonSwitcher extends SubsystemBase {

   private ButtonSwitcherState state;
    public ButtonSwitcher()
    {
        state = ButtonSwitcherState.AUTO_MODE;

    }

    @Override
    public void periodic(){
        //check the state of the xbox switch mode button
    }


    public ButtonSwitcherState getButtonState() {
        if (RobotContainer.driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            state = (state == ButtonSwitcherState.AUTO_MODE) ? ButtonSwitcherState.MANUAL_MODE : ButtonSwitcherState.AUTO_MODE;
        }
        return state;
    }


}
