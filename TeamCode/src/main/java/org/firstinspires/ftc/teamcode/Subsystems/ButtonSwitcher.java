package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;

public class ButtonSwitcher extends SubsystemBase {
    private boolean initalState;
    public ButtonSwitcher(){
        initalState = true;
    }

    @Override
    public void periodic(){
        //check the state of the xbox switch mode button
    }


}
