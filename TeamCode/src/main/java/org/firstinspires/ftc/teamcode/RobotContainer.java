package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.*;
public class RobotContainer extends com.arcrobotics.ftclib.command.Robot {
    private Gamepad gamepad;
     Chassis m_chassis;
    private final exampleSubsytem m_exmapleSubsystem;
    public RobotContainer(HardwareMap map){
        m_exmapleSubsystem= new exampleSubsytem(map);

    }
    //bind commands to trigger
    public void bindCommands(){
        new Trigger(()->gamepad.left_stick_x > 0.1 || gamepad.left_stick_y > 0.1 ||
                gamepad.left_trigger > 0.1 ||gamepad.right_trigger > 0.1).whileActiveContinuous(
                        m_chassis.drive(()->gamepad.left_stick_x,()-> gamepad.right_trigger - gamepad.left_trigger
                                , ()->gamepad.left_stick_y)
        );

    }

    public Command AutonomousCommand(){
        return null;
    }
}
