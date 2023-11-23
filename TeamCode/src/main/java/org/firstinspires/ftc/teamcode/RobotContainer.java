package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.utils.BTController;

public class RobotContainer extends com.arcrobotics.ftclib.command.Robot {
    private Gamepad gamepad;
     Chassis m_chassis;
     BTController m_controller;
    private final exampleSubsytem m_exmapleSubsystem;
    public RobotContainer(HardwareMap map){
        m_controller = new BTController();
        m_exmapleSubsystem= new exampleSubsytem(map);

    }
    //bind commands to trigger
    public void bindCommands(){
        m_controller.assignCommand(m_chassis.drive(()->gamepad.left_stick_x, ()->gamepad.left_trigger+gamepad.right_trigger, ()->gamepad.left_stick_y),
                true, BTController.Axis.LEFT_X, BTController.Axis.LEFT_Y, BTController.Axis.LEFT_TRIGGER, BTController.Axis.RIGHT_TRIGGER);
    }

    public Command AutonomousCommand(){
        return null;
    }
}
