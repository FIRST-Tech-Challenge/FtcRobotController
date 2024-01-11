package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.*;

import android.media.Image;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.utils.BTController;

// todo: check the time from the start to when I have the cords

public class RobotContainer extends com.arcrobotics.ftclib.command.Robot {
    private Gamepad m_gamepad1,m_gamepad2;
     Chassis m_chassis;
     BTController m_controller;
     Gripper m_gripper;
     plane m_plane;


    public RobotContainer(HardwareMap map,Telemetry telemetry,Gamepad gamepad1,Gamepad gamepad2){
        m_gamepad1=gamepad1;
        m_gamepad2=gamepad2;
        m_controller = new BTController(gamepad1);
        m_gripper = new Gripper(map,telemetry);
        m_chassis= new Chassis(map, telemetry);
        m_plane = new plane(map, telemetry);
        m_gamepad1=gamepad1;
        m_gamepad2=gamepad2;


        bindCommands();
    }
    //bind commands to trigger
    public void bindCommands(){

        m_controller.assignCommand(m_chassis.drive(m_controller.left_x, ()-> m_gamepad1.left_trigger+ m_gamepad1.right_trigger, ()-> m_gamepad1.left_stick_y),
                true, LEFT_X, LEFT_Y, LEFT_TRIGGER, RIGHT_TRIGGER);

        m_controller.assignCommand(m_gripper.closeGripper(),false,BUTTON_LEFT).whenInactive();
        m_controller.assignCommand(m_gripper.openGripper(),false,BUTTON_LEFT);
        m_controller.assignCommand(m_plane.shootPlane(),false,BUTTON_UP);


    }
    public Command AutonomousCommand(){
        return null;
    }


}
