package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.*;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.utils.BTController;

import java.util.function.BooleanSupplier;

// todo: check the time from the start to when I have the cords

public class RobotContainer extends com.arcrobotics.ftclib.command.Robot {
    private Gamepad m_gamepad1,m_gamepad2;
     Chassis m_chassis;
     BTController m_controller;
     Gripper m_gripper;
     plane m_plane;
     climb m_climb;


    public RobotContainer(HardwareMap map,Telemetry telemetry,Gamepad gamepad1,Gamepad gamepad2, BooleanSupplier[] m_buttonsSuppliers){
        m_gamepad1=gamepad1;
        m_gamepad2=gamepad2;
        m_controller = new BTController(gamepad1, m_buttonsSuppliers, ()->gamepad1.left_stick_x, ()->gamepad2.left_stick_x, ()->gamepad1.left_stick_y, ()->gamepad2.left_stick_y);
        m_gripper = new Gripper(map,telemetry);
        m_chassis= new Chassis(map, telemetry);
        m_plane = new plane(map, telemetry);
        m_climb = new climb(map, telemetry);
        m_gamepad1=gamepad1;
        m_gamepad2=gamepad2;


        bindCommands();
    }
    //bind commands to trigger
    public void bindCommands(){

        m_controller.assignCommand(m_chassis.drive(m_controller.left_x2, ()-> m_gamepad1.left_trigger+ m_gamepad1.right_trigger, ()-> m_gamepad1.left_stick_y),
                true, LEFT_X2, LEFT_Y2, LEFT_TRIGGER2, RIGHT_TRIGGER2);

        m_controller.assignCommand(m_gripper.closeGripper(),false,BUTTON_LEFT1);WhileInActive()
        m_controller.assignCommand(m_gripper.openGripper(),false,BUTTON_LEFT1);
        m_controller.assignCommand(m_plane.shootPlane(),false,BUTTON_UP1);
//        m_controller.assignCommand(m_climb.climb( ),false,BUTTON_UP1); //TODO: ask Adi about trapezoid profiles
        m_controller.assignCommand(m_plane.shootPlane(),false,BUTTON_UP1);

//סוגר ביחד גריפר A

    }
    public Command AutonomousCommand(){
        return null;
    }


}
