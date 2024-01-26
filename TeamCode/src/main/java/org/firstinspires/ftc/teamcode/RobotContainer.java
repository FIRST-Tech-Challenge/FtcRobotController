package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.BTController.Buttons.*;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.climb;
import org.firstinspires.ftc.teamcode.subsystems.plane;
import org.firstinspires.ftc.teamcode.utils.BTController;

import java.util.function.BooleanSupplier;

// todo: check the time from the start to when I have the cords

public class RobotContainer extends com.arcrobotics.ftclib.command.Robot {
     Chassis m_chassis;
     BTController m_controller;
     Gripper m_gripper;
     plane m_plane;
     climb m_climb;
     arm m_arm;




    public RobotContainer(HardwareMap map,Telemetry telemetry,Gamepad gamepad1,Gamepad gamepad2){
        m_controller = new BTController(gamepad1);

  //      m_gripper = new Gripper(map,telemetry);


        m_chassis= new Chassis(map, telemetry);
//        m_plane = new plane(map, telemetry);
//        m_climb = new climb(map, telemetry);
//        m_arm= new arm(map, telemetry);


        bindCommands();
    }



    //bind commands to trigger
    public void bindCommands(){

        m_controller.assignCommand(m_chassis.fieldRelativeDrive(
                ()->-m_controller.left_y.getAsDouble(),
                m_controller.left_x,
                ()-> -m_controller.left_trigger.getAsDouble()+ m_controller.right_trigger.getAsDouble()),
                true, LEFT_X, LEFT_Y, LEFT_TRIGGER, RIGHT_TRIGGER).whenInactive(m_chassis.stopMotor());
//
//       m_controller.assignCommand(m_gripper.toggleGripper(),false,BUTTON_RIGHT);
////        m_controller.assignCommand(m_plane.shootPlane(),false,DPAD_RIGHT);
//        m_controller.assignCommand(m_gripper.toggleGripper1(),false,BUTTON_RIGHT);
//        m_controller.assignCommand(m_gripper.toggleGripper2(),false,BUTTON_LEFT);
////        m_controller.assignCommand(m_climb.climb_up( ),false,DPAD_LEFT);
//        m_controller.assignCommand(m_arm.downArm(m_controller.right_y),false,RIGHT_Y);
//        m_controller.assignCommand(m_arm.downArm(m_controller.left_y),false,LEFT_Y);
//        m_controller.assignCommand(m_arm.downArm(()-> -m_controller.left_trigger.getAsDouble()+ m_controller.right_trigger.getAsDouble()),false,RIGHT_TRIGGER,LEFT_TRIGGER);
//        m_controller.assignCommand(m_climb.climb_manual(m_controller.right_y), true, RIGHT_Y).whenInactive(m_climb.climb_manual(()->0));

// TODO:גריפר לזוז
    }
    public Command AutonomousCommand(){
        return null;
    };

}
