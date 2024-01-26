package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagDetector;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.gripper;
import org.firstinspires.ftc.teamcode.utils.BTController;


public class RobotContainer extends com.arcrobotics.ftclib.command.Robot {
    private Gamepad m_gamepad1,m_gamepad2;
     Chassis m_chassis;
     BTController m_controller;
     gripper m_gripper;
     AprilTagDetector m_tagDetector;
     ElapsedTime elapsedTime;
    AprilTag aprilTag;

    MotorEx motorL;
    MotorEx motorR;
    public RobotContainer(HardwareMap map,Telemetry telemetry){
        motorR = new MotorEx(map, "encoderRight");
        motorL = new MotorEx(map, "encoderLeft");

//        m_controller = new BTController(gamepad1);
//        m_tagDetector = new AprilTagDetector(aprilTag);
//        m_gripper = new gripper(map,telemetry);
        m_chassis= new Chassis(map, telemetry, ()->motorL.encoder.getPosition(),()->motorR.encoder.getPosition());
//        m_gamepad1=gamepad1;
//        m_gamepad2=gamepad2;

    m_chassis.register();

//        bindCommands();
    }
    //bind commands to trigger
    public void bindCommands(){

//        m_controller.assignCommand(m_chassis.drive(()-> m_gamepad1.left_stick_x, ()-> m_gamepad1.left_trigger+ m_gamepad1.right_trigger, ()-> m_gamepad1.left_stick_y),
//                true, LEFT_X, LEFT_Y, LEFT_TRIGGER, RIGHT_TRIGGER);
    }
    public Command AutonomousCommand(){
        return null;
    }
}
