package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.BTController.Axis.*;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.utils.BTController;
import org.firstinspires.ftc.teamcode.utils.PoseEstimator;


public class RobotContainer extends com.arcrobotics.ftclib.command.Robot {
    private Gamepad m_gamepad1,m_gamepad2;
     Chassis m_chassis;
     BTController m_controller;
     gripper m_gripper;
     AprilTagDetector m_tagDetector;
     PoseEstimator m_poseEstimator;
    AprilTag aprilTag;
    public RobotContainer(HardwareMap map,Telemetry telemetry,Gamepad gamepad1,Gamepad gamepad2){
        m_controller = new BTController(gamepad1);
        m_tagDetector = new AprilTagDetector(aprilTag);
        m_gripper = new gripper(map,telemetry);
        m_chassis= new Chassis(map, telemetry, m_tagDetector.getTagPose(), m_poseEstimator);
        m_gamepad1=gamepad1;
        m_gamepad2=gamepad2;


        bindCommands();
    }
    //bind commands to trigger
    public void bindCommands(){

        m_controller.assignCommand(m_chassis.drive(()-> m_gamepad1.left_stick_x, ()-> m_gamepad1.left_trigger+ m_gamepad1.right_trigger, ()-> m_gamepad1.left_stick_y),
                true, LEFT_X, LEFT_Y, LEFT_TRIGGER, RIGHT_TRIGGER);
    }
    public Command AutonomousCommand(){
        return null;
    }
}
