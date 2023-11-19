package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController.RFMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.ArrayList;

/**
 * Warren Zhou
 * 7/29/23
 * Test teleOp for a butterfly drivetrain
 */
@Disabled
@Config
@TeleOp(name = "ButterflyTest")
public class ButterflyTest extends LinearOpMode {
    public static double INITIAL1 = 0.005, INITIAL2 = 0.005,INITIAL3 = .01, INITIAL4 = 0.9;
    public static double FINAL1 = 1.00, FINAL2 = 0.605,FINAL3 = .601, FINAL4 = 0.20;

    private boolean isButtered = false;
    private ArrayList<Servo> servos;


    public void runOpMode(){
        double lastSwitchTime = 0;
        BasicRobot robot = new BasicRobot(this, true);
        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        servos = new ArrayList<>();
        servos.add(hardwareMap.servo.get("servoLeftFront"));
        servos.add(hardwareMap.servo.get("servoLeftBack"));
        servos.add(hardwareMap.servo.get("servoRightFront"));
        servos.add(hardwareMap.servo.get("servoRightBack"));
        toggleServos();
        waitForStart();
        while(opModeIsActive()){
            drive.setWeightedDrivePower(new Pose2d(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));
            if(time>lastSwitchTime+1.0 && gamepad1.a){
                isButtered=!isButtered;
                toggleServos();
                drive.toggleButtered();
                lastSwitchTime=time;
            }
            robot.update();
        }
    }
    public void toggleServos(){
        if(isButtered){
            servos.get(0).setPosition(INITIAL1);
            servos.get(1).setPosition(INITIAL2);
            servos.get(2).setPosition(INITIAL3);
            servos.get(3).setPosition(INITIAL4);
//            isButtered = false;
        }
        else{
                servos.get(0).setPosition(FINAL1);
                servos.get(1).setPosition(FINAL2);
                servos.get(2).setPosition(FINAL3);
                servos.get(3).setPosition(FINAL4);
//                isButtered = true;
        }
    }
}
