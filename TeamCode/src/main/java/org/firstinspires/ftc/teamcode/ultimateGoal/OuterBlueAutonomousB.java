//package org.firstinspires.ftc.teamcode;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import java.util.Timer;
//
////@Autonomous(name = "OuterBlueAutonomous B")
//public class OuterBlueAutonomousB extends LinearOpMode{
//
//    RobotClass robot;
//
//
//    @Override
//    public void runOpMode() {
//
//        robot= new RobotClass(hardwareMap, telemetry, this);
//
//        robot.wobbleGoalGrippyThingGrab();
//
//        // robot.innitDisplayTelemetryGyro();
//
//        waitForStart();
//
//        robot.strafeLeft(0.4, 1.9);
//        robot.strafeRight(0.4,1.9);
//        robot.forward(.8,-2);
//        robot.shooterEngageAlt();
//        robot.pause(800);
//        robot.shooterServo1(.8);
//        robot.shooterServo2(.8);
//        robot.pause(200);
//        robot.intakeServoEngage(.9);
//        robot.pause(4500);
//        robot.shooterStop();
//        robot.shooterServo1Stop();
//        robot.shooterServo2Stop();
//        robot.intakeServoStop();
//
//        robot.forward(.6,-5.6);
//        robot.pivotRight(.4,170);
//        robot.depositWobbleGoal();
//        robot.forward(0.5,-2);
//    }
//}