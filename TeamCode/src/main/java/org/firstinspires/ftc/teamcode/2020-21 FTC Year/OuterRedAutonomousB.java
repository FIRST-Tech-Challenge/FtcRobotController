//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
////@Autonomous(name = "OuterRedAutonomous B")
//public class OuterRedAutonomousB extends LinearOpMode {
//
//    RobotClass robot;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        robot= new RobotClass(hardwareMap, telemetry, this);
//        robot.wobbleGoalGrippyThingGrab();
//
//        waitForStart();
//
//        robot.strafeLeft(.5,.2);
//        robot.forward(.6,-2.6);
//        robot.pivotLeft(.3,27);
//        robot.shooterEngage();
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
//        robot.startShooting();
//        robot.stopTimingBelt();
//        robot.pause(950);
//        robot.startTimingBelt();
//        robot.pause(500);
//        robot.stopTimingBelt();
//        robot.pause(750);
//        robot.startTimingBelt();
//        robot.pause(2000);
//        robot.stopShooting();
//
//        robot.pivotRight(.5, 27);
//        robot.forward(.8,-2);
//        robot.strafeRight(.6,2.1);
//        robot.forward(.6,-1.5);
//        robot.pivotRight(.5,105);
//        robot.pause(500);
//        robot.depositWobbleGoal();
//
//    }
//}