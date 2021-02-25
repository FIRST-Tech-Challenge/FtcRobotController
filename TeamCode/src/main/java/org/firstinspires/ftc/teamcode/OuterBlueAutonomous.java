package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;

@Autonomous(name = "OuterBlueAutonomous")
public class OuterBlueAutonomous extends LinearOpMode{

    RobotClass robot;


    @Override
    public void runOpMode() {

        robot= new RobotClass(hardwareMap, telemetry, this);

        robot.wobbleGoalGrippyThingGrab();
        waitForStart();

        robot.forward(.8,-2);
        robot.shooterEngage(4000);
        robot.pause(500);
        robot.shooterServo1(.8);
        robot.shooterServo2(.8);
        robot.pause(500);
        robot.intakeServoEngage(.9);
        robot.pause(4500);
        robot.shooterStop();
        robot.shooterServo1Stop();
        robot.shooterServo2Stop();
        robot.intakeServoStop();
        robot.forward(.8,-1);
        robot.strafeLeft(.6,2.1);
        robot.forward(.6,-1.5);
        robot.pivotRight(.6,15);
        robot.depositWobbleGoal();




//        robot.forward(.8,-6);
//        robot.pivotRight(.4,90);
//        robot.pivotRight(.4,90);
//        robot.moveWobbleGoalArm(.3,.5);
//        robot.wobbleGoalGrippyThingRelease();
//
    }
}
