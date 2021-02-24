package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "OuterRedAutonomous")
public class OuterRedAutonomous extends LinearOpMode {

    RobotClass robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot= new RobotClass(hardwareMap, telemetry, this);
        robot.wobbleGoalGrippyThingGrab();

        waitForStart();

        robot.strafeLeft(.5,.2);
        robot.forward(.6,-2.2);
        robot.pivotLeft(.5,27);
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
        robot.pivotRight(.5, 27);
        robot.forward(.8,-1.8);
        robot.strafeRight(.6,2.1);
        robot.forward(.6,-1.5);
        robot.pivotRight(.8,105);
        robot.moveWobbleGoalArm(.8,.3);
        robot.wobbleGoalGrippyThingRelease();
        robot.moveWobbleGoalArm(.7, -.4);

    }
}
