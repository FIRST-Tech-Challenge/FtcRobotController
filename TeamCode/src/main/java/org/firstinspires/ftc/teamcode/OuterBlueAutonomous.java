package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "OuterBlueAutonomous")
public class OuterBlueAutonomous extends LinearOpMode{

    RobotClass robot;


    @Override
    public void runOpMode() {

        robot= new RobotClass(hardwareMap, telemetry, this);

        robot.wobbleGoalGrippyThingGrab();

       // robot.innitDisplayTelemetryGyro();

        waitForStart();

        robot.forward(.8,-2);
//        robot.shooterEngageAlt();
        robot.pause(800);
//        robot.shooterServo1(.8);
//        robot.shooterServo2(.8);
        robot.pause(200);
//        robot.intakeServoEngage(.9);
        robot.pause(4500);
//        robot.shooterStop();
//        robot.shooterServo1Stop();
//        robot.shooterServo2Stop();
//        robot.intakeServoStop();
        robot.forward(.6,-1);
        robot.strafeLeft(.5,2.1);
        robot.forward(.6,-1.85);
        robot.pivotRight(.4,15);
        robot.depositWobbleGoal();
    }
}
