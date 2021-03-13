package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Date;

@Autonomous(name = "OuterBlueAutonomous")
public class OuterBlueAutonomous extends LinearOpMode{

    RobotClass robot;


    @Override
    public void runOpMode() {

        robot= new RobotClass(hardwareMap, telemetry, this, "blue");

        robot.wobbleGoalGrippyThingGrab();

       // robot.innitDisplayTelemetryGyro();
        robot.openCVInnitShenanigans();

        RobotClass.RingPosition ringNmb = null;

        waitForStart();

        robot.forward(0.1, -0.3);
        robot.strafeLeft(0.4, 1.7);
        long startTime = new Date().getTime();
        long time = 0;

        while (time < 500 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            ringNmb = robot.analyze();

            telemetry.addData("Position", ringNmb);
            telemetry.update();
        }
        robot.strafeRight(0.4,1.9);
        robot.forward(.5,-1.7);
        robot.shooterEngageAlt();
        robot.pause(800);
        robot.shooterServo1(.8);
        robot.shooterServo2(.8);
        robot.pause(200);
        robot.intakeServoEngage(.9);
        robot.pause(4500);
        robot.shooterStop();
        robot.shooterServo1Stop();
        robot.shooterServo2Stop();
        robot.intakeServoStop();
        if (ringNmb == RobotClass.RingPosition.NONE) {
            robot.forward(.5, -1);
            robot.strafeLeft(.5, 2.1);
            robot.forward(.5, -2.4);
            //robot.pivotRight(.4, 15);
            robot.depositWobbleGoal();
        } else if (ringNmb == RobotClass.RingPosition.ONE) {
            robot.forward(.6,-5.6);
            robot.pivotRight(.4,170);
            robot.depositWobbleGoal();
            robot.forward(0.5,-2);
        } else if (ringNmb == RobotClass.RingPosition.FOUR) {
            robot.forward(.6,-6);
            robot.pivotRight(.4,90);
            robot.depositWobbleGoal();
            robot.strafeLeft(0.5,3);

        }
    }
}
