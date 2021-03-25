package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Date;

@Autonomous(name = "OuterRedAutonomous")
public class OuterRedAutonomous extends LinearOpMode {

    RobotClass robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot= new RobotClassInnerBlueOuterRed(hardwareMap, telemetry, this, "red");
        robot.wobbleGoalGrippyThingGrab();
        robot.openCVInnitShenanigans();

        RobotClass.RingPosition ringNmb = null;

        waitForStart();
        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            ringNmb = robot.analyze();

            telemetry.addData("Position", ringNmb);
            telemetry.update();
        }
        robot.forward(.1, -0.3);
        robot.strafeLeft(.3,1);
        robot.forward(.3,-1.7);
        robot.pivotLeft(.3,28);
        robot.shooterEngage();
        robot.pause(800);

        robot.pause(200);
        robot.intakeServoEngage(.9);
        robot.pause(4500);
        robot.shooterStop();
        robot.intakeServoStop();
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

//        if (ringNmb == RobotClass.RingPosition.NONE) {
//            robot.pivotRight(.5, 28);
//            robot.strafeRight(0.5, 2);
//            robot.forward(0.5, -4);
//            robot.pivotRight(0.4, 170);
//            robot.pause(500);
//            robot.depositWobbleGoal();
//        } else if (ringNmb == RobotClass.RingPosition.ONE) {
//            robot.pivotRight(.5, 28);
//            robot.forward(0.3, -.6);
////            robot.intakeServoEngage(.9);
//            robot.pivotRight(0.3,90);
//            robot.forward(0.5,2);
//            robot.forward(0.5, 1.8);
////            robot.shooterServo1(.8);
////            robot.shooterServo2(.8);
////            robot.pause(1000);
////            robot.shooterServo1Stop();
////            robot.shooterServo2Stop();
////            robot.intakeServoStop();
//            robot.pivotLeft(0.3, 90);
//            robot.forward(0.3, .6);
//            robot.pivotLeft(.3,28);
//
////            robot.shooterEngage();
////            robot.pause(800);
////            robot.shooterServo1(.8);
////            robot.shooterServo2(.8);
////            robot.pause(200);
////            robot.intakeServoEngage(.9);
////            robot.pause(4500);
////            robot.shooterStop();
////            robot.shooterServo1Stop();
////            robot.shooterServo2Stop();
////            robot.intakeServoStop();
//
//            robot.pivotRight(.5, 28);
//            robot.forward(.4,-6.5);
//            robot.depositWobbleGoal();
//            robot.forward(0.5, 2);
//        } else if (ringNmb == RobotClass.RingPosition.FOUR) {
//            robot.pause(4000);
//            robot.pivotRight(.3, 28);
//            robot.forward(.5,-6.15);
//            robot.pivotRight(0.4,90);
//            robot.pause(500);
//           // robot.forward(0.2,-1);
//            robot.depositWobbleGoal();
//            robot.strafeLeft(0.5,2.2);
//        }

    }
}
