package com.kalipsorobotics.test;

import android.util.Log;

import com.kalipsorobotics.actions.drivetrain.MoveRobotStraightInchesAction;
import com.kalipsorobotics.localization.SparkfunOdometry;

import com.kalipsorobotics.localization.WheelOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.kalipsorobotics.actions.PurePursuitAction;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.OpModeUtilities;

@TeleOp
public class TestTeleOpp extends LinearOpMode {

    DriveTrain driveTrain;
    SparkfunOdometry sparkfunOdometry;
    WheelOdometry wheelOdometry;

    OpModeUtilities opModeUtilities;


    @Override
    public void runOpMode() throws InterruptedException {

        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        driveTrain = new DriveTrain(opModeUtilities);
        sparkfunOdometry = new SparkfunOdometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));
        wheelOdometry = new WheelOdometry(driveTrain,opModeUtilities,0,0,Math.toRadians(0));
        //Outtake outtake = new Outtake(opModeUtilities);

//        PurePursuitAction purePursuitAction = new PurePursuitAction(driveTrain, sparkfunOdometry, wheelOdometry);
//        purePursuitAction.addPoint(0,0,0);
//        purePursuitAction.addPoint(12,12,Math.toRadians(-45));
//        purePursuitAction.addPoint(36,36, Math.toRadians(-90));
//        purePursuitAction.addPoint(24,0, Math.toRadians(-90));
        //x=24.251404795259038, y=-23.87280174910445, theta=-0.031638353750141346
//        purePursuitAction.addPoint(1000,0);
//        purePursuitAction.addPoint(400,-400);

        MoveRobotStraightInchesAction moveRobotStraightInchesAction = new MoveRobotStraightInchesAction(24, driveTrain, sparkfunOdometry, wheelOdometry, 0);
//        Point checkpoint1 = new Point(100, 0);
//        CheckPointDone checkPointDone = new CheckPointDone(checkpoint1, purePursuitAction, odometry);
        //AutoBasketAction autoBasketAction = new AutoBasketAction(outtake);

        waitForStart();

        //outtake.outtakeClawServo.setPosition(0.5);

        while (opModeIsActive()) {

            sparkfunOdometry.updatePosition();
            wheelOdometry.updatePosition();
            moveRobotStraightInchesAction.updateCheckDone();

            //driveTrain.setPower(0.2,-0.2,0.2,-0.2);
//            checkPointDone.updateCheckDone();
            //autoBasketAction.updateCheckDone();

//
//            if (checkPointDone.getIsDone()) {
//                Log.d("checkpointdone", "done");
//            }

            //Log.d("purepursaction_debug_odo", sparkfunOdometry.getCurrentPosition().toString());

            //purePursuitAction.updateCheckDone();

        }
    }
}
