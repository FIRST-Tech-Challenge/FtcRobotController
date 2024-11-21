package com.kalipsorobotics.test;

import android.util.Log;

import com.kalipsorobotics.actions.CheckPointDone;
import com.kalipsorobotics.actions.MoveLSAction;
import com.kalipsorobotics.actions.outtake.OuttakeClawAutoAction;
import com.kalipsorobotics.actions.outtake.OuttakePivotAutoAction;
import com.kalipsorobotics.localization.Odometry;

import com.kalipsorobotics.math.CalculateTickInches;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.kalipsorobotics.actions.PurePursuitAction;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;

@TeleOp
public class TestTeleOpp extends LinearOpMode {

    DriveTrain driveTrain;
    Odometry odometry;

    OpModeUtilities opModeUtilities;


    @Override
    public void runOpMode() throws InterruptedException {

        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        driveTrain = new DriveTrain(opModeUtilities);
        odometry = new Odometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));
        IMUModule imu = new IMUModule(opModeUtilities);
        Outtake outtake = new Outtake(opModeUtilities);

//        PurePursuitAction purePursuitAction = new PurePursuitAction(driveTrain, odometry);
//        purePursuitAction.addPoint(0,0);
//        purePursuitAction.addPoint(400,0);
////        purePursuitAction.addPoint(1000,0);
////        purePursuitAction.addPoint(400,-400);
//
//        Point checkpoint1 = new Point(100, 0);
//        CheckPointDone checkPointDone = new CheckPointDone(checkpoint1, purePursuitAction, odometry);

        MoveLSAction moveLSToBasket = new MoveLSAction(CalculateTickInches.inchToTicksLS(58), outtake);
        OuttakePivotAutoAction pivotOut = new OuttakePivotAutoAction(outtake, OuttakePivotAutoAction.Position.BASKET);
        pivotOut.setDependentAction(moveLSToBasket);
        OuttakeClawAutoAction clawOpen = new OuttakeClawAutoAction(outtake, OuttakeClawAutoAction.ClawPosition.OPEN);
        clawOpen.setDependentAction(pivotOut);

        waitForStart();

        outtake.outtakeClawServo.setPosition(0.5);

        while (opModeIsActive()) {

//            odometry.updatePosition();
//            purePursuitAction.updateCheckDone();
//            checkPointDone.updateCheckDone();
            moveLSToBasket.updateCheckDone();
            pivotOut.updateCheckDone();
            clawOpen.updateCheckDone();
//
//            if (checkPointDone.getIsDone()) {
//                Log.d("checkpointdone", "done");
//            }
//
//            Log.d("purepursactionlog", odometry.getCurrentPosition().toString());

            //purePursuitAction.updateCheckDone();

        }
    }
}
