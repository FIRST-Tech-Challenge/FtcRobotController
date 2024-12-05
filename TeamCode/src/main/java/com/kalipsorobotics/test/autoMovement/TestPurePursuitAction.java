package com.kalipsorobotics.test.autoMovement;

import android.util.Log;

import com.kalipsorobotics.actions.AutoActions.PurePursuitAction;
import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestPurePursuitAction extends LinearOpMode {

    DriveTrain driveTrain;
    SparkfunOdometry sparkfunOdometry;
    OpModeUtilities opModeUtilities;


    @Override
    public void runOpMode() throws InterruptedException {

        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        driveTrain = new DriveTrain(opModeUtilities);
        IMUModule imuModule = new IMUModule(opModeUtilities);
        sparkfunOdometry = new SparkfunOdometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));
        WheelOdometry wheelOdometry = new WheelOdometry(opModeUtilities, driveTrain, imuModule,0,0,Math.toRadians(0));

        PurePursuitAction purePursuitAction = new PurePursuitAction(driveTrain, wheelOdometry);
        purePursuitAction.addPoint(0,0,0);
        purePursuitAction.addPoint(600,600,0);
//        purePursuitAction.addPoint(36,36, Math.toRadians(-90));
//        purePursuitAction.addPoint(24,0, Math.toRadians(-90));
//        purePursuitAction.addPoint(1000,0, 0);
//        purePursuitAction.addPoint(400,-400, 0);

        waitForStart();
        while (opModeIsActive()) {
            wheelOdometry.updatePosition();
            Log.d("purepursaction_debug_odo_wheel global", wheelOdometry.getCurrentPosition().toString());

//            purePursuitAction.update();
//
//            if (purePursuitAction.checkDoneCondition()) {
//                break;
//            }


        }
    }

}
