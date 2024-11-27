package com.kalipsorobotics.test.autoMovement;

import android.util.Log;

import com.kalipsorobotics.actions.PurePursuitAction;
import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class NewTestPurePursuit extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        IMUModule imuModule = new IMUModule(opModeUtilities);
        sleep(1000);
        WheelOdometry wheelOdometry = new WheelOdometry(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        PurePursuitAction purePursuitAction = new PurePursuitAction(driveTrain, wheelOdometry);
        purePursuitAction.addPoint(0, 0, 0);
        purePursuitAction.addPoint(600, -600, 90);

        Log.d("pidangle", purePursuitAction.getPidAngle().toString());
        while (opModeInInit()) {
            if (gamepad1.dpad_up) {
                purePursuitAction.incrementAnglePID(0.00001, 0., 0.);
            }
            if (gamepad1.dpad_down) {
                purePursuitAction.incrementAnglePID(-0.00001, 0., 0.);
            }
            if (gamepad1.dpad_left) {
                purePursuitAction.incrementAnglePID(0., -0.00001, 0.);
            }
            if (gamepad1.dpad_right) {
                purePursuitAction.incrementAnglePID(0., 0.00001, 0.);
            }

        }

        waitForStart();
        while (opModeIsActive()) {

            wheelOdometry.updatePosition();

            purePursuitAction.update();

            if (purePursuitAction.checkDoneCondition()) {
                break;
            }
        }

    }
}
