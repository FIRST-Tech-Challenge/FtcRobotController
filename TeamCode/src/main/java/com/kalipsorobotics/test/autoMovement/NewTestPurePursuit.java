package com.kalipsorobotics.test.autoMovement;

import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="pptest")
public class NewTestPurePursuit extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        PurePursuitAction test = new PurePursuitAction(driveTrain, wheelOdometry);
        test.addPoint(0, 0, 0);
        test.addPoint(50, 0, 0);
        test.addPoint(1200, 0, 0);
        test.addPoint(800, 300, 0);
        test.addPoint(500, 300, 90);
        test.addPoint(200, 300, 0);
        test.addPoint(0, 0, 0);



        waitForStart();
        while (opModeIsActive()) {

            wheelOdometry.updatePosition();

            test.update();

            if (test.checkDoneCondition()) {
                break;
            }
        }

    }
}
