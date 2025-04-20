package com.kalipsorobotics.odometryDataPaths;

import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.localization.OdometryFileWriter;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@TeleOp
public class Straight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        OdometryFileWriter odometryFileWriter = new OdometryFileWriter("Straight", opModeUtilities);

        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);

        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);

        WheelOdometry.setInstanceNull();
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        SharedData.resetOdometryPosition();


        PurePursuitAction moveStraight600MM = new PurePursuitAction(driveTrain, wheelOdometry);
        moveStraight600MM.addPoint(600, 0, 0);

        ExecutorService executorService = Executors.newSingleThreadExecutor();
        OpModeUtilities.runOdometryExecutorService(executorService, wheelOdometry);


        waitForStart();
        while (opModeIsActive()) {

            odometryFileWriter.writeOdometryPositionHistory(SharedData.getOdometryPositionMap());
            moveStraight600MM.updateCheckDone();

            //Log.d("odometryData", "currentPos" + SharedData.getOdometryPosition().toString());
        }
        odometryFileWriter.close();
        OpModeUtilities.shutdownExecutorService(executorService);
    }
}
