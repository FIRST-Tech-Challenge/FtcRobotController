package com.kalipsorobotics.test;



import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import android.util.Log;

import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.localization.KalmanFilter;
import com.kalipsorobotics.localization.OdometrySpark;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Point;

@TeleOp(name = "KalmanFilterTest")
public class TestKalmanOdometry extends LinearOpMode {
    @Override
    public void runOpMode() {
        SparkFunOTOS otos = hardwareMap.get(SparkFunOTOS.class, "sprk sensor OTOS");
        KalmanFilter kalmanFilter = new KalmanFilter(0.7, 9, 5);
        //accurate wheel odometry noise: 0.5, 5
        //accurate sparkfun optical sensor noise: 0.7 9
        OdometrySpark odometrySpark = new OdometrySpark(otos);
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        DriveAction driveAction = new DriveAction(driveTrain);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        kalmanFilter.reset();
        waitForStart();
//
        while (opModeIsActive()) {
            driveAction.move(gamepad1);
            Point noisyMeasurement1 = new Point((odometrySpark.sparkUpdateData().getX() + wheelOdometry.updatePosition().getX())/2,(odometrySpark.sparkUpdateData().getY() + wheelOdometry.updatePosition().getY())/2);
            //Point noisyMeasurement2 =  new Point(, );
            Point filteredPoint1 = kalmanFilter.update(noisyMeasurement1);
            //Point filteredPoint2 = kalmanFilter.update(noisyMeasurement2);
            //TODO fuse with wheel odo
            //TODO add kalman filter with heading
            telemetry.addLine("Noisy data");
            telemetry.addLine(String.format("x: %.3f y: %.3f", noisyMeasurement1.x, noisyMeasurement1.y));
            telemetry.addLine("filtered data");
            telemetry.addLine(String.format("x: %.3f y: %.3f", filteredPoint1.x, filteredPoint1.y));
            Log.d("kalmanFilter", noisyMeasurement1.x + noisyMeasurement1.y + "  " + filteredPoint1.x + filteredPoint1.y);
            if (gamepad1.a) {
                odometrySpark.sparkResetData(true, odometrySpark.headingUpdateData("right", 0, 0));
                kalmanFilter.reset();
                Log.d("kalmanFilter", "odometry values reset");
            }
            telemetry.update();
        }
    }
}
