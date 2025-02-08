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
        KalmanFilter kalmanFilter = new KalmanFilter(0.7, 9);
        //accurate wheel odometry noise: 0.5, 5
        //accurate sparkfun optical sensor noise: 0.7 9
        //TODO find a way to test the accuracy while running
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
            Point noisyMeasurement1 = new Point(odometrySpark.sparkUpdateData().getX(),odometrySpark.sparkUpdateData().getY());
            Point noisyMeasurement2 =  new Point(wheelOdometry.updatePosition().getX(), wheelOdometry.updatePosition().getY());
            Point filteredPoint1 = kalmanFilter.update(noisyMeasurement1);
            Point filteredPoint2 = kalmanFilter.update(noisyMeasurement2);
            telemetry.addLine("Noisy data");
            telemetry.addLine("x: " + noisyMeasurement1.x + " y: " + noisyMeasurement1.y);
            telemetry.addLine("filtered data");
            telemetry.addLine("x: " + filteredPoint1.x + " y: " + filteredPoint1.y);
            Log.d("kalmanFilter", noisyMeasurement1.x + noisyMeasurement1.y + "  " + filteredPoint1.x + filteredPoint1.y);
            if (gamepad1.a) {
                kalmanFilter.reset();
                //TODO make a function in OdometrySpark to reset sparkfun x and y values
                telemetry.addLine("odometry values reset");
            }
            telemetry.update();
        }
    }
}
