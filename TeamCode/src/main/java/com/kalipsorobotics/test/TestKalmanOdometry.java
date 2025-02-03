package com.kalipsorobotics.test;



import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

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
        //SparkFunOTOS otos = hardwareMap.get(SparkFunOTOS.class, "sprk sensor OTOS");
        KalmanFilter kalmanFilter = new KalmanFilter(0.5, 5); //accuate wheel odometry                                   ``); //0.2 and 8 for wheel odometry
        //OdometrySpark odometrySpark = new OdometrySpark(otos);
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        kalmanFilter.reset();
        waitForStart();
//
        while (opModeIsActive()) {
            //Point noisyMeasurement = new Point(odometrySpark.sparkUpdateData().getX(),odometrySpark.sparkUpdateData().getY());
            Point noisyMeasurement =  new Point(wheelOdometry.updatePosition().getX(), wheelOdometry.updatePosition().getY());
            // Filter the measurement
            Point filteredPoint = kalmanFilter.update(noisyMeasurement);
            telemetry.addLine("Noisy data");
            telemetry.addLine("x: " + noisyMeasurement.x + " y: " + noisyMeasurement.y);
            telemetry.addLine("filtered data");
            telemetry.addLine("x: " + filteredPoint.x + " y: " + filteredPoint.y);
            if (gamepad1.a) {
                kalmanFilter.reset();
                telemetry.addLine("odometry values reset");
            }
            telemetry.update();
        }
    }
}
