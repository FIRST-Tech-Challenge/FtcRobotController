package com.kalipsorobotics.test;

import com.kalipsorobotics.localization.OdometrySpark;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class TestKalmanOdometry extends LinearOpMode {
    OdometrySpark odometry;
    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeIsActive()) {
            telemetry.addLine("" + odometry.sparkUpdateFiltered());
            telemetry.update()
        }
    }
}
