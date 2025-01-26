package com.kalipsorobotics.test;

import com.kalipsorobotics.modules.RevDistance;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RevDistanceSensorTest extends LinearOpMode {
    RevDistance revDistance = new RevDistance(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine(revDistance.getDistance() + "");
            telemetry.update();
        }
    }
}
