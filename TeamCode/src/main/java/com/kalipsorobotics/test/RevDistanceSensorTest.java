package com.kalipsorobotics.test;

import com.kalipsorobotics.modules.RevDistance;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class RevDistanceSensorTest extends LinearOpMode {
    Rev2mDistanceSensor revDistance;
    @Override
    public void runOpMode() throws InterruptedException {
        revDistance = hardwareMap.get(Rev2mDistanceSensor.class, "Rev Distance");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine(revDistance.getDistance(DistanceUnit.MM) + "");
            if (revDistance.getDistance(DistanceUnit.MM) < 40) {
                telemetry.addLine("sample grabbed");
            }
            telemetry.update();
        }
    }
}
