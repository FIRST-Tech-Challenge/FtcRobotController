package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

@TeleOp(name = "info Test")
public class ConnectionDataTest extends TeleOpTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        initAll();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Info", frontDistanceSensor.getConnectionInfo());
            telemetry.addData("Distance", frontDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
