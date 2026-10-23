package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Sync Test OpMode", group = "Test")
public class BasicTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addData("Sync Status", "Successfully received by team!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running live loop");
            telemetry.update();
        }
    }
}