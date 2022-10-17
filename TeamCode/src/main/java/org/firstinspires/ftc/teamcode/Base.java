package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Concept: Base", group = "Concept")

public class Base extends LinearOpMode {
    private Robot2022 robot;

    @Override
    public void runOpMode() {
        robot = new Robot2022(hardwareMap, telemetry, gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            robot.update();
            robot.run();
            robot.telemetryUpdate();
            robot.dashboardTelemetryUpdate();
        }
    }
}