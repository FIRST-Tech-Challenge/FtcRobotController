package org.firstinspires.ftc.masters.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config // Enables FTC Dashboard
@TeleOp(name = "BoooServoTest")
public class ServoTest extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    Servo servo = hardwareMap.servo.get("servo");

    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            servo.setPosition(.5);

        }
    }
}

