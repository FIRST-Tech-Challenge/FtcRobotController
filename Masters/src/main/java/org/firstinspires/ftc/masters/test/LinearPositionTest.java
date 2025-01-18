package org.firstinspires.ftc.masters.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.masters.components.ITDCons;

@TeleOp(group = "Test", name = "Linear Position Test")
@Config
public class LinearPositionTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo position = hardwareMap.servo.get("position");


        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                position.setPosition(ITDCons.positionFront);
            }

            if (gamepad1.b) {
                position.setPosition(ITDCons.positionBack);
            }

        }
    }
}

