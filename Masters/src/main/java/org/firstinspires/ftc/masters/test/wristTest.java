package org.firstinspires.ftc.masters.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.masters.components.ITDCons;
import org.firstinspires.ftc.masters.old.CSCons;

@TeleOp(name = "Wrist Test")
@Config
public class wristTest extends LinearOpMode {

    private Servo wristServo;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        wristServo = hardwareMap.servo.get("wrist");
        wristServo.setPosition(ITDCons.wristFront);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                wristServo.setPosition(ITDCons.wristFront);
            }

            if (gamepad1.b) {
                wristServo.setPosition(ITDCons.wristBack);
            }

        }
    }
}

