package org.firstinspires.ftc.masters.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config // Enables FTC Dashboard
@TeleOp(name = "LED Test")
public class ledTest extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double Blank = 0;

    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.update();

        Servo servo = hardwareMap.servo.get("servo");

        waitForStart();

        while (opModeIsActive()) {

            while (true){

                servo.setPosition(.611);
                sleep(1000);
                servo.setPosition(.722);
                sleep(1000);
                servo.setPosition(1);
                sleep(1000);
                servo.setPosition(.722);
                sleep(1000);
                servo.setPosition(.611);
                sleep(1000);

            }

        }
    }
}

