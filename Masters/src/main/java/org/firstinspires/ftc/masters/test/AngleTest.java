package org.firstinspires.ftc.masters.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.masters.components.ITDCons;

@TeleOp(group="Test", name = "Angle Test")
@Config
public class AngleTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo angleServoLeft = hardwareMap.servo.get("angleLeft");
        angleServoLeft = hardwareMap.servo.get("angleLeft");
        Servo angleServoRight = hardwareMap.servo.get("angleRight");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                angleServoLeft.setPosition(ITDCons.angleTransfer);
                angleServoRight.setPosition(ITDCons.angleTransfer);
            }

            if (gamepad1.b) {
                angleServoLeft.setPosition(ITDCons.angleBack);
                angleServoRight.setPosition(ITDCons.angleBack);
            }
            if (gamepad1.x){
                angleServoLeft.setPosition(ITDCons.angleMiddle);
                angleServoRight.setPosition(ITDCons.angleMiddle);
            }
            if (gamepad1.y){
                angleServoLeft.setPosition(ITDCons.angleScoreSpec);
                angleServoRight.setPosition(ITDCons.angleScoreSpec);
            }

        }
    }
}

