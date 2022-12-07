package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoLimits")
@Config

public class test extends LinearOpMode {
    public static double limitOne = 0.992;
    public static double limitTwo = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo one = hardwareMap.get(Servo.class, "claw");

        waitForStart();
        while (opModeIsActive()) {
            // one.setPosition(limitOne);
            telemetry.addData("pos: ", one.getPosition());
            telemetry.update();
        }
    }
}