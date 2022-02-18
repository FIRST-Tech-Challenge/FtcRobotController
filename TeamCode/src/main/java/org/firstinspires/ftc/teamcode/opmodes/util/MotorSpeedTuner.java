package org.firstinspires.ftc.teamcode.opmodes.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoCarousel;

@TeleOp
@Config
public class MotorSpeedTuner extends LinearOpMode {
    public static double motorSpeed = 0.45D;
    public static String motorName = AutoCarousel.carouselName;

    @Override
    public void runOpMode() {
        final DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        waitForStart();
        while (!isStopRequested()) {
            motor.setPower(motorSpeed);
        }
    }
}
