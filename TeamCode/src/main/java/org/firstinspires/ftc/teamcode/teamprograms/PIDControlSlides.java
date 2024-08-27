package org.firstinspires.ftc.teamcode.teamprograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Elementary, my dear Watson!
 * (taken from Reddit, the wellspring of internet knowledge)
 * A rule-of-thumb process:
 * 1.
 * Set I and D to 0. Increase P until you get oscillation off the system. Then cut P in half
 * 2.
 * increase I until you are returning to the set-point quick enough for your application. Too much integral gain can cause a lot of overshoot.
 * 3.
 * increase D until you’ve started to dampen your response as you approach the set point.
 * -
 * -
 * -
 * Since a custom PID control algorithm was not created, here is the gist of how it does it's magic:
 * -
 *     error = reference - encoderPosition;
 * -
 *     derivative = (error - lastError) / timer.seconds(); // rate of change of the error
 * -
 * -
 *     integralSum = integralSum + (error * timer.seconds()); // sum of all error over time
 * -
 *     out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
 */

@TeleOp(group = "PID")
@Config
public class PIDControlSlides extends LinearOpMode {

    public static double kp = 0, ki = 0, kd = 0; // PID coefficients
    public static double kg = 0; // constant power to eliminate the effect of gravity
    public static int targetPos = 0;

    private PIDController controller;
    private DcMotorEx slide;


    @Override
    public void runOpMode() {

        controller = new PIDController(kp, ki, kd);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            int slidePos = slide.getCurrentPosition();
            double slidePID = controller.calculate(slidePos, targetPos);

            double power = slidePID + kg;

            slide.setPower(power);

            telemetry.addData("Target Position", targetPos);
            telemetry.addData("Slide Position", slidePos);
            telemetry.update();

        }


    }

}
