package org.firstinspires.ftc.teamcode.Swerve.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Core.HWMap;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class SwervePIDTune extends LinearOpMode {

    HWMap hwMap;
    public static double targetAngle = 0;

    private PIDFController pidfController;


    public static double P = 0.015, I = 0.0, D = 0.005, F = 0.0;

    public static double direction = -1.0;

    private CRServoImplEx FLS;
    private AnalogInput FLE;

    Timing.Timer timer;
    public static long sleepTime = 25;


    @Override
    public void runOpMode() throws InterruptedException {

        timer = new Timing.Timer(3000000, TimeUnit.MILLISECONDS);
        hwMap = new HWMap(hardwareMap);
        FLS = hwMap.FLS;
        FLE = hwMap.FLE;

        // Setup Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Setup PID
        pidfController = new PIDFController(P, I, D, F);

        waitForStart();

        while (opModeIsActive()) {
            pidfController.setPIDF(P, I, D, F);

            double currentPos = (FLE.getVoltage() / 3.3) * 360;

            double error = AngleUnit.normalizeDegrees(targetAngle - currentPos);

            double power = pidfController.calculate(0, error);

            double safePower = Math.max(-1, Math.min(1, power * direction));

            FLS.setPower(safePower);

            telemetry.addData("Target", targetAngle);
            telemetry.addData("Current", currentPos);
            telemetry.addData("Error", error);
            telemetry.addData("Power", safePower);
            telemetry.update();
            sleep(sleepTime);

        }
    }
}