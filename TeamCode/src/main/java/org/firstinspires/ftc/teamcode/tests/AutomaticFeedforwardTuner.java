package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.commandftc.opModes.LinearOpModeWithCommands;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.lib.LoggingUtil;
import org.firstinspires.ftc.teamcode.lib.RegressionUtil;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import java.util.ArrayList;
import java.util.List;

/*
 * Op mode for computing kV, kStatic, and kA from various drive routines. For the curious, here's an
 * outline of the procedure:
 *   1. Slowly ramp the motor power and record encoder values along the way.
 *   2. Run a linear regression on the encoder velocity vs. motor power plot to obtain a slope (kV)
 *      and an optional intercept (kStatic).
 *   3. Accelerate the robot (apply constant power) and record the encoder counts.
 *   4. Adjust the encoder data based on the velocity tuning data and find kA with another linear
 *      regression.
 */
@Config
@Disabled
@Autonomous(group = "tests")
public class AutomaticFeedforwardTuner extends LinearOpModeWithCommands {
    public static double MAX_POWER = 1;
    public static double DISTANCE = 4; // meter
    public static double MAX_VELOCITY = 2;

    private DriveTrainSubsystem drive;

    @Override
    public void init_subsystems() {
        drive = new DriveTrainSubsystem();
    }

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        NanoClock clock = NanoClock.system();

        telemetry.addLine("Press play to begin the feedforward tuning routine");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.addLine("Would you like to fit kStatic?");
        telemetry.addLine("Press (Y/Δ) for yes, (B/O) for no");
        telemetry.update();

        boolean fitIntercept = false;
        while (!isStopRequested()) {
            if (gamepad1.y) {
                fitIntercept = true;
                while (!isStopRequested() && gamepad1.y) {
                    idle();
                }
                break;
            } else if (gamepad1.b) {
                while (!isStopRequested() && gamepad1.b) {
                    idle();
                }
                break;
            }
            idle();
        }

        telemetry.clearAll();
        telemetry.addLine(Misc.formatInvariant(
                "Place your robot on the field with at least %.2f meters of room in front", DISTANCE));
        telemetry.addLine("Press (Y/Δ) to begin");
        telemetry.update();

        while (!isStopRequested() && !gamepad1.y) {
            idle();
        }
        while (!isStopRequested() && gamepad1.y) {
            idle();
        }

        telemetry.clearAll();
        telemetry.addLine("Running...");
        telemetry.update();

        double finalVel = MAX_POWER * MAX_VELOCITY;
        double accel = (finalVel * finalVel) / (2.0 * DISTANCE);
        double rampTime = Math.sqrt(2.0 * DISTANCE / accel);
        telemetry.addData("accel", accel);
        telemetry.addData("rampTime", rampTime);
        telemetry.update();

        List<Double> timeSamples = new ArrayList<>();
        List<Double> positionSamples = new ArrayList<>();
        List<Double> powerSamples = new ArrayList<>();

        drive.setPoseEstimate(new Pose2d());

        double startTime = clock.seconds();
        while (!isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > rampTime) {
                break;
            }
            double vel = accel * elapsedTime;
            double power = vel / MAX_VELOCITY;

            timeSamples.add(elapsedTime);
            positionSamples.add(drive.getPoseEstimate().getX());
            powerSamples.add(power);

            drive.setDrivePower(new Pose2d(power, 0.0, 0.0));
            drive.updatePoseEstimate();
        }
        drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

        RegressionUtil.RampResult rampResult = RegressionUtil.fitRampData(
                timeSamples, positionSamples, powerSamples, fitIntercept,
                LoggingUtil.getLogFile(Misc.formatInvariant(
                        "DriveRampRegression-%d.csv", System.currentTimeMillis())));

        telemetry.clearAll();
        telemetry.addLine("Quasi-static ramp up test complete");
        if (fitIntercept) {
            telemetry.addLine(Misc.formatInvariant("kV = %.5f, kStatic = %.5f (R^2 = %.2f)",
                    rampResult.kV, rampResult.kStatic, rampResult.rSquare));
        } else {
            telemetry.addLine(Misc.formatInvariant("kV = %.5f (R^2 = %.2f)",
                    rampResult.kStatic, rampResult.rSquare));
        }
        telemetry.addLine("Would you like to fit kA?");
        telemetry.addLine("Press (Y/Δ) for yes, (B/O) for no");
        telemetry.update();

        boolean fitAccelFF = false;
        while (!isStopRequested()) {
            if (gamepad1.y) {
                fitAccelFF = true;
                while (!isStopRequested() && gamepad1.y) {
                    idle();
                }
                break;
            } else if (gamepad1.b) {
                while (!isStopRequested() && gamepad1.b) {
                    idle();
                }
                break;
            }
            idle();
        }

        if (fitAccelFF) {
            telemetry.clearAll();
            telemetry.addLine("Place the robot back in its starting position");
            telemetry.addLine("Press (Y/Δ) to continue");
            telemetry.update();

            while (!isStopRequested() && !gamepad1.y) {
                idle();
            }
            while (!isStopRequested() && gamepad1.y) {
                idle();
            }

            telemetry.clearAll();
            telemetry.addLine("Running...");
            telemetry.update();

            double maxPowerTime = DISTANCE / MAX_VELOCITY;

            timeSamples.clear();
            positionSamples.clear();
            powerSamples.clear();

            drive.setPoseEstimate(new Pose2d());
            drive.setDrivePower(new Pose2d(MAX_POWER, 0.0, 0.0));

            startTime = clock.seconds();
            while (!isStopRequested()) {
                double elapsedTime = clock.seconds() - startTime;
                if (elapsedTime > maxPowerTime) {
                    break;
                }

                timeSamples.add(elapsedTime);
                positionSamples.add(drive.getPoseEstimate().getX());
                powerSamples.add(MAX_POWER);

                drive.updatePoseEstimate();
            }
            drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

            RegressionUtil.AccelResult accelResult = RegressionUtil.fitAccelData(
                    timeSamples, positionSamples, powerSamples, rampResult,
                    LoggingUtil.getLogFile(Misc.formatInvariant(
                            "DriveAccelRegression-%d.csv", System.currentTimeMillis())));

            telemetry.clearAll();
            telemetry.addLine("Constant power test complete");
            telemetry.addLine(Misc.formatInvariant("kA = %.5f (R^2 = %.2f)",
                    accelResult.kA, accelResult.rSquare));
            telemetry.update();
        }

        while (!isStopRequested()) {
            idle();
        }
    }
}