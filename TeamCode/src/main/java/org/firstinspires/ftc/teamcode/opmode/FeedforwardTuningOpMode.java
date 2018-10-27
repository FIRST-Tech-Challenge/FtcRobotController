package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveSimple;

import java.util.ArrayList;
import java.util.List;

/*
 * Op mode for computing kV, kStatic, and kA from various drive routines. Note: for those using the
 * built-in PID, **kStatic and kA should not be tuned**. For the curious, here's an outline of the
 * basic procedure:
 *   1. Slowly ramp the motor power and record encoder values along the way.
 *   2. Run a linear regression on the encoder velocity vs. motor power plot to obtain a slope (kV)
 *      and an optional intercept (kStatic).
 *   3. Accelerate the robot (apply max power) and record the encoder counts.
 *   4. Adjust the encoder data based on the velocity tuning data and find kA with another linear
 *      regression.
 */
@Config
@Autonomous
public class FeedforwardTuningOpMode extends LinearOpMode {
    private static final double EPSILON = 1e-2;

    public static final double MAX_POWER = 0.7;
    public static final double DISTANCE = 100;

    private static List<Double> numericalDerivative(List<Double> x, List<Double> y) {
        List<Double> deriv = new ArrayList<>();
        for (int i = 2; i < x.size(); i++) {
            deriv.add((y.get(i) - y.get(i-2)) / (x.get(i) - x.get(i-2)));
        }
        deriv.add(0, deriv.get(0));
        deriv.add(deriv.get(deriv.size() - 1));
        return deriv;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveSimple drive = new SampleMecanumDriveSimple(hardwareMap);

        NanoClock clock = NanoClock.system();

        telemetry.log().add("Press play to begin the feedforward tuning routine");
        telemetry.update();

        waitForStart();

        telemetry.log().clear();
        telemetry.log().add("Would you like to fit kStatic?");
        telemetry.log().add("Press (A) for yes, (B) for no");
        telemetry.update();

        boolean fitIntercept = false;
        while (opModeIsActive()) {
            if (gamepad1.a) {
                fitIntercept = true;
                while (opModeIsActive() && gamepad1.a) {
                    idle();
                }
                break;
            } else if (gamepad1.b) {
                while (opModeIsActive() && gamepad1.b) {
                    idle();
                }
                break;
            }
            idle();
        }

        telemetry.log().clear();
        telemetry.log().add(String.format("Place your robot on the field with at least %.2f in of room in front", DISTANCE));
        telemetry.log().add("Press (A) to begin");
        telemetry.update();

        while (opModeIsActive() && !gamepad1.a) {
            idle();
        }
        while (opModeIsActive() && gamepad1.a) {
            idle();
        }

        telemetry.log().clear();
        telemetry.log().add("Running...");
        telemetry.update();

        double maxRpm = DriveConstants.MOTOR_CONFIG.getMaxRPM();
        double maxVel = maxRpm * DriveConstants.GEAR_RATIO * 2 * Math.PI * DriveConstants.WHEEL_RADIUS / 60.0;
        double finalVel = MAX_POWER * maxVel;
        double accel = (finalVel * finalVel) / (2.0 * DISTANCE);
        double rampTime = Math.sqrt(2.0 * DISTANCE / accel);

        double startTime = clock.seconds();
        List<Double> timeSamples = new ArrayList<>();
        List<Double> powerSamples = new ArrayList<>();
        List<Double> positionSamples = new ArrayList<>();

        drive.setPoseEstimate(new Pose2d());
        while (opModeIsActive()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > rampTime) {
                break;
            }
            double vel = accel * elapsedTime;
            double power = vel / maxVel;

            timeSamples.add(elapsedTime);
            powerSamples.add(power);
            positionSamples.add(drive.getPoseEstimate().getX());

            drive.setVelocity(new Pose2d(power, 0.0, 0.0));
            drive.updatePoseEstimate();
        }
        drive.setVelocity(new Pose2d(0.0, 0.0, 0.0));

        List<Double> velocitySamples = numericalDerivative(timeSamples, positionSamples);
        SimpleRegression rampRegression = new SimpleRegression(fitIntercept);
        for (int i = 0; i < velocitySamples.size(); i++) {
            rampRegression.addData(velocitySamples.get(i), powerSamples.get(i));
        }
        double kV = rampRegression.getSlope();
        double kStatic = rampRegression.getIntercept();

        telemetry.log().clear();
        telemetry.log().add("Quasi-static ramp up test complete");
        if (fitIntercept) {
            telemetry.log().add(String.format("kV = %.5f, kStatic = %.5f (R^2 = %.2f)", kV, kStatic, rampRegression.getRSquare()));
        } else {
            telemetry.log().add(String.format("kV = %.5f (R^2 = %.2f)", kV, rampRegression.getRSquare()));
        }
        telemetry.log().add("Would you like to fit kA?");
        telemetry.log().add("Press (A) for yes, (B) for no");
        telemetry.update();

        boolean fitAccelFF = false;
        while (opModeIsActive()) {
            if (gamepad1.a) {
                fitAccelFF = true;
                while (opModeIsActive() && gamepad1.a) {
                    idle();
                }
                break;
            } else if (gamepad1.b) {
                while (opModeIsActive() && gamepad1.b) {
                    idle();
                }
                break;
            }
            idle();
        }

        if (fitAccelFF) {
            telemetry.log().clear();
            telemetry.log().add("Place the robot back in its starting position");
            telemetry.log().add("Press (A) to continue");
            telemetry.update();

            while (opModeIsActive() && !gamepad1.a) {
                idle();
            }
            while (opModeIsActive() && gamepad1.a) {
                idle();
            }

            telemetry.log().clear();
            telemetry.log().add("Running...");
            telemetry.update();

            double maxPowerTime = DISTANCE / maxVel;

            startTime = clock.seconds();
            timeSamples.clear();
            positionSamples.clear();

            drive.setPoseEstimate(new Pose2d());
            drive.setVelocity(new Pose2d(MAX_POWER, 0.0, 0.0));
            while (opModeIsActive()) {
                double elapsedTime = clock.seconds() - startTime;
                if (elapsedTime > maxPowerTime) {
                    break;
                }

                timeSamples.add(elapsedTime);
                positionSamples.add(drive.getPoseEstimate().getX());

                drive.updatePoseEstimate();
            }
            drive.setVelocity(new Pose2d(0.0, 0.0, 0.0));

            velocitySamples = numericalDerivative(timeSamples, positionSamples);
            List<Double> accelerationSamples = numericalDerivative(timeSamples, velocitySamples);

            SimpleRegression maxPowerRegression = new SimpleRegression(false);
            for (int i = 0; i < accelerationSamples.size(); i++) {
                double velocityPower = kV * velocitySamples.get(i);
                if (Math.abs(velocityPower) > EPSILON) {
                    velocityPower += Math.signum(velocityPower) * kStatic;
                } else {
                    velocityPower = 0;
                }
                double accelerationPower = MAX_POWER - velocityPower;
                maxPowerRegression.addData(accelerationSamples.get(i), accelerationPower);
            }
            double kA = maxPowerRegression.getSlope();

            telemetry.log().clear();
            telemetry.log().add("Max power test complete");
            telemetry.log().add(String.format("kA = %.5f (R^2 = %.2f)", kA, maxPowerRegression.getRSquare()));
            telemetry.update();
        }

        while (opModeIsActive()) {
            idle();
        }
    }
}