package com.acmerobotics.splinelib.drive;

import com.acmerobotics.splinelib.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.stat.regression.SimpleRegression;

import java.util.ArrayList;
import java.util.List;

// TODO: kV seems to be double what it should actually be
public abstract class FeedforwardTuningOpMode extends LinearOpMode {

    private double distance;
    private double wheelMotorRpm;
    private double wheelDiameter;
    private double wheelGearRatio;

    public FeedforwardTuningOpMode(double distance, double wheelMotorRpm, double wheelDiameter, double wheelGearRatio) {
        this.distance = distance;
        this.wheelMotorRpm = wheelMotorRpm;
        this.wheelDiameter = wheelDiameter;
        this.wheelGearRatio = wheelGearRatio;
    }

    public FeedforwardTuningOpMode(double distance, double wheelMotorRpm, double wheelDiameter) {
        this(distance, wheelMotorRpm, wheelDiameter, 1.0);
    }

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
        Drive drive = initDrive();

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
                break;
            } else if (gamepad1.b) {
                break;
            }
            idle();
        }

        telemetry.log().clear();
        telemetry.log().add(String.format("Place your robot on the field with at least %.2f in of room in front", distance));
        telemetry.log().add("Press (A) to begin");
        telemetry.update();

        while (opModeIsActive() && !gamepad1.a) {
            idle();
        }

        telemetry.log().clear();
        telemetry.log().add("Running...");
        telemetry.update();

        double maxVel = wheelMotorRpm * wheelGearRatio * Math.PI * wheelDiameter / 60.0;
        double finalVel = 0.7 * maxVel;
        double accel = (finalVel * finalVel) / (2.0 * distance);
        double t = Math.sqrt(2.0 * distance / accel);

        double startTime = System.nanoTime() / 1e9;
        List<Double> timeSamples = new ArrayList<>();
        List<Double> powerSamples = new ArrayList<>();
        List<Double> positionSamples = new ArrayList<>();

        drive.resetPoseEstimate(new Pose2d());
        while (true) {
            double elapsedTime = System.nanoTime() / 1e9 - startTime;
            if (elapsedTime > t) {
                drive.setVelocity(new Pose2d(0.0, 0.0, 0.0));
                break;
            }
            double vel = accel * elapsedTime;
            double power = vel / maxVel;

            timeSamples.add(elapsedTime);
            powerSamples.add(power);
            positionSamples.add(drive.getPoseEstimate().x());

            drive.setVelocity(new Pose2d(power, 0.0, 0.0));
            drive.updatePoseEstimate(elapsedTime);
        }

        List<Double> velocitySamples = numericalDerivative(timeSamples, positionSamples);
        SimpleRegression regression = new SimpleRegression(fitIntercept);
        for (int i = 0; i < velocitySamples.size(); i++) {
            regression.addData(velocitySamples.get(i), powerSamples.get(i));
        }
        double kV = regression.getSlope();
        double kStatic = regression.getIntercept();

        telemetry.log().clear();
        telemetry.log().add("Quasi-static ramp up test complete");
        if (fitIntercept) {
            telemetry.log().add(String.format("kV = %.5f, kStatic = %.5f (R^2 = %.2f)", kV, kStatic, regression.getRSquare()));
        } else {
            telemetry.log().add(String.format("kV = %.5f (R^2 = %.2f)", kV, regression.getRSquare()));
        }
        telemetry.log().add("Place the robot back in its starting position");
        telemetry.log().add("Press (A) to continue");
        telemetry.update();

        // TODO: add kA tuning routine

        while (opModeIsActive()) {
            idle();
        }
    }

    protected abstract Drive initDrive();
}