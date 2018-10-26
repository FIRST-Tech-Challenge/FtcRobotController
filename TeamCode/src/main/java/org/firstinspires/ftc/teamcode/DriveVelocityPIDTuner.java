package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionConstraints;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

/*
 * This routine is designed to tune the PIDF coefficients used by the REV Expansion Hubs for closed-
 * loop velocity control. Although it may seem unnecessary, tuning these coefficients is just as
 * important as the positional parameters. Like the other manual tuning routines, this op mode
 * relies heavily upon the dashboard. To access the dashboard, connect your computer to the RC's
 * WiFi network and navigate to https://192.168.49.1:8080/dash in your browser. Once you've
 * successfully connected, start the program, and your robot will begin moving forward and backward
 * according to a motion profile. Your job is to graph the velocity errors over time and adjust the
 * PIDF coefficients (it's highly suggested to leave F at its default value) like any normal PID
 * controller. Once you've found a satisfactory set of gains, add them to your drive class init.
 */
@Config
@Autonomous
public class DriveVelocityPIDTuner extends LinearOpMode {
    public static PIDFCoefficients MOTOR_PIDF = new PIDFCoefficients();
    public static double DISTANCE = 72;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        SampleMecanumDriveSimple drive = new SampleMecanumDriveSimple(hardwareMap);

        PIDFCoefficients currentCoeffs = drive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        MOTOR_PIDF = pidfCopy(currentCoeffs);
        dashboard.updateConfig();

        RobotLog.i("Initial motor PIDF coefficients: " + MOTOR_PIDF);

        NanoClock clock = NanoClock.system();

        telemetry.log().add("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        MotionProfile activeProfile = null;
        boolean movingForwards = false;

        List<Double> lastWheelPositions = null;
        double lastTimestamp = 0;
        double profileStartTimestamp = clock.seconds();

        while (opModeIsActive()) {
            // update the coefficients if necessary
            if (!pidfEquals(currentCoeffs, MOTOR_PIDF)) {
                RobotLog.i("Updated motor PIDF coefficients: " + MOTOR_PIDF);
                currentCoeffs = pidfCopy(MOTOR_PIDF);
                drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_PIDF);
            }

            // calculate and set the motor power
            double profileTime = clock.seconds() - profileStartTimestamp;
            double dt = profileTime - lastTimestamp;
            lastTimestamp = profileTime;
            if (profileTime > activeProfile.duration()) {
                // generate a new profile
                movingForwards = !movingForwards;
                MotionState start = new MotionState(movingForwards ? 0 : DISTANCE, 0, 0, 0);
                MotionState goal = new MotionState(movingForwards ? DISTANCE : 0, 0, 0, 0);
                activeProfile = MotionProfileGenerator.generateMotionProfile(start, goal, new MotionConstraints() {
                    @Override
                    public double maximumVelocity(double v) {
                        return SampleMecanumDriveBase.BASE_CONSTRAINTS.maximumVelocity;
                    }

                    @Override
                    public double maximumAcceleration(double v) {
                        return SampleMecanumDriveBase.BASE_CONSTRAINTS.maximumAcceleration;
                    }
                });
                profileStartTimestamp = clock.seconds();
            }
            MotionState motionState = activeProfile.get(profileTime);
            double targetPower = SampleMecanumDriveBase.kV * motionState.getV();
            drive.setVelocity(new Pose2d(targetPower, 0, 0));

            List<Double> wheelPositions = drive.getWheelPositions();
            if (lastWheelPositions != null) {
                // compute velocities
                List<Double> syntheticVelocities = new ArrayList<>();
                for (int i = 0; i < wheelPositions.size(); i++) {
                    syntheticVelocities.add((wheelPositions.get(i) - lastWheelPositions.get(i)) / dt);
                }

                // update telemetry
                telemetry.addData("targetVelocity", motionState.getV());
                for (int i = 0; i < syntheticVelocities.size(); i++) {
                    telemetry.addData("velocity" + i, syntheticVelocities.get(i));
                    telemetry.addData("error" + i, motionState.getV() - syntheticVelocities.get(i));
                }
                telemetry.update();
            }
            lastWheelPositions = wheelPositions;
        }
    }

    private static boolean pidfEquals(PIDFCoefficients coeff1, PIDFCoefficients coeff2) {
        return coeff1.p == coeff2.p && coeff1.i == coeff1.i && coeff1.d == coeff2.d &&
                coeff1.f == coeff2.f && coeff1.algorithm == coeff2.algorithm;
    }

    private static PIDFCoefficients pidfCopy(PIDFCoefficients coeff) {
        return new PIDFCoefficients(coeff.p, coeff.i, coeff.d, coeff.f, coeff.algorithm);
    }
}
