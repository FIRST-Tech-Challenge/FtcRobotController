package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveREV;

import java.util.ArrayList;
import java.util.List;

/*
 * This routine is designed to tune the PID coefficients used by the REV Expansion Hubs for closed-
 * loop velocity control. Although it may seem unnecessary, tuning these coefficients is just as
 * important as the positional parameters. Like the other manual tuning routines, this op mode
 * relies heavily upon the dashboard. To access the dashboard, connect your computer to the RC's
 * WiFi network and navigate to https://192.168.49.1:8080/dash in your browser. Once you've
 * successfully connected, start the program, and your robot will begin moving forward and backward
 * according to a motion profile. Your job is to graph the velocity errors over time and adjust the
 * PID coefficients (it's highly suggested to leave F at its default value) like any normal PID
 * controller. Once you've found a satisfactory set of gains, add them to your drive class init.
 */
@Config
@Autonomous(group = "drive")
public class DriveVelocityPIDTuner extends LinearOpMode {
    public static PIDCoefficients MOTOR_PID = new PIDCoefficients();
    public static double DISTANCE = 72;

    /*
     * If true, the kV value is computed from the free speed determined by the manufacturer (likely
     * an overestimate of the actual value. If false, the value from DriveConstants.kV is used.
     */
    public static boolean USE_THEORETICAL_KV = true;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        PIDCoefficients currentCoeffs = drive.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidCopy(currentCoeffs, MOTOR_PID);
        dashboard.updateConfig();

        RobotLog.i("Initial motor PID coefficients: " + MOTOR_PID);

        NanoClock clock = NanoClock.system();

        telemetry.log().add("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        MotionProfile activeProfile = new MotionProfile();
        boolean movingForwards = false;

        List<Double> lastWheelPositions = null;
        double lastTimestamp = 0;
        double profileStartTimestamp = clock.seconds();

        double maxVel = DriveConstants.rpmToVelocity(DriveConstants.getMaxRpm());
        double kV = USE_THEORETICAL_KV ? (1.0 / maxVel) : DriveConstants.kV;

        while (!isStopRequested()) {
            // update the coefficients if necessary
            if (!pidEquals(currentCoeffs, MOTOR_PID)) {
                RobotLog.i("Updated motor PID coefficients: " + MOTOR_PID);
                pidCopy(MOTOR_PID, currentCoeffs);
                drive.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_PID);
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
                activeProfile = MotionProfileGenerator.generateSimpleMotionProfile(start, goal,
                        DriveConstants.BASE_CONSTRAINTS.maxVel, DriveConstants.BASE_CONSTRAINTS.maxAccel);
                profileStartTimestamp = clock.seconds();
            }
            MotionState motionState = activeProfile.get(profileTime);
            double targetPower = kV * motionState.getV();
            drive.setDrivePower(new Pose2d(targetPower, 0, 0));

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

    // TODO: integrate these methods directly into the next Road Runner release
    private static boolean pidEquals(PIDCoefficients coeff1, PIDCoefficients coeff2) {
        return coeff1.kP == coeff2.kP && coeff1.kI == coeff2.kI && coeff1.kD == coeff2.kD;
    }

    private static void pidCopy(PIDCoefficients source, PIDCoefficients dest) {
        dest.kP = source.kP;
        dest.kI = source.kI;
        dest.kD = source.kD;
    }
}
