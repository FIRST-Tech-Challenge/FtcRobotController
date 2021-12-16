package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_RPM;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.rpmToVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drive.roadrunner.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.LoggingUtil;
import org.firstinspires.ftc.teamcode.util.RegressionUtil;

import java.util.ArrayList;
import java.util.List;

/**
 * Op mode for computing kV, kStatic, and kA from various drive routines. For the curious, here's an
 * outline of the procedure:
 *   1. Slowly ramp the motor power and record encoder values along the way.
 *   2. Run a linear regression on the encoder velocity vs. motor power plot to obtain a slope (kV)
 *      and an optional intercept (kStatic).
 *   3. Accelerate the robot (apply constant power) and record the encoder counts.
 *   4. Adjust the encoder data based on the velocity tuning data and find kA with another linear
 *      regression.
 *
 * NOTE: this has been refactored to use FTCLib's command-based
 */
@Config
@Autonomous(group = "drive")
public class AutomaticFeedforwardTuner extends CommandOpMode {

    public static final double MAX_POWER = 0.7;
    public static final double DISTANCE = 100; // in

    private MecanumDriveSubsystem drive;
    private GamepadEx gamepad;
    private Button aButton, bButton;
    private NanoClock clock;

    private boolean fitIntercept = false;

    private double maxVel, startTime, finalVel, rampTime, accel, elapsedTime, maxPowerTime;
    private List<Double> timeSamples, positionSamples, powerSamples;
    private RegressionUtil.RampResult rampResult;

    private State state;

    enum State {
        STATIC, ACCEL, ACCEL_STARTED
    }

    @Override
    public void initialize() {
        if (RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg("Feedforward constants usually don't need to be tuned " +
                    "when using the built-in drive motor velocity PID.");
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        gamepad = new GamepadEx(gamepad1);
        state = State.STATIC;

        telemetry.addLine("Press play to begin the feedforward tuning routine");
        telemetry.update();

        schedule(new InstantCommand(() -> {
            telemetry.clearAll();
            telemetry.addLine("Would you like to fit kStatic?");
            telemetry.addLine("Press (A) for yes, (B) for no");
            telemetry.update();
        }));

        InstantCommand staticFinishedCommand = new InstantCommand(() -> {
            drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

            rampResult = RegressionUtil.fitRampData(
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
            telemetry.addLine("Press (A) for yes, (B) for no");
            telemetry.update();

            state = State.ACCEL;
        }, drive);

        FunctionalCommand staticCommand = new FunctionalCommand(
                () -> {
                    telemetry.clearAll();
                    telemetry.addLine("Running...");
                    telemetry.update();

                    maxVel = rpmToVelocity(MAX_RPM);
                    finalVel = MAX_POWER * maxVel;
                    accel = (finalVel * finalVel) / (2.0 * DISTANCE);
                    rampTime = Math.sqrt(2.0 * DISTANCE / accel);

                    timeSamples = new ArrayList<>();
                    positionSamples = new ArrayList<>();
                    powerSamples = new ArrayList<>();

                    drive.setPoseEstimate(new Pose2d());

                    startTime = clock.seconds();
                },
                () -> {
                    elapsedTime = clock.seconds() - startTime;

                    double vel = accel * elapsedTime;
                    double power = vel / maxVel;

                    timeSamples.add(elapsedTime);
                    positionSamples.add(drive.getPoseEstimate().getX());
                    powerSamples.add(MAX_POWER);

                    drive.setDrivePower(new Pose2d(power, 0.0, 0.0));
                },
                staticFinishedCommand::schedule,
                () -> elapsedTime > rampTime,
                drive
        );

        InstantCommand accelFinishedCommand = new InstantCommand(() -> {
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

            reset(); // end the scheduler instance
        }, drive);

        FunctionalCommand accelCommand = new FunctionalCommand(
                () -> {
                    telemetry.clearAll();
                    telemetry.addLine("Running...");
                    telemetry.update();

                    maxPowerTime = DISTANCE / maxVel;

                    timeSamples.clear();
                    positionSamples.clear();
                    powerSamples.clear();

                    drive.setPoseEstimate(new Pose2d());
                    drive.setDrivePower(new Pose2d(MAX_POWER, 0.0, 0.0));

                    startTime = clock.seconds();
                },
                () -> {
                    elapsedTime = clock.seconds() - startTime;

                    timeSamples.add(elapsedTime);
                    positionSamples.add(drive.getPoseEstimate().getX());
                    powerSamples.add(MAX_POWER);
                },
                accelFinishedCommand::schedule,
                () -> elapsedTime > maxPowerTime,
                drive
        );

        aButton = new GamepadButton(gamepad, GamepadKeys.Button.A)
            .whenPressed(() -> {
                switch (state) {
                    case STATIC:
                        telemetry.clearAll();
                        telemetry.addLine(Misc.formatInvariant(
                                "Place your robot on the field with at least %.2f in of room in front", DISTANCE));
                        telemetry.addLine("Press (A) to begin");
                        telemetry.update();

                        fitIntercept = true;
                        staticCommand.schedule();
                        break;
                    case ACCEL:
                        telemetry.clearAll();
                        telemetry.addLine("Place the robot back in its starting position");
                        telemetry.addLine("Press (A) to continue");
                        telemetry.update();

                        state = State.ACCEL_STARTED;
                        break;
                    case ACCEL_STARTED:
                        accelCommand.schedule();
                        break;
                }
            });

        bButton = new GamepadButton(gamepad, GamepadKeys.Button.B)
            .whenPressed(() -> {
                switch (state) {
                    case STATIC:
                        telemetry.clearAll();
                        telemetry.addLine(Misc.formatInvariant(
                                "Place your robot on the field with at least %.2f in of room in front", DISTANCE));
                        telemetry.addLine("Press (A) to begin");
                        telemetry.update();

                        staticCommand.schedule();
                        break;
                    case ACCEL:
                        reset(); // end the scheduled commands
                        break;
                }
            });
    }

}