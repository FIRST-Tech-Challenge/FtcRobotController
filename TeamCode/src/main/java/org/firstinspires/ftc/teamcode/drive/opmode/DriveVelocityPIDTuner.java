package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.drive.roadrunner.RunCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drive.roadrunner.MecanumDriveSubsystem;

import java.util.List;

/**
 * This routine is designed to tune the PID coefficients used by the REV Expansion Hubs for closed-
 * loop velocity control. Although it may seem unnecessary, tuning these coefficients is just as
 * important as the positional parameters. Like the other manual tuning routines, this op mode
 * relies heavily upon the dashboard. To access the dashboard, connect your computer to the RC's
 * WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're using the RC
 * phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once you've successfully
 * connected, start the program, and your robot will begin moving forward and backward according to
 * a motion profile. Your job is to graph the velocity errors over time and adjust the PID
 * coefficients (note: the tuning variable will not appear until the op mode finishes initializing).
 * Once you've found a satisfactory set of gains, add them to the DriveConstants.java file under the
 * MOTOR_VELO_PID field.
 *
 * Recommended tuning process:
 *
 * 1. Increase kP until any phase lag is eliminated. Concurrently increase kD as necessary to
 *    mitigate oscillations.
 * 2. Add kI (or adjust kF) until the steady state/constant velocity plateaus are reached.
 * 3. Back off kP and kD a little until the response is less oscillatory (but without lag).
 *
 * Pressing X (on the Xbox and Logitech F310 gamepads, square on the PS4 Dualshock gamepad) will
 * pause the tuning process and enter driver override, allowing the user to reset the position of
 * the bot in the event that it drifts off the path.
 * Pressing A (on the Xbox and Logitech F310 gamepads, X on the PS4 Dualshock gamepad) will cede
 * control back to the tuning process.
 *
 * NOTE: this has been refactored to use FTCLib's command-based
 */
@Disabled
@Config
@Autonomous(group = "drive")
public class DriveVelocityPIDTuner extends CommandOpMode {

    public static double DISTANCE = 72; // in

    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    private static MotionProfile generateProfile(boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, MAX_VEL, MAX_ACCEL);
    }

    private MecanumDriveSubsystem drive;
    private GamepadEx gamepad;
    private Button xButton, aButton;
    private Mode mode;
    private boolean movingForwards;
    private NanoClock clock;
    private MotionProfile activeProfile;
    private double profileStart, lastKp, lastKi, lastKd, lastKf;

    @Override
    public void initialize() {
        if (!RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg("%s does not need to be run if the built-in motor velocity" +
                    "PID is not in use", getClass().getSimpleName());
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepad = new GamepadEx(gamepad1);

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        mode = Mode.TUNING_MODE;

        lastKp = MOTOR_VELO_PID.p;
        lastKi = MOTOR_VELO_PID.i;
        lastKd = MOTOR_VELO_PID.d;
        lastKf = MOTOR_VELO_PID.f;

        drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

        clock = NanoClock.system();

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        schedule(new SequentialCommandGroup(
            new WaitUntilCommand(this::isStarted),
            new InstantCommand(() -> {
                movingForwards = true;
                activeProfile = generateProfile(true);
                profileStart = clock.seconds();
            })
        ), new RunCommand(() -> telemetry.addData("mode", mode)));

        xButton = new GamepadButton(gamepad, GamepadKeys.Button.X)
                .whenPressed(() -> {
                    mode = Mode.DRIVER_MODE;
                    drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                });
        aButton = new GamepadButton(gamepad, GamepadKeys.Button.A)
                .whenPressed(() -> {
                    drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    mode = Mode.TUNING_MODE;
                    movingForwards = true;
                    activeProfile = generateProfile(movingForwards);
                    profileStart = clock.seconds();
                });

        schedule(new RunCommand(() -> {
            switch (mode) {
                case TUNING_MODE:
                    double profileTime = clock.seconds() - profileStart;

                    if (profileTime > activeProfile.duration()) {
                        // generate a new profile
                        movingForwards = !movingForwards;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

                    MotionState motionState = activeProfile.get(profileTime);
                    double targetPower = kV * motionState.getV();
                    drive.setDrivePower(new Pose2d(targetPower, 0, 0));

                    List<Double> velocities = drive.getWheelVelocities();

                    // update telemetry
                    telemetry.addData("targetVelocity", motionState.getV());
                    for (int i = 0; i < velocities.size(); i++) {
                        telemetry.addData("measuredVelocity" + i, velocities.get(i));
                        telemetry.addData(
                                "error" + i,
                                motionState.getV() - velocities.get(i)
                        );
                    }
                    break;
                case DRIVER_MODE:
                    drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                    break;
            }

            if (lastKp != MOTOR_VELO_PID.p || lastKd != MOTOR_VELO_PID.d
                    || lastKi != MOTOR_VELO_PID.i || lastKf != MOTOR_VELO_PID.f) {
                drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

                lastKp = MOTOR_VELO_PID.p;
                lastKi = MOTOR_VELO_PID.i;
                lastKd = MOTOR_VELO_PID.d;
                lastKf = MOTOR_VELO_PID.f;
            }

            telemetry.update();
        }));
    }

}
