package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.annotation.SuppressLint;
import android.util.Log;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.AngleTracker;

public class DriveToPositionCommand extends SounderBotCommandBase {
    private static final String TAG = "DriveToPositionCmd";

    private static class DriftStats {
        double maxDriftX = 0.0;
        double maxDriftY = 0.0;
        double maxDriftRot = 0.0;
        double totalDriftX = 0.0;
        double totalDriftY = 0.0;
        double totalDriftRot = 0.0;
        int driftSamples = 0;
        double lastX = 0.0;
        double lastY = 0.0;
        double lastRot = 0.0;
        ElapsedTime holdTimer = new ElapsedTime();

        void reset() {
            maxDriftX = maxDriftY = maxDriftRot = 0.0;
            totalDriftX = totalDriftY = totalDriftRot = 0.0;
            driftSamples = 0;
            holdTimer.reset();
        }

        void update(double currentX, double currentY, double currentRot) {
            if (driftSamples == 0) {
                lastX = currentX;
                lastY = currentY;
                lastRot = currentRot;
                driftSamples++;
                return;
            }

            double driftX = Math.abs(currentX - lastX);
            double driftY = Math.abs(currentY - lastY);
            double driftRot = Math.abs(AngleUnit.normalizeRadians(currentRot - lastRot));

            maxDriftX = Math.max(maxDriftX, driftX);
            maxDriftY = Math.max(maxDriftY, driftY);
            maxDriftRot = Math.max(maxDriftRot, driftRot);

            totalDriftX += driftX;
            totalDriftY += driftY;
            totalDriftRot += driftRot;

            lastX = currentX;
            lastY = currentY;
            lastRot = currentRot;
            driftSamples++;
        }

        double getAverageDriftX() {
            return driftSamples > 1 ? totalDriftX / (driftSamples - 1) : 0.0;
        }

        double getAverageDriftY() {
            return driftSamples > 1 ? totalDriftY / (driftSamples - 1) : 0.0;
        }

        double getAverageDriftRot() {
            return driftSamples > 1 ? totalDriftRot / (driftSamples - 1) : 0.0;
        }
    }

    private final AutoMecanumDriveTrain drive;
    private final Telemetry telemetry;
    private final GoBildaPinpointDriver odo;
    private ProfiledPIDController xController, yController, rotController;
    private AngleTracker angleTracker;

    // PID constants
    private static final double kP_trans = 0.1;
    private static final double kI_trans = 0.02;
    private static final double kD_trans = 0.05;

    private static final double kP_rot = 0.1;
    private static final double kI_rot = 0.02;
    private static final double kD_rot = 0.05;

    // Minimum power needed to overcome friction
    private static final double MIN_MOVE_POWER = 0.1;
    private static final double MIN_ROTATION_POWER = 0.08;

    // Tolerance for considering the movement complete
    private static final double POSITION_TOLERANCE = 10.0; // mm
    private static final double ROTATION_TOLERANCE = Math.toRadians(2.0); // rad
    private static final double VELOCITY_TOLERANCE = 10.0; // mm/s

    // Hold position mode constants
    private static final double HOLD_POSITION_kP = 0.15; // Stronger P term for holding position
    private static final double DRIFT_THRESHOLD = 20.0; // mm

    private double targetX, targetY, targetRotation;
    private boolean isHoldingPosition = false;

    private DriftStats driftStats = new DriftStats();

    public DriveToPositionCommand(AutoMecanumDriveTrain drive, Telemetry telemetry) {
        this.drive = drive;
        this.telemetry = telemetry;
        this.odo = drive.getOdo();

        // Create constraints for the profiles
        TrapezoidProfile.Constraints transConstraints =
                new TrapezoidProfile.Constraints(1000, 500); // max vel: 1000mm/s, max accel: 500mm/s²
        TrapezoidProfile.Constraints rotConstraints =
                new TrapezoidProfile.Constraints(Math.PI, Math.PI/2); // max vel: π rad/s, max accel: π/2 rad/s²

        // Initialize ProfiledPIDControllers
        xController = new ProfiledPIDController(kP_trans, kI_trans, kD_trans, transConstraints);
        yController = new ProfiledPIDController(kP_trans, kI_trans, kD_trans, transConstraints);
        rotController = new ProfiledPIDController(kP_rot, kI_rot, kD_rot, rotConstraints);

        // Set tolerance
        xController.setTolerance(POSITION_TOLERANCE);
        yController.setTolerance(POSITION_TOLERANCE);
        rotController.setTolerance(ROTATION_TOLERANCE);

        angleTracker = new AngleTracker();

        // Add drive as requirement
        addRequirements(drive);
    }

    public DriveToPositionCommand setTargetPosition(double x, double y, double rotation) {
        targetX = x;
        targetY = y;
        targetRotation = rotation;

        // Update controller goals with new targets
        xController.setGoal(targetX);
        yController.setGoal(targetY);
        rotController.setGoal(targetRotation);

        // Reset controllers
        xController.reset(odo.getPosX());
        yController.reset(odo.getPosY());
        rotController.reset(odo.getHeading());

        angleTracker = new AngleTracker();
        angleTracker.update(odo.getHeading());

        driftStats.reset();

        return this;
    }

    @Override
    public void initialize() {
        isHoldingPosition = false;
        driftStats.reset();
        Log.i(TAG, String.format("Starting movement to X=%.1f, Y=%.1f, Rot=%.1f°",
                targetX, targetY, Math.toDegrees(targetRotation)));
    }

    @Override
    public void doExecute() {
        odo.update();

        double currentX = odo.getPosX();
        double currentY = odo.getPosY();
        double currentHeading = angleTracker.update(odo.getHeading());

        double xError = targetX - currentX;
        double yError = targetY - currentY;
        double rotError = AngleUnit.normalizeRadians(targetRotation - currentHeading);
        double totalError = Math.hypot(xError, yError);

        if (isHoldingPosition) {
            driftStats.update(currentX, currentY, currentHeading);
        }

        // Check if we should enter or exit hold position mode
        if (totalError < POSITION_TOLERANCE && Math.abs(rotError) < ROTATION_TOLERANCE) {
            if (!isHoldingPosition) {
                enterHoldPositionMode();
            }
        } else if (totalError > DRIFT_THRESHOLD) {
            isHoldingPosition = false;
        }

        // Calculate outputs using ProfiledPIDController
        double xOutput, yOutput, rotOutput;

        if (isHoldingPosition) {
            xOutput = HOLD_POSITION_kP * xError;
            yOutput = HOLD_POSITION_kP * yError;
            rotOutput = HOLD_POSITION_kP * rotError;
        } else {
            xOutput = xController.calculate(currentX, targetX);
            yOutput = yController.calculate(currentY, targetY);
            rotOutput = rotController.calculate(currentHeading, targetRotation);
        }

        // Apply minimum power to overcome friction when needed
        if (!isHoldingPosition) {
            if (Math.abs(xOutput) < MIN_MOVE_POWER && Math.abs(xOutput) > 0.01) {
                xOutput = Math.signum(xOutput) * MIN_MOVE_POWER;
            }
            if (Math.abs(yOutput) < MIN_MOVE_POWER && Math.abs(yOutput) > 0.01) {
                yOutput = Math.signum(yOutput) * MIN_MOVE_POWER;
            }
            if (Math.abs(rotOutput) < MIN_ROTATION_POWER && Math.abs(rotOutput) > 0.01) {
                rotOutput = Math.signum(rotOutput) * MIN_ROTATION_POWER;
            }
        }

        // Apply deadband and clamp
        xOutput = clamp(applyDeadband(xOutput, 0.05), -1.0, 1.0);
        yOutput = clamp(applyDeadband(yOutput, 0.05), -1.0, 1.0);
        rotOutput = clamp(applyDeadband(rotOutput, 0.05), -1.0, 1.0);

        drive.driveRobotCentric(yOutput, xOutput, rotOutput);

        if (isTargetReached()) {
            finished = true;
        }
        updateTelemetry(currentX, currentY, currentHeading, xError, yError, rotError, xOutput, yOutput, rotOutput);
    }

    @Override
    protected boolean isTargetReached() {
        double currentX = odo.getPosX();
        double currentY = odo.getPosY();
        double currentHeading = odo.getHeading();

        double xError = Math.abs(targetX - currentX);
        double yError = Math.abs(targetY - currentY);
        double rotError = Math.abs(AngleUnit.normalizeRadians(targetRotation - currentHeading));

        // Also check velocities are near zero
        double xVel = Math.abs(odo.getVelX());
        double yVel = Math.abs(odo.getVelY());

        boolean atTarget = xError < POSITION_TOLERANCE &&
                yError < POSITION_TOLERANCE &&
                rotError < ROTATION_TOLERANCE &&
                xVel < VELOCITY_TOLERANCE &&
                yVel < VELOCITY_TOLERANCE;

        if (atTarget) {
            Log.i(TAG, String.format("Target reached - Final Position: X=%.1f, Y=%.1f, Rot=%.1f°",
                    currentX, currentY, Math.toDegrees(currentHeading)));
        }

        return atTarget;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drive.driveRobotCentric(0, 0, 0);

        if (interrupted) {
            Log.i(TAG, "Command interrupted");
        } else {
            Log.i(TAG, "Command completed normally");
        }
    }

    private void enterHoldPositionMode() {
        isHoldingPosition = true;
        // Reset integral terms when entering hold mode
        xController.reset();
        yController.reset();
        rotController.reset();
    }

    @SuppressLint("DefaultLocale")
    private void updateTelemetry(double currentX, double currentY, double currentHeading,
                                 double xError, double yError, double rotError,
                                 double xOutput, double yOutput, double rotOutput) {
        // Build base movement info (existing code remains the same)
        String posInfo = String.format("Mode: %s\n" +
                        "Target: X=%.1f, Y=%.1f, Rot=%.1f°\n" +
                        "Current: X=%.1f, Y=%.1f, Rot=%.1f°\n" +
                        "Error: X=%.1f, Y=%.1f, Rot=%.1f°\n" +
                        "Output: X=%.2f, Y=%.2f, Rot=%.2f",
                isHoldingPosition ? "Hold Position" : "Moving",
                targetX, targetY, Math.toDegrees(targetRotation),
                currentX, currentY, Math.toDegrees(currentHeading),
                xError, yError, Math.toDegrees(rotError),
                xOutput, yOutput, rotOutput);

        // Log movement info
        Log.i(TAG, posInfo);

        // Update base telemetry (existing code remains the same)
        telemetry.addData("Mode", isHoldingPosition ? "Hold Position" : "Moving");
        telemetry.addData("Target Position", String.format("X: %.1f, Y: %.1f, Rot: %.1f°",
                targetX, targetY, Math.toDegrees(targetRotation)));
        telemetry.addData("Current Position", String.format("X: %.1f, Y: %.1f, Rot: %.1f°",
                currentX, currentY, Math.toDegrees(currentHeading)));
        telemetry.addData("Error", String.format("X: %.1f, Y: %.1f, Rot: %.1f°",
                xError, yError, Math.toDegrees(rotError)));
        telemetry.addData("Output Powers", String.format("X: %.2f, Y: %.2f, Rot: %.2f",
                xOutput, yOutput, rotOutput));

        // Add drift statistics if in hold position mode
        if (isHoldingPosition) {
            String driftInfo = String.format("Drift Stats:\n" +
                            "Hold Time: %.1f sec\n" +
                            "Max Drift: X=%.1fmm, Y=%.1fmm, Rot=%.1f°\n" +
                            "Avg Drift: X=%.1fmm, Y=%.1fmm, Rot=%.1f°",
                    driftStats.holdTimer.seconds(),
                    driftStats.maxDriftX, driftStats.maxDriftY, Math.toDegrees(driftStats.maxDriftRot),
                    driftStats.getAverageDriftX(), driftStats.getAverageDriftY(),
                    Math.toDegrees(driftStats.getAverageDriftRot()));

            // Log drift stats
            Log.i(TAG, driftInfo);

            // Add drift stats to telemetry
            telemetry.addLine("\nDrift Statistics:");
            telemetry.addData("Hold Time", String.format("%.1f sec", driftStats.holdTimer.seconds()));
            telemetry.addData("Max Drift", String.format("X: %.1fmm, Y: %.1fmm, Rot: %.1f°",
                    driftStats.maxDriftX, driftStats.maxDriftY,
                    Math.toDegrees(driftStats.maxDriftRot)));
            telemetry.addData("Avg Drift", String.format("X: %.1fmm, Y: %.1fmm, Rot: %.1f°",
                    driftStats.getAverageDriftX(), driftStats.getAverageDriftY(),
                    Math.toDegrees(driftStats.getAverageDriftRot())));
        }

        telemetry.update();
    }

    private double clamp(double value, double min, double max) {
        return Math.min(max, Math.max(min, value));
    }

    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0;
        }
        return value;
    }
}
