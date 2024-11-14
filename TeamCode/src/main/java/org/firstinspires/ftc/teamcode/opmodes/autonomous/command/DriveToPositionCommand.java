package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.AngleTracker;

import android.annotation.SuppressLint;
import android.util.Log;

public class DriveToPositionCommand extends CommandBase {
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
    private PIDController xController, yController, rotController;
    private AngleTracker angleTracker;

    // PID constants
    private static final double kP_trans = 0.1;
    private static final double kI_trans = 0.02;
    private static final double kD_trans = 0.05;

    private static final double kP_rot = 0.1;
    private static final double kI_rot = 0.02;
    private static final double kD_rot = 0.05;

    // Motion profile constants
    private static final double MAX_VELOCITY = 1000; // mm/s
    private static final double MAX_ACCELERATION = 500; // mm/s²
    private static final double MAX_ROT_VELOCITY = Math.PI; // rad/s
    private static final double MAX_ROT_ACCELERATION = Math.PI/2; // rad/s²

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
    private double motionProfileScale = 0.0;

    private DriftStats driftStats = new DriftStats();

    public DriveToPositionCommand(AutoMecanumDriveTrain drive, Telemetry telemetry) {
        this.drive = drive;
        this.telemetry = telemetry;
        this.odo = drive.getOdo();

        // Initialize controllers
        xController = new PIDController(kP_trans, kI_trans, kD_trans);
        yController = new PIDController(kP_trans, kI_trans, kD_trans);
        rotController = new PIDController(kP_rot, kI_rot, kD_rot);

        // Set integral limits to prevent windup
        xController.setIntegrationBounds(-0.3, 0.3);
        yController.setIntegrationBounds(-0.3, 0.3);
        rotController.setIntegrationBounds(-0.3, 0.3);

        angleTracker = new AngleTracker();

        // Add drive as requirement
        addRequirements(drive);
    }

    public DriveToPositionCommand setTargetPosition(double x, double y, double rotation) {
        targetX = x;
        targetY = y;
        targetRotation = rotation;

        xController.setSetPoint(targetX);
        yController.setSetPoint(targetY);
        rotController.setSetPoint(targetRotation);

        xController.reset();
        yController.reset();
        rotController.reset();

        angleTracker = new AngleTracker();
        angleTracker.update(odo.getHeading());

        motionProfileScale = 0.0;
        driftStats.reset();
        return this;
    }

    @Override
    public void initialize() {
        isHoldingPosition = false;
        motionProfileScale = 0.0;
        driftStats.reset();
        Log.i(TAG, String.format("Starting movement to X=%.1f, Y=%.1f, Rot=%.1f°",
                targetX, targetY, Math.toDegrees(targetRotation)));
    }

    @Override
    public void execute() {
        odo.update();

        // Get current position and heading
        double currentX = odo.getPosX();
        double currentY = odo.getPosY();
        double currentHeading = angleTracker.update(odo.getHeading());

        // Calculate errors
        double xError = targetX - currentX;
        double yError = targetY - currentY;
        double rotError = AngleUnit.normalizeRadians(targetRotation - currentHeading);
        double totalError = Math.hypot(xError, yError);

        // Check if we should enter or exit hold position mode
        if (totalError < POSITION_TOLERANCE && Math.abs(rotError) < ROTATION_TOLERANCE) {
            if (!isHoldingPosition) {
                enterHoldPositionMode();
            }
        } else if (totalError > DRIFT_THRESHOLD) {
            isHoldingPosition = false;
        }

        // Calculate base PID outputs
        double xOutput, yOutput, rotOutput;

        if (isHoldingPosition) {
            // Use stronger P term when holding position
            xOutput = HOLD_POSITION_kP * xError;
            yOutput = HOLD_POSITION_kP * yError;
            rotOutput = HOLD_POSITION_kP * rotError;
            driftStats.update(currentX, currentY, currentHeading);
        } else {
            xOutput = xController.calculate(currentX);
            yOutput = yController.calculate(currentY);
            rotOutput = rotController.calculate(currentHeading);

            // Apply motion profile scaling during movement
            updateMotionProfileScale(totalError, Math.abs(rotError));
            xOutput *= motionProfileScale;
            yOutput *= motionProfileScale;
            rotOutput *= motionProfileScale;
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

        // Apply deadband and clamp outputs
        xOutput = clamp(applyDeadband(xOutput, 0.05), -1.0, 1.0);
        yOutput = clamp(applyDeadband(yOutput, 0.05), -1.0, 1.0);
        rotOutput = clamp(applyDeadband(rotOutput, 0.05), -1.0, 1.0);

        // Drive the robot
        drive.driveRobotCentric(yOutput, xOutput, rotOutput);

        // Update telemetry
        updateTelemetry(currentX, currentY, currentHeading, xError, yError, rotError, xOutput, yOutput, rotOutput);
    }

    @Override
    public boolean isFinished() {
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

    private void updateMotionProfileScale(double distance, double rotDistance) {
        double rampUpDistance = Math.pow(MAX_VELOCITY, 2) / (2 * MAX_ACCELERATION);
        double rampDownDistance = rampUpDistance;

        if (distance < (rampUpDistance + rampDownDistance) && distance > 0) {
            double scaleFactor = distance / (rampUpDistance + rampDownDistance);
            rampUpDistance *= scaleFactor;
            rampDownDistance *= scaleFactor;
        }

        double currentX = odo.getPosX();
        double currentY = odo.getPosY();
        double distanceTraveled = Math.hypot(currentX, currentY);

        double translateScale;
        if (distanceTraveled < rampUpDistance) {
            translateScale = Math.min(1.0, distanceTraveled / rampUpDistance);
        } else if (distance < rampDownDistance) {
            translateScale = Math.min(1.0, distance / rampDownDistance);
        } else {
            translateScale = 1.0;
        }

        double rotScale;
        if (rotDistance > 0.1) {
            if (rotDistance > Math.PI/2) {
                rotScale = 1.0;
            } else {
                rotScale = rotDistance / (Math.PI/2);
            }
        } else {
            rotScale = 0.0;
        }

        motionProfileScale = Math.min(translateScale, rotScale);
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
