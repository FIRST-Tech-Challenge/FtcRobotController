package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.utils.Constants.AIM_DEADBAND_RAD;
import static org.firstinspires.ftc.teamcode.utils.Constants.AIM_INTEGRAL_LIMIT;
import static org.firstinspires.ftc.teamcode.utils.Constants.AIM_KD;
import static org.firstinspires.ftc.teamcode.utils.Constants.AIM_KI;
import static org.firstinspires.ftc.teamcode.utils.Constants.AIM_KP;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.Controls;

import java.util.function.DoubleSupplier;

import dev.nextftc.robot.Mechanism;

public final class DriveController implements Mechanism {
    private final KronBot robot;
    private Pose targetPose;
    private boolean fieldCentric;
    private boolean aimingAtTarget;
    private int selectedAimTarget;
    private double targetX;
    private double targetY;
    private DoubleSupplier target1X = () -> Double.NaN;
    private DoubleSupplier target1Y = () -> Double.NaN;
    private DoubleSupplier target2X = () -> Double.NaN;
    private DoubleSupplier target2Y = () -> Double.NaN;
    private double integral;
    private double lastError;
    private long lastTime = System.nanoTime();
    private boolean hasLastAimError;
    private boolean lastAimUsedLimelight;
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public DriveController(KronBot robot) {
        this.robot = robot;
    }

    public void initialize() {
        targetPose = null;
        fieldCentric = false;
        aimingAtTarget = false;
        selectedAimTarget = 0;
        robot.follower.setMaxPowerScaling(1.0);
        robot.follower.startTeleopDrive(true);
        resetAimPid();
    }

    public void configureAimTargets(
            DoubleSupplier target1X,
            DoubleSupplier target1Y,
            DoubleSupplier target2X,
            DoubleSupplier target2Y) {
        this.target1X = target1X;
        this.target1Y = target1Y;
        this.target2X = target2X;
        this.target2Y = target2Y;
        selectedAimTarget = 0;
        loadSelectedAimTarget();
        resetAimPid();
    }

//    public void startManual() {
//        targetPose = null;
//
//    }

    public void runManual(Controls controls) {
        if (!isManual()) {
            return;
        }

        double forward = Constants.applyDeadzone(-controls.leftStickY);
        double strafe = Constants.applyDeadzone(-controls.leftStickX);
        double turn = aimingAtTarget
                ? getAimTurn()
                : Constants.applyDeadzone(-controls.rightStickX);

        robot.follower.setTeleOpDrive(forward, strafe, turn, !fieldCentric);
    }

//    public void driveTo(Pose destination) {
//        Pose currentPose = robot.follower.getPose();
//        targetPose = new Pose(destination.getX(), destination.getY(), destination.getHeading());
//
//        PathChain path = robot.follower.pathBuilder()
//                .addPath(new BezierLine(currentPose, targetPose))
//                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
//                .build();
//
//        robot.follower.followPath(path, Constants.PATH_MAX_POWER, true);
//    }

    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public boolean isManual() {
        return robot.follower != null && robot.follower.isTeleopDrive();
    }

    public boolean startAiming() {
        if (!loadSelectedAimTarget()) {
            aimingAtTarget = false;
            resetAimPid();
            return false;
        }
        resetAimPid();
        aimingAtTarget = true;
        return true;
    }

    public void stopAiming() {
        aimingAtTarget = false;
        resetAimPid();
    }

    public boolean isAiming() {
        return aimingAtTarget;
    }

    public void toggleAimingTarget() {
        selectedAimTarget = 1 - selectedAimTarget;
        if (!loadSelectedAimTarget()) {
            aimingAtTarget = false;
        }
        resetAimPid();
    }

    public int getSelectedAimTargetNumber() {
        return selectedAimTarget + 1;
    }

    public boolean isSelectedAimTargetConfigured() {
        return loadSelectedAimTarget();
    }

    public void stop() {
        if (robot.follower != null) {
            robot.follower.breakFollowing();
        }
    }

    public void telemetry(Telemetry telemetry) {
        if (robot.follower == null) {
            return;
        }
        Pose pose = robot.follower.getPose();
        String mode = isManual()
                ? "Manual"
                : (robot.follower.isBusy() ? "Following path" : "Holding pose");

        telemetry.addLine("--- DRIVE ---");
        telemetry.addData("Mode", mode);
        telemetry.addData("X (in)", "%.2f", pose.getX());
        telemetry.addData("Y (in)", "%.2f", pose.getY());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Reference", fieldCentric ? "Field centric" : "Robot centric");
        if (targetPose != null) {
            telemetry.addData("Target", "(%.1f, %.1f, %.0f deg)",
                    targetPose.getX(), targetPose.getY(),
                    Math.toDegrees(targetPose.getHeading()));
        }
        telemetry.addData("Aim basket", getSelectedAimTargetNumber());
        telemetry.addData("Aim coordinates", isSelectedAimTargetConfigured()
                ? String.format("(%.2f, %.2f)", targetX, targetY)
                : "NOT CONFIGURED");
        telemetry.addData("Autoaim", aimingAtTarget ? "ON" : "OFF");
    }

    private double getAimTurn() {
        if (!loadSelectedAimTarget()) {
            aimingAtTarget = false;
            resetAimPid();
            return 0.0;
        }

        Pose pose = robot.follower.getPose();
        double txDeg = LimelightController.getAimTx();
        boolean usingLimelight = Double.isFinite(txDeg);
        if (hasLastAimError && usingLimelight != lastAimUsedLimelight) {
            // Do not carry integral or derivative history between error sources.
            resetAimPid();
        }
        lastAimUsedLimelight = usingLimelight;
        double desiredHeading;

        if (usingLimelight) {
            desiredHeading = pose.getHeading() - Math.toRadians(txDeg);
        } else {
            desiredHeading = Math.atan2(
                    targetY - pose.getY(),
                    targetX - pose.getX()
            );
        }

        double error = wrap(desiredHeading - pose.getHeading());

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        if (Math.abs(error) < AIM_DEADBAND_RAD) {
            integral = 0.0;
            lastError = error;
            hasLastAimError = true;
            return 0.0;
        }

        if (dt <= 0 || dt > 0.1) {
            dt = 0.01;
        }
        integral = clamp(integral + error * dt, AIM_INTEGRAL_LIMIT);

        double derivative = hasLastAimError ? wrap(error - lastError) / dt : 0.0;
        lastError = error;
        hasLastAimError = true;

        double kP = usingLimelight ? Constants.LIMELIGHT_AIM_KP : AIM_KP;
        double kI = usingLimelight ? Constants.LIMELIGHT_AIM_KI : AIM_KI;
        double kD = usingLimelight ? Constants.LIMELIGHT_AIM_KD : AIM_KD;
        double output = kP * error + kI * integral + kD * derivative;
        panelsTelemetry.addData("Aim PID source", usingLimelight ? "Limelight" : "Odometry");
        panelsTelemetry.addData("Aim Error", error);
        panelsTelemetry.addData("Aim Desired", desiredHeading);
        panelsTelemetry.addData("Aim Output", output);
        panelsTelemetry.addData("TX deg", txDeg);
        panelsTelemetry.update();
        return clamp(output, 1.0);
    }

    private void resetAimPid() {
        integral = 0.0;
        lastError = 0.0;
        lastTime = System.nanoTime();
        hasLastAimError = false;
    }

    public boolean isUsingFirstAimTarget() {
        return robot.follower.getPose().getY() < 72.0;
    }

    private boolean loadSelectedAimTarget() {
        boolean useFirstBasket = isUsingFirstAimTarget();
        targetX = (useFirstBasket ? target1X : target2X).getAsDouble();
        targetY = (useFirstBasket ? target1Y : target2Y).getAsDouble();
        return Double.isFinite(targetX) && Double.isFinite(targetY);
    }

    private static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private static double clamp(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }
}
