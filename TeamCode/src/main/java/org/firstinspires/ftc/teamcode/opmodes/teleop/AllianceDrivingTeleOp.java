package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.Controls;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

import java.util.function.DoubleSupplier;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.OpModeHook;

public abstract class AllianceDrivingTeleOp extends NextOpMode {
    private final String allianceName;
    private final KronBot robot;
    private final Controls controls;
    private Pose selectedStartPose;

    private static boolean canSave = false;

    private final int basket1Pipeline;
    private final int basket2Pipeline;

    protected AllianceDrivingTeleOp(
            KronBot robot,
            String allianceName,
            DoubleSupplier basket1X,
            DoubleSupplier basket1Y,
            DoubleSupplier basket2X,
            DoubleSupplier basket2Y,
            int basket1Pipeline,
            int basket2Pipeline) {
        super(robot, stopRobotOnExit(robot));
        this.robot = robot;
        this.allianceName = allianceName;
        this.basket1Pipeline = basket1Pipeline;
        this.basket2Pipeline = basket2Pipeline;
        controls = new Controls(gamepad1);
        selectedStartPose = leftStartPose();
        canSave = false;

        try {
            robot.initialize(hardwareMap, selectedStartPose);
            robot.drive.configureAimTargets(basket1X, basket1Y, basket2X, basket2Y);
            robot.vision.initialize(hardwareMap, basket1Pipeline);
            addInitTelemetry();
        } catch (RuntimeException | Error initializationFailure) {
            try {
                robot.release();
            } catch (RuntimeException | Error cleanupFailure) {
                initializationFailure.addSuppressed(cleanupFailure);
            }
            throw initializationFailure;
        }
    }

    @Override
    public void disabledPeriodic() {
        controls.update();
        if (controls.dpadLeft.justPressed()) {
            selectedStartPose = leftStartPose();
            robot.setPose(selectedStartPose);
        } else if (controls.dpadRight.justPressed()) {
            selectedStartPose = rightStartPose();
            robot.setPose(selectedStartPose);
        } else if (controls.dpadDown.justPressed()) {
            selectedStartPose = PoseStorage.loadPose();
            robot.setPose(selectedStartPose);
        }

        robot.update();
        addInitTelemetry();
    }

    @Override
    public void start() {
        robot.vision.start();
        canSave = true;
    }

    @Override
    public void periodic() {
        controls.update();
        handleDrivingButtons();
        updateAutoAim();
        handlePoseReset();
        handleIntakeButtons();

        robot.drive.runManual(controls);
        updateAimPipeline();
        robot.update();
        addDriverTelemetry();
        telemetry.update();
    }

    private void handleDrivingButtons() {
        if (controls.triangle.justPressed() && robot.drive.isManual()) {
            robot.drive.toggleFieldCentric();
        }
    }

    private void updateAutoAim() {
        if (controls.leftTrigger.getValue() > 0.5 && robot.drive.isManual()) {
            robot.drive.startAiming();
        } else {
            robot.drive.stopAiming();
        }
    }

    private void handlePoseReset() {
        if (controls.leftStickButton.justPressed()) {
            robot.setPose(robot.follower.getPose().getX() < 72.0
                    ? leftStartPose() : rightStartPose());
        }
    }

    private void updateAimPipeline() {
        if (!robot.drive.isAiming()) {
            return;
        }

        int pipeline = robot.drive.isUsingFirstAimTarget()
                ? basket1Pipeline
                : basket2Pipeline;
        robot.vision.selectPipeline(pipeline);
    }

    private void handleIntakeButtons() {
        if (robot.intake == null) {
            return;
        }
        if (controls.rightBumper.justPressed()) {
            robot.intake.toggle(false);
        }
        if (controls.leftBumper.justPressed()) {
            robot.intake.toggle(true);
        }
    }

    private static OpModeHook stopRobotOnExit(KronBot robot) {
        return new OpModeHook() {
            @Override
            public void afterEnd() {
                try {
                    if (canSave && robot.follower != null) {
                        PoseStorage.savePose(robot.follower.getPose());
                    }
                } finally {
                    robot.release();
                }
            }
        };
    }

    private void addInitTelemetry() {
        telemetry.addData("Alliance", allianceName);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start pose", "(%.1f, %.1f, %.0f deg)",
                selectedStartPose.getX(), selectedStartPose.getY(),
                Math.toDegrees(selectedStartPose.getHeading()));
        telemetry.addLine("INIT: D-pad left/right selects the physical starting side, down selects saved pose");
        robot.drive.telemetry(telemetry);
        if (!robot.drive.isSelectedAimTargetConfigured()) {
            telemetry.addLine("WARNING: enter this alliance's basket coordinates in Constants");
        }
        telemetry.update();
    }

    private void addDriverTelemetry() {
        telemetry.addData("Alliance", allianceName);
        robot.drive.telemetry(telemetry);
        if (robot.intake != null) {
            robot.intake.telemetry(telemetry);
        }
        robot.vision.telemetry(telemetry);
        telemetry.addLine("Triangle: field centric");
        telemetry.addLine("L2 > 50%: autoaim | Basket: automatic by Y | L3: reset pose");
        telemetry.addLine("Limelight pipeline follows the aim basket");
    }

    private static Pose leftStartPose() {
        return new Pose(Constants.LEFT_START_X, Constants.START_Y,
                Math.toRadians(Constants.START_HEADING_DEG));
    }

    private static Pose rightStartPose() {
        return new Pose(Constants.RIGHT_START_X, Constants.START_Y,
                Math.toRadians(Constants.START_HEADING_DEG));
    }

    private static Pose centerPose() {
        return new Pose(Constants.CENTER_X, Constants.CENTER_Y,
                Math.toRadians(Constants.CENTER_HEADING_DEG));
    }
}
