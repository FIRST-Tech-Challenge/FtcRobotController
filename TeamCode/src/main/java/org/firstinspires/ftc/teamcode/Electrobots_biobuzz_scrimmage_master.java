package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.navigate.SnapMecanumMover;
import org.firstinspires.ftc.teamcode.navigate.MecanumWheelMotorOdometry;

import org.firstinspires.ftc.teamcode.navigate.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.drivesupport.Robot_Placement;
import org.firstinspires.ftc.teamcode.drivesupport.LightIndicator;
import org.firstinspires.ftc.teamcode.scoring.IntakeController;
import org.firstinspires.ftc.teamcode.scoring.ShooterController;
import org.firstinspires.ftc.teamcode.scoring.ScoringConfig;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Teleop-Scrimmage_v1", group = "Scrimmage")

public class Electrobots_biobuzz_scrimmage_master extends OpMode {
    // Four drive motors for a mecanum drivetrain.
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // IMU gives heading/orientation so field-relative driving can work.
    IMU imu;

    // Helper that converts joystick inputs into motor powers.
    private SnapMecanumMover snapMecanumMover;
    // Helper that detects AprilTags and stores values like x offset and confidence/percent.
    private AprilTagWebcam aprilTagWebcam;

    // Helper that decides which target AprilTag ID we care about.
    private Robot_Placement robotPlacement;

    // Driver/starting-side profiles used by Robot_Placement.
    private static final int RED_1 = 1;
    private static final int RED_2 = 2;
    private static final int BLUE_1 = 3;
    private static final int BLUE_2 = 4;

    private static final double SNAP_STRAFE_POWER = 0.2;
    private static final double SNAP_CLUSTER_LOCK_PERCENT = 75.0;
    private static final double SNAP_EAST_SHIFT_ROTATIONS = 1.5;
    private static final double SNAP_WEST_SHIFT_ROTATIONS = 1.5;

    // true: run full scoring-bin recovery flow (turn + lateral move + re-snap).
    // false: legacy mode (no east/west move), with side-based yaw snap assist only.
    private static final boolean SNAP_TO_SCORING_BIN = true;

    private IntakeController intakeController;
    private ShooterController shooterController;
    private boolean intakeRoutineActive;
    private boolean shooterRoutineActive;

    // Light helper for driver-facing state feedback.
    private LightIndicator lightIndicator;

    private int placementProfile = RED_1;
    private SnapRecoveryPhase snapRecoveryPhase = SnapRecoveryPhase.IDLE;
    private boolean snapRecoveryMoveEast;
    private boolean snapRecoveryMoveTargetInitialized;
    private boolean previousSnapRequest;

    private enum SnapRecoveryPhase {
        IDLE,
        TURN_INITIAL,
        MOVE_LATERAL,
        SNAP_ALIGN,
        TURN_FINAL
    }

    /*
     * What this method does:
     * - Uses camera numbers to move left/right until the AprilTag is centered.
     *
     * Values it reads:
     * - aprilTagWebcam.percent_value: how "good"/complete the detection is.
     *   Smaller than 100 here means we still need to adjust position.
     * - aprilTagWebcam.x_value: horizontal error from center.
     *   < 0 means tag appears left, > 0 means tag appears right, 0 means centered.
     *
     * Parameters passed to driveFieldRelative(forward, strafe, turn):
     * - forward = 0.0 (do not move forward/back)
     * - strafe  = +/-0.2 (slide left/right slowly)
     * - turn    = 0.0 (do not rotate)
     *
     * Outcome:
     * - Robot nudges sideways to line the tag up in camera view.
     */
    public void snap_to_tag() {
        AprilTagWebcam.SnapTelemetry snapTelemetry = aprilTagWebcam.getSnapTelemetry();

        if (!snapTelemetry.hasCluster || snapTelemetry.percentClusterFound >= SNAP_CLUSTER_LOCK_PERCENT) {
            // Stop early once cluster confidence is good enough, rather than forcing x close to 0.
            snapMecanumMover.driveFieldRelative(0.0, 0.0, 0.0);
            return;
        }

        if (aprilTagWebcam.x_value < 0) {
            snapMecanumMover.driveFieldRelative(0.0, -SNAP_STRAFE_POWER, 0.0); // Tag appears left: strafe left.
        } else if (aprilTagWebcam.x_value > 0) {
            snapMecanumMover.driveFieldRelative(0.0, SNAP_STRAFE_POWER, 0.0); // Tag appears right: strafe right.
        } else {
            snapMecanumMover.driveFieldRelative(0.0, 0.0, 0.0); // Already centered enough this cycle.
        }
    }

    private boolean isPlacementColorMatch(String clusterName) {
        if (clusterName == null) {
            return false;
        }

        if (placementProfile == RED_1 || placementProfile == RED_2) {
            return clusterName.startsWith("RED");
        }

        if (placementProfile == BLUE_1 || placementProfile == BLUE_2) {
            return clusterName.startsWith("BLUE");
        }

        return false;
    }

    private boolean shouldStartSnapRecovery(boolean snapRequested) {
        if (!SNAP_TO_SCORING_BIN) {
            return false;
        }

        if (!snapRequested || snapRecoveryPhase != SnapRecoveryPhase.IDLE) {
            return false;
        }

        AprilTagWebcam.SnapTelemetry snapTelemetry = aprilTagWebcam.getSnapTelemetry();
        if (!snapTelemetry.hasCluster) {
            return false;
        }

        // "Non scoring / tipped down" means the cluster is upside-down by roll test.
        boolean nonScoringTippedDown = Math.abs(snapTelemetry.rollDegrees) >= 90.0;
        if (!nonScoringTippedDown) {
            return false;
        }

        return isPlacementColorMatch(snapTelemetry.clusterName);
    }

    private void startSnapRecovery() {
        AprilTagWebcam.SnapTelemetry snapTelemetry = aprilTagWebcam.getSnapTelemetry();
        boolean audienceSide = snapTelemetry.clusterName != null && snapTelemetry.clusterName.contains("AUDIENCE");

        // If the non-scoring tag is audience side => left trigger + west shift.
        // Else => right trigger + east shift.
        snapRecoveryMoveEast = !audienceSide;
        snapRecoveryMoveTargetInitialized = false;

        if (snapRecoveryMoveEast) {
            snapMecanumMover.snapTurnRightTriggerShortcut();
        } else {
            snapMecanumMover.snapTurnLeftTriggerShortcut();
        }

        snapRecoveryPhase = SnapRecoveryPhase.TURN_INITIAL;
    }

    private void runSnapRecoveryRoutine() {
        if (snapRecoveryPhase == SnapRecoveryPhase.TURN_INITIAL) {
            if (snapMecanumMover.stepSnapTurn(telemetry)) {
                snapRecoveryPhase = SnapRecoveryPhase.MOVE_LATERAL;
                snapRecoveryMoveTargetInitialized = false;
            }
            return;
        }

        if (snapRecoveryPhase == SnapRecoveryPhase.MOVE_LATERAL) {
            if (!snapRecoveryMoveTargetInitialized) {
                MecanumWheelMotorOdometry.Pose2d pose = snapMecanumMover.getOdometry().getPose();
                double shiftInches = (snapRecoveryMoveEast ? SNAP_EAST_SHIFT_ROTATIONS : SNAP_WEST_SHIFT_ROTATIONS)
                        * MecanumWheelMotorOdometry.WHEEL_CIRCUMFERENCE_INCHES;
                double targetX = pose.xInches + (snapRecoveryMoveEast ? shiftInches : -shiftInches);

                snapMecanumMover.setTargetPose(targetX, pose.yInches, pose.headingRadians);
                snapRecoveryMoveTargetInitialized = true;
            }

            if (snapMecanumMover.navigateToTarget()) {
                snapMecanumMover.cancelTarget();
                snapRecoveryPhase = SnapRecoveryPhase.SNAP_ALIGN;
            }
            return;
        }

        if (snapRecoveryPhase == SnapRecoveryPhase.SNAP_ALIGN) {
            snap_to_tag();

            AprilTagWebcam.SnapTelemetry snapTelemetry = aprilTagWebcam.getSnapTelemetry();
            if (!snapTelemetry.hasCluster || snapTelemetry.percentClusterFound >= SNAP_CLUSTER_LOCK_PERCENT) {
                if (snapRecoveryMoveEast) {
                    snapMecanumMover.snapTurnRightTriggerShortcut();
                } else {
                    snapMecanumMover.snapTurnLeftTriggerShortcut();
                }
                snapRecoveryPhase = SnapRecoveryPhase.TURN_FINAL;
            }
            return;
        }

        if (snapRecoveryPhase == SnapRecoveryPhase.TURN_FINAL) {
            if (snapMecanumMover.stepSnapTurn(telemetry)) {
                snapRecoveryPhase = SnapRecoveryPhase.IDLE;
            }
        }
    }

    private boolean isScoringSideCluster(String clusterName) {
        return clusterName != null && clusterName.contains("SCORING");
    }

    private boolean isAudienceSideCluster(String clusterName) {
        return clusterName != null && clusterName.contains("AUDIENCE");
    }

    private void runLegacySnapMode(boolean snapRequestEdge) {
        AprilTagWebcam.SnapTelemetry snapTelemetry = aprilTagWebcam.getSnapTelemetry();
        if (!snapTelemetry.hasCluster || !isPlacementColorMatch(snapTelemetry.clusterName)) {
            return;
        }

        // Legacy mode yaw assist:
        // - scoring side => left trigger snap target
        // - audience side => right trigger snap target
        if (snapRequestEdge && !snapMecanumMover.isSnapTurnActive()) {
            if (isScoringSideCluster(snapTelemetry.clusterName)) {
                snapMecanumMover.snapTurnLeftTriggerShortcut();
            } else if (isAudienceSideCluster(snapTelemetry.clusterName)) {
                snapMecanumMover.snapTurnRightTriggerShortcut();
            }
        }

        if (snapMecanumMover.isSnapTurnActive()) {
            snapMecanumMover.stepSnapTurn(telemetry);
            return;
        }

        // In legacy mode, strafe alignment is used only on scoring-side target clusters.
        if (isScoringSideCluster(snapTelemetry.clusterName)) {
            snap_to_tag();
        } else {
            snapMecanumMover.driveFieldRelative(0.0, 0.0, 0.0);
        }
    }

    /**
     * Adds dedicated driver telemetry while snap-to-tag is active.
     *
     * Data is fetched from AprilTagWebcam as a ready-to-render snapshot so
     * detection parsing and type decisions stay inside the camera helper.
     */
    private void addSnapToTagTelemetry() {
        AprilTagWebcam.SnapTelemetry snapTelemetry = aprilTagWebcam.getSnapTelemetry();
        telemetry.addLine("=== SNAP TO TAG ===");
        telemetry.addData("Visible tags", snapTelemetry.visibleTagCount);

        if (!snapTelemetry.hasCluster) {
            telemetry.addData("Selected cluster", "NONE");
            return;
        }

        telemetry.addData("Selected cluster", snapTelemetry.clusterName);
        telemetry.addData("Cluster percent", "%.0f", snapTelemetry.percentClusterFound);
        telemetry.addData("Tag XYZ (in)", "%.1f, %.1f, %.1f",
                snapTelemetry.xInches,
                snapTelemetry.yInches,
                snapTelemetry.zInches);
        telemetry.addData("Tag PRY (deg)", "%.1f, %.1f, %.1f",
                snapTelemetry.pitchDegrees,
                snapTelemetry.rollDegrees,
                snapTelemetry.yawDegrees);
        telemetry.addData("Tag RBE", "%.1f in, %.1f deg, %.1f deg",
                snapTelemetry.rangeInches,
                snapTelemetry.bearingDegrees,
                snapTelemetry.elevationDegrees);
    }

    /*
     * FTC calls init() once when you press INIT on Driver Station.
     *
     * Job of this method:
     * 1) Connect Java variables to real hardware using names from Robot Configuration.
     * 2) Set motor directions/modes.
     * 3) Set IMU orientation so heading math is correct.
     * 4) Build helper classes and tell camera which AprilTag to track.
     */
    @Override
    public void init() {
        // hardwareMap.get(Type, "name") links code to configured hardware devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Match base starterbot mecanum direction profile.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // RUN_USING_ENCODER lets the motor controller use encoder feedback for steadier speed control.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        // These two lines describe how the Control/Expansion Hub is mounted on the robot.
        // If these are wrong, field-relative driving will feel "rotated" or incorrect.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot)); // Apply mounting info to IMU.

        // Build driving helper with all 4 motors + imu.
        snapMecanumMover = new SnapMecanumMover(
                frontLeftDrive,
                frontRightDrive,
                backLeftDrive,
                backRightDrive,
                imu
        );

        // Choose team/start profile, then it sets the target AprilTag.
        // Keep profile in a variable so we can reuse it for indicator color selection.
        placementProfile = RED_1; // Change this if your side/starting position changes.

        robotPlacement = new Robot_Placement();
        robotPlacement.init(placementProfile);

        // Start camera/AprilTag system and pass in which tag we want to track.
        aprilTagWebcam = new AprilTagWebcam();
        aprilTagWebcam.init(hardwareMap, telemetry, robotPlacement.target_tag_1, robotPlacement.target_tag_2);

        intakeController = new IntakeController();
        intakeController.init(hardwareMap);

        shooterController = new ShooterController();
        shooterController.init(hardwareMap);

        // Needed for short distance shifts during assisted snap recovery.
        snapMecanumMover.initializeOdometry();

        // Light setup:
        // - hardware name expected in Robot Config: "ftc_light"
        // - placement profile selects default team color
        lightIndicator = new LightIndicator();
        lightIndicator.init(hardwareMap);

        LightIndicator.LightColor placementColor =
                (placementProfile == RED_1 || placementProfile == RED_2)
                        ? LightIndicator.RED
                        : LightIndicator.BLUE;
        lightIndicator.state(true, placementColor); // Turn on using placement profile color.

        intakeRoutineActive = false;
        shooterRoutineActive = false;
    }

    /*
     * FTC calls loop() over and over while the OpMode is running.
     *
     * Flow each cycle:
     * 1) Update camera detections.
     * 2) Decide if driver requested "snap" mode (gamepad2 right bumper) and tag is visible.
     * 3) Run normal drive logic.
     * 4) If snap mode is active, add sideways auto-correction.
     */
    @Override
    public void loop() {
        aprilTagWebcam.updateDetections();
        // Must be true for BOTH conditions:
        // - driver is holding right bumper
        // - camera currently sees any tag
        boolean driverSnappingRequested = gamepad2.right_bumper;
        boolean driver_snapping = driverSnappingRequested && aprilTagWebcam.hasActiveTags();
        boolean snapRequestEdge = driverSnappingRequested && !previousSnapRequest;
        previousSnapRequest = driverSnappingRequested;
        //telemetry.addData("tag detected value:", tagDetected);

        if (!SNAP_TO_SCORING_BIN && snapRecoveryPhase != SnapRecoveryPhase.IDLE) {
            // Safety: if mode is switched to legacy behavior, stop any in-progress lateral recovery.
            snapMecanumMover.cancelTarget();
            snapRecoveryPhase = SnapRecoveryPhase.IDLE;
            snapRecoveryMoveTargetInitialized = false;
        }

        if (shouldStartSnapRecovery(driverSnappingRequested)) {
            startSnapRecovery();
        }

        // Passes both controllers and telemetry into the drive helper.
        // Last parameter tells helper whether auto-snap mode is active.
        // Pass A / A.1 refactor note: call contract is intentionally unchanged.
        boolean legacyYawSnapActive = !SNAP_TO_SCORING_BIN && snapMecanumMover.isSnapTurnActive();
        boolean snapRoutineActive = driver_snapping || snapRecoveryPhase != SnapRecoveryPhase.IDLE || legacyYawSnapActive;
        snapMecanumMover.loop(gamepad1, gamepad2, telemetry, snapRoutineActive);

        if (SNAP_TO_SCORING_BIN && snapRecoveryPhase != SnapRecoveryPhase.IDLE) {
            runSnapRecoveryRoutine();
            telemetry.addData("Snap recovery", snapRecoveryPhase.name());
        } else if (driver_snapping) {
            if (SNAP_TO_SCORING_BIN) {
                snap_to_tag(); // Extra correction only when snap mode is requested and tag is detected.
            } else {
                runLegacySnapMode(snapRequestEdge);
            }
            addSnapToTagTelemetry();
        } else if (!SNAP_TO_SCORING_BIN && snapMecanumMover.isSnapTurnActive()) {
            // Finish legacy yaw snap cleanly even if driver released the button mid-turn.
            snapMecanumMover.stepSnapTurn(telemetry);
        }

        boolean intakeRequested = gamepad2.left_trigger > ScoringConfig.TRIGGER_THRESHOLD;
        boolean feederRequested = gamepad2.b
                || (ScoringConfig.intake_side_and_stratight_combined && intakeRequested);
        boolean shooterRequested = gamepad2.right_trigger > ScoringConfig.TRIGGER_THRESHOLD;
        boolean windmillRequested = shooterRequested && gamepad2.a;
        boolean shooterTuningRequested = gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.left_bumper;
        double shooterYawAdjustRpm = 0.0;

        if (windmillRequested) {
            double yawDegrees = Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            if (yawDegrees > ScoringConfig.SHOOTER_YAW_ADJUST_THRESHOLD_DEGREES) {
                shooterYawAdjustRpm = ScoringConfig.SHOOTER_YAW_ADJUSTER;
            }
        }

        if (intakeRequested || feederRequested || windmillRequested) {
            intakeController.applyIntakeAndWindmillControl(intakeRequested, feederRequested, windmillRequested);
            intakeRoutineActive = true;
        } else if (intakeRoutineActive) {
            intakeController.stop();
            intakeRoutineActive = false;
        }

        if (shooterRequested || shooterTuningRequested) {
            shooterController.updateTargetRpmFromDriverButtons(gamepad2);
            shooterController.applyShooterSpeedControl(shooterRequested, shooterYawAdjustRpm);
            shooterRoutineActive = true;

            if (shooterRequested) {
                double shooterRpm = shooterController.getLastRpm();
                double shooterTargetRpm = shooterController.getTargetRpm();
                double shooterPower = shooterController.getLastPower();
                double rpmError = shooterTargetRpm - shooterRpm;
                boolean shooterPowerSaturated = Math.abs(shooterPower) >= 0.999;

                telemetry.addData("Shooter RPM", "%.1f", shooterRpm);
                telemetry.addData("Shooter Target RPM", "%.1f", shooterTargetRpm);
                telemetry.addData("Shooter Baseline RPM", "%.1f", shooterController.getBaselineRpm());
                telemetry.addData("Shooter Power", "%.3f", shooterPower);
                telemetry.addData("Shooter Power Saturated", shooterPowerSaturated ? "YES" : "NO");
                telemetry.addData("Shooter RPM Error", "%.1f", rpmError);
                telemetry.addData("Shooter Estimated Max Seen RPM", "%.1f", shooterController.getMaxSeenRpm());
                telemetry.addData("Shooter Ready", shooterController.isReady() ? "YES" : "NO");
                telemetry.addData("Intake Active", intakeRequested);
                telemetry.addData("Windmill Active", windmillRequested);
            }
        } else if (shooterRoutineActive) {
            shooterController.stop();
            shooterRoutineActive = false;
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (intakeController != null) {
            intakeController.stop();
        }

        if (shooterController != null) {
            shooterController.stop();
        }

        if (lightIndicator != null) {
            lightIndicator.state(false);
        }

        intakeRoutineActive = false;
        shooterRoutineActive = false;
    }
}
