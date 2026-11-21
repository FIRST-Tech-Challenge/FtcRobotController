package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * Shared Limelight Python-SnapScript aiming code.
 * Not an OpMode by itself: run MainTeleOpRed or MainTeleOpBlue.
 * Dashboard tuning values live here, so both OpModes share them.
 */
@Config
public abstract class MainTeleOpBase extends LinearOpMode {

    // Alliance codes sent to the SnapScript (llrobot[7])
    protected static final int ALLIANCE_RED = 1;
    protected static final int ALLIANCE_BLUE = 2;

    /** Which alliance's clusters this OpMode is allowed to target. */
    protected abstract int getAlliance();

    private boolean clusterAllowed(int cluster) {
        if (getAlliance() == ALLIANCE_RED) {
            return cluster == 0 || cluster == 1;   // Red Scoring, Red Audience
        }
        if (getAlliance() == ALLIANCE_BLUE) {
            return cluster == 2 || cluster == 3;   // Blue Scoring, Blue Audience
        }
        return true;
    }

    private String allianceName() {
        return getAlliance() == ALLIANCE_RED ? "RED" : "BLUE";
    }

    private DcMotorEx launcher;
    private DcMotor intake;

    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;

    private Servo feeder;
    private Servo limelightServo;

    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;

    // ==========================================
    // DASHBOARD SETTINGS
    // ==========================================

    public static PIDFCoefficients Pidf =
            new PIDFCoefficients(28, 1, 0, 13);

    public static double vel = 1920;
    public static double position = 0.15;

    // ==========================================
    // AUTOMATIC DRIVE SETTINGS
    // ==========================================
    public static double axialAdjust = -0.7;
    public static double axialForwardGain = 0.03;
    public static double maxAutoAxial = 0.5;

    public static double yawGain = 0.025;
    public static double rotationVelocityGain = 0.12;
    public static double maxAutoYaw = 0.5;

    public static double axialDeadband = 1.0;
    public static double yawDeadband = 0.7;

    public static final int SMOOTHING_WINDOW = 10;
    public static double SMOOTHING_ALPHA = 0.2;
    public static double yawOffsetDegrees = -9.0;

    // Limelight pipeline slot that holds the Python SnapScript
    public static int pythonPipeline = 3;
    // Ignore Limelight results older than this (ms)
    public static double maxStalenessMs = 150;

    // "TARGET LOCKED" when the smoothed aim is inside these windows
    // Yaw: |tx| < lockYawTolerance
    // Axial: |ty - axialAdjust| < lockAxialTolerance (axialAdjust is where axial drive stops)
    public static double lockYawTolerance = 0.7;
    public static double lockAxialTolerance = 1.5;

    private double[] aimYawHistory = new double[SMOOTHING_WINDOW];
    private double[] aimAxialHistory = new double[SMOOTHING_WINDOW];

    private int smoothingIndex = 0;
    private int previousSmoothingCluster = -1;
    private int smoothingCount = 0;

    private double smoothedAimYaw = 0;
    private double smoothedAimAxial = 0;

    // ==========================================
    // DRIVE VARIABLES
    // ==========================================

    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;

    double axial;
    double lateral;
    double yaw;

    boolean intakeActive = false;

    // ==========================================
    // PINPOINT VELOCITY
    // ==========================================

    double forwardVelocity;
    double sidewaysVelocity;
    double angularVelocity;

    // ==========================================
    // AIMING VARIABLES
    // ==========================================

    String[] clusters = {
            "Red scoring",
            "Red Audience",
            "Blue Scoring",
            "Blue Audience"
    };

    double aimYaw = 0;
    double aimAxial = 0;

    double targetLauncherVelocity = -1920;

    double autoYaw = 0;
    double yawError = 0;
    double axialError = 0;
    boolean targetFound = false;

    int selectedCluster = -1;

    int clusterTagCount = 0;
    int totalTags = 0;
    double targetRangeIn = 0;
    double headingDeg = 0;
    boolean aimLocked = false;

    // ==========================================
    // MAIN OP MODE
    // ==========================================

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(
                telemetry,
                dashboard.getTelemetry()
        );

        // ==========================================
        // HARDWARE INITIALIZATION
        // ==========================================

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");

        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");

        feeder = hardwareMap.get(Servo.class, "feeder");
        limelightServo = hardwareMap.get(Servo.class, "limelightservo");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class, "pinpoint"
        );

        // ==========================================
        // MOTOR CONFIGURATION
        // ==========================================

        launcher.setDirection(DcMotor.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, Pidf
        );

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ==========================================
        // PINPOINT CONFIGURATION
        // ==========================================

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.update();

        // ==========================================
        // LIMELIGHT CONFIGURATION
        // ==========================================

        feeder.setPosition(0.04);

        limelight.pipelineSwitch(pythonPipeline);
        limelight.start();

        dashboard.startCameraStream(limelight, 10);

        telemetry.addLine("Limelight Aim " + allianceName() + " Ready");
        telemetry.update();

        waitForStart();

        // ==========================================
        // MAIN LOOP
        // ==========================================

        while (opModeIsActive()) {

            targetLauncherVelocity = vel;
            autoYaw = 0;
            yawError = 0;
            axialError = 0;

            // ==========================================
            // LAUNCHER PIDF
            // ==========================================

            launcher.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER, Pidf
            );

            // ==========================================
            // FEEDER
            // ==========================================

            if (gamepad1.left_trigger >= 0.3) {
                feeder.setPosition(0.04);
            } else {
                feeder.setPosition(0.3);
            }

            if (gamepad1.dpad_down) {
                feeder.setPosition(
                        Math.max(feeder.getPosition() - 0.01, 0)
                );
            }

            if (gamepad1.dpad_up) {
                feeder.setPosition(
                        Math.min(feeder.getPosition() + 0.01, 1)
                );
            }

            // ==========================================
            // INTAKE
            // ==========================================

            if (gamepad1.rightBumperWasPressed()) {
                intakeActive = !intakeActive;
            }

            intake.setPower(intakeActive ? (gamepad1.right_trigger >= 0.5) ? 0.35 : 0.8 : 0);

            // ==========================================
            // PINPOINT VELOCITY
            // ==========================================

            pinpoint.update();

            forwardVelocity =
                    pinpoint.getVelX(DistanceUnit.MM) / 1000.0;

            sidewaysVelocity =
                    pinpoint.getVelY(DistanceUnit.MM) / 1000.0;

            angularVelocity =
                    pinpoint.getHeadingVelocity(
                            UnnormalizedAngleUnit.RADIANS
                    );

            // ==========================================
            // MANUAL DRIVE
            // ==========================================

            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            targetFound = false;
            selectedCluster = -1;
            aimLocked = false;

            // ==========================================
            // LIMELIGHT PYTHON SNAPSCRIPT OUTPUT
            // [valid, tx, ty, cluster, tagCount, rangeM, totalTags, frame]
            // ==========================================

            LLResult result = limelight.getLatestResult();

            double[] py = null;
            boolean fresh = false;

            clusterTagCount = 0;
            totalTags = 0;
            targetRangeIn = 0;

            if (result != null) {
                py = result.getPythonOutput();
                fresh = result.getStaleness() < maxStalenessMs;
            }

            if (py != null && py.length >= 7) {
                totalTags = (int) py[6];
            }

            if (fresh && py != null && py.length >= 6 && py[0] >= 0.5) {

                int cluster = (int) Math.round(py[3]);

                // Double-check the alliance here too, in case the
                // SnapScript on the Limelight is an older version
                if (cluster >= 0 && cluster < clusters.length
                        && clusterAllowed(cluster)) {
                    selectedCluster = cluster;

                    // Script reconstructs the cluster CENTER from the known
                    // tag layout, so this is valid even with 1 tag visible
                    aimYaw = py[1];
                    aimAxial = py[2];

                    clusterTagCount = (int) py[4];
                    targetRangeIn = py[5] * 39.37;
                }
            }

            if (selectedCluster != -1) {

                //Fix yaw mistakes

                targetFound = true;

                // ==========================================
                // SMOOTHING
                // ==========================================


                // Reset history if the selected cluster changes
                if (selectedCluster != previousSmoothingCluster) {

                    smoothingIndex = 0;
                    smoothingCount = 0;

                    previousSmoothingCluster = selectedCluster;
                }


                // Store newest readings
                aimYawHistory[smoothingIndex] = aimYaw;
                aimAxialHistory[smoothingIndex] = aimAxial;

// Advance circular buffer
                smoothingIndex = (smoothingIndex + 1) % SMOOTHING_WINDOW;

                if (smoothingCount < SMOOTHING_WINDOW) {
                    smoothingCount++;
                }

                // Calculate exponentially weighted averages
                double yawSum = 0;
                double axialSum = 0;
                double totalWeight = 0;

// Find newest reading
                int newestIndex =
                        (smoothingIndex - 1 + SMOOTHING_WINDOW)
                                % SMOOTHING_WINDOW;

                for (int age = 0; age < smoothingCount; age++) {

                    // Find the actual buffer position for this reading
                    int index =
                            (newestIndex - age + SMOOTHING_WINDOW)
                                    % SMOOTHING_WINDOW;

                    // Newer readings receive greater weight
                    double weight =
                            Math.pow(1.0 - SMOOTHING_ALPHA, age);

                    yawSum += aimYawHistory[index] * weight;
                    axialSum += aimAxialHistory[index] * weight;

                    totalWeight += weight;
                }

                smoothedAimYaw = yawSum / totalWeight;
                smoothedAimAxial = axialSum / totalWeight;

                // ==========================================
                // LOCK CHECK (drives the TARGET LOCKED HUD banner)
                // ==========================================

                aimLocked =
                        Math.abs(smoothedAimYaw) < lockYawTolerance &&
                                Math.abs(smoothedAimAxial - axialAdjust) < lockAxialTolerance;


                // ==========================================
                // AUTOMATIC AIMING
                // ==========================================

                if (gamepad1.right_trigger >= 0.5) {

                    yawError =
                            Math.abs(smoothedAimYaw) < yawDeadband
                                    ? 0 : smoothedAimYaw;

                    axialError =
                            Math.abs(smoothedAimAxial) < axialDeadband
                                    ? 0 : smoothedAimAxial;

                    autoYaw =
                            -yawError * yawGain
                                    - angularVelocity * rotationVelocityGain;

                    autoYaw = Math.max(
                            -maxAutoYaw,
                            Math.min(maxAutoYaw, autoYaw)
                    );

                    axial = (-axialError + axialAdjust) * axialForwardGain;

                    axial = Math.max(-maxAutoAxial, Math.min(maxAutoAxial, axial)
                    );

                    yaw = -autoYaw;

                    if ((Math.abs(axial) <= 0.05) && (Math.abs(yaw) <= 0.05)) {
                        intakeActive = true;
                        feeder.setPosition(0.04);
                    }
                }
            } else {
                // Reset smoothing when no target is found
                smoothingIndex = 0;
                smoothingCount = 0;

                previousSmoothingCluster = -1;

                smoothedAimYaw = 0;
                smoothedAimAxial = 0;
            }

            // ==========================================
            // STOP IF AUTOMATIC AIMING HAS NO TARGET
            // ==========================================

            if (gamepad1.right_trigger >= 0.5 && !targetFound) {
                axial = 0;
                lateral = 0;
                yaw = 0;
            }

            // ==========================================
            // LAUNCHER
            // ==========================================

            targetLauncherVelocity = vel;
            launcher.setVelocity(targetLauncherVelocity);

            // ==========================================
            // DRIVE CALCULATIONS
            // ==========================================

            frontLeftPower = axial + lateral + yaw;
            frontRightPower = axial - lateral - yaw;

            backLeftPower = axial - lateral + yaw;
            backRightPower = axial + lateral - yaw;

            limelightServo.setPosition(position);

            double max = JavaUtil.maxOfList(
                    JavaUtil.createListWith(
                            Math.abs(frontLeftPower),
                            Math.abs(frontRightPower),
                            Math.abs(backLeftPower),
                            Math.abs(backRightPower)
                    )
            );

            if (max > 1) {

                frontLeftPower /= max;
                frontRightPower /= max;

                backLeftPower /= max;
                backRightPower /= max;
            }

            // ==========================================
            // SEND DRIVE POWERS
            // ==========================================

            FL.setPower(frontLeftPower);
            FR.setPower(frontRightPower);

            BL.setPower(backLeftPower);
            BR.setPower(backRightPower);

            // ==========================================
            // SEND DATA TO SNAPSCRIPT HUD
            // [heading, autoAim, launcherVel, launcherTarget, yawRateDeg,
            //  aimLocked, robotConnected, alliance]
            // ==========================================

            headingDeg = pinpoint.getHeading(AngleUnit.DEGREES);

            limelight.updatePythonInputs(new double[]{
                    headingDeg,
                    gamepad1.right_trigger >= 0.5 ? 1 : 0,
                    launcher.getVelocity(),
                    targetLauncherVelocity,
                    Math.toDegrees(angularVelocity),
                    aimLocked ? 1 : 0,
                    1,
                    getAlliance()
            });

            // ==========================================
            // COLLAPSED TELEMETRY
            // ==========================================

            // Limelight / SnapScript

            telemetry.addData(
                    "Limelight",
                    "Result: %s | Fresh: %s | Tags: %d | Cluster tags: %d",
                    result != null,
                    fresh,
                    totalTags,
                    clusterTagCount
            );

            telemetry.addData(
                    "Range",
                    "%.1f in",
                    targetRangeIn
            );

            // Selected cluster

            telemetry.addData(
                    "Alliance",
                    allianceName() + " only"
            );

            telemetry.addData(
                    "Selected Cluster",
                    selectedCluster == -1
                            ? "None"
                            : clusters[selectedCluster]
            );

            telemetry.addData(
                    "Target",
                    "Found: %s | LOCKED: %s",
                    targetFound,
                    aimLocked
            );

            // Aiming values

            telemetry.addData(
                    "Aim Yaw",
                    "Raw: %.2f | Smooth: %.2f | Error: %.2f",
                    aimYaw,
                    smoothedAimYaw,
                    yawError
            );

            telemetry.addData(
                    "Aim Axial",
                    "Raw: %.2f | Smooth: %.2f | Error: %.2f",
                    aimAxial,
                    smoothedAimAxial,
                    axialError
            );

            // Automatic drive

            telemetry.addData(
                    "Auto Drive",
                    "Yaw: %.3f | Axial: %.3f",
                    autoYaw,
                    axial
            );

            // Odometry

            telemetry.addData(
                    "Pinpoint Velocity",
                    "Forward: %.3f | Sideways: %.3f | Rotation: %.3f",
                    forwardVelocity,
                    sidewaysVelocity,
                    angularVelocity
            );

            // Launcher

            telemetry.addData(
                    "Launcher",
                    "Target: %.0f | Actual: %.0f | Current: %.0f mA",
                    targetLauncherVelocity,
                    launcher.getVelocity(),
                    launcher.getCurrent(CurrentUnit.MILLIAMPS)
            );

            // Mechanisms

            telemetry.addData(
                    "Mechanisms",
                    "Intake: %s | Feeder: %.2f",
                    intakeActive,
                    feeder.getPosition()
            );

            // Drive motor powers

            telemetry.addData(
                    "Drive Powers",
                    "FL: %.2f | FR: %.2f | BL: %.2f | BR: %.2f",
                    frontLeftPower,
                    frontRightPower,
                    backLeftPower,
                    backRightPower
            );

            telemetry.update();
        }
    }
}