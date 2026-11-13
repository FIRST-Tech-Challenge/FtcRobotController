
package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@TeleOp(name = "AimTestOdom", group = "Test")
@Config
public class AimTestOdom extends LinearOpMode {

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
            new PIDFCoefficients(120, 0.67, 12, 150);

    public static double vel = 1150;
    public static double position = 0.3;

    // ==========================================
    // AUTOMATIC DRIVE SETTINGS
    // ==========================================
    public static double axialAdjust = 2.15;
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

    double targetLauncherVelocity = 1150;

    double autoYaw = 0;
    double yawError = 0;
    double axialError = 0;
    boolean targetFound = false;

    int selectedCluster = -1;

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

        limelight.pipelineSwitch(0);
        limelight.start();

        dashboard.startCameraStream(limelight, 10);

        telemetry.addLine("AimTestOdom Ready");
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

            intake.setPower(intakeActive ? 0.8 : 0);

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

            // ==========================================
            // LIMELIGHT TARGET DETECTION
            // ==========================================

            LLResult result = limelight.getLatestResult();

            int[] clusterCount = new int[4];
            double[] clusterYaw = new double[4];

            List<LLResultTypes.FiducialResult> fiducials =
                    new ArrayList<>();

            StringBuilder detectedIDs = new StringBuilder();

            if (result != null) {
                List<LLResultTypes.FiducialResult> detected =
                        result.getFiducialResults();

                if (detected != null) {
                    fiducials = detected;
                }
            }

            // ==========================================
            // GROUP APRILTAGS
            // ==========================================

            for (LLResultTypes.FiducialResult fr : fiducials) {

                int id = fr.getFiducialId();

                detectedIDs.append(id).append(" ");

                int cluster = -1;

                if (id >= 30 && id <= 33) {
                    cluster = 0;
                } else if (id >= 34 && id <= 37) {
                    cluster = 1;
                } else if (id >= 38 && id <= 41) {
                    cluster = 2;
                } else if (id >= 42 && id <= 45) {
                    cluster = 3;
                }

                if (cluster != -1) {
                    clusterCount[cluster]++;
                    clusterYaw[cluster] += fr.getTargetXDegrees();
                }
            }

// ==========================================
// SELECT CLUSTER
// ==========================================

            int highest = 0;
            double closestToZero = Double.MAX_VALUE;

            for (int i = 0; i < 4; i++) {

                if (clusterCount[i] == 0) {
                    continue;
                }

                double averageYaw =
                        clusterYaw[i] / clusterCount[i];

                double distanceFromCenter = Math.abs(averageYaw);

                // Most tags wins.
                // Ties go to the cluster closest to center.

                if (clusterCount[i] > highest ||
                        (clusterCount[i] == highest &&
                                distanceFromCenter < closestToZero)) {

                    highest = clusterCount[i];
                    closestToZero = distanceFromCenter;
                    selectedCluster = i;
                }
            }

            // ==========================================
            // SELECTED CLUSTER AIMING
            // ==========================================

            if (selectedCluster != -1) {

                List<Double> yawValues = new ArrayList<>();
                List<Double> axialValues = new ArrayList<>();

                // Collect ONLY selected cluster tags.

                for (LLResultTypes.FiducialResult fr : fiducials) {

                    int id = fr.getFiducialId();

                    boolean belongsToSelectedCluster =
                            (selectedCluster == 0 && id >= 30 && id <= 33) ||
                                    (selectedCluster == 1 && id >= 34 && id <= 37) ||
                                    (selectedCluster == 2 && id >= 38 && id <= 41) ||
                                    (selectedCluster == 3 && id >= 42 && id <= 45);

                    if (belongsToSelectedCluster) {
                        yawValues.add(fr.getTargetXDegrees());
                        axialValues.add(fr.getTargetYDegrees());
                    }
                }

                // ==========================================
                // MEDIAN YAW
                // ==========================================

                Collections.sort(yawValues);

                int yawCount = yawValues.size();

                if (yawCount > 0) {

                    if (yawCount % 2 == 0) {
                        aimYaw = (
                                yawValues.get(yawCount / 2 - 1) +
                                        yawValues.get(yawCount / 2)
                        ) / 2.0;

                    } else {
                        aimYaw = yawValues.get(yawCount / 2);
                    }
                }

                // ==========================================
                // MEDIAN AXIAL
                // ==========================================

                Collections.sort(axialValues);

                int axialCount = axialValues.size();

                if (axialCount > 0) {

                    if (axialCount % 2 == 0) {
                        aimAxial = (
                                axialValues.get(axialCount / 2 - 1) +
                                        axialValues.get(axialCount / 2)
                        ) / 2.0;

                    } else {
                        aimAxial = axialValues.get(axialCount / 2);
                    }
                }

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

                    axial = Math.max(
                            -maxAutoAxial,
                            Math.min(maxAutoAxial, axial)
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
            // COLLAPSED TELEMETRY
            // ==========================================

            // AprilTag detection

            telemetry.addData(
                    "Limelight",
                    "Result: %s | Tags: %d | IDs: %s",
                    result != null,
                    fiducials.size(),
                    detectedIDs.toString()
            );

            // Cluster counts

            telemetry.addData(
                    "Cluster Counts",
                    "RS: %d | RA: %d | BS: %d | BA: %d",
                    clusterCount[0],
                    clusterCount[1],
                    clusterCount[2],
                    clusterCount[3]
            );

            // Selected cluster

            telemetry.addData(
                    "Selected Cluster",
                    selectedCluster == -1
                            ? "None"
                            : clusters[selectedCluster]
            );

            telemetry.addData(
                    "Target Found",
                    targetFound
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
