package org.firstinspires.ftc.teamcode.mechanisms;

import android.annotation.SuppressLint;
import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Turret {
    private Telemetry telemetry;

    // ---------- AprilTag ----------
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private final List<AprilTagDetection> allianceColorTags = new ArrayList<>();

    // ---------- Alignment ----------
    private DcMotor turnMotor;
    private double kP = 0.013;
    private double kD = 0.0004;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.5;

    private final ElapsedTime timer = new ElapsedTime();

    // ---------- Intake & Shooting Mechanism
    private NormalizedColorSensor colorSensor;

    private final float[] hsv = new float[3];
    private DcMotor shooter;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // ---------- AprilTag ----------
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "webcam1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);
        visionPortal = builder.build();

        // ---------- Alignment ----------

        turnMotor = hwMap.get(DcMotor.class, "turn_motor");
        turnMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turnMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnMotor.setPower(0.0);
        timer.reset();

        // ---------- Intake & Shooting ----------
//        shooter = hwMap.get(DcMotor.class, "shooter");
//        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        shooter.setDirection(DcMotor.Direction.REVERSE);
//
//        colorSensor = hwMap.get(NormalizedColorSensor.class, "left_inner_color_sensor");
    }

    public void updateTagProcessor() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        detectedTags = (detections != null) ? detections : Collections.emptyList();
    }

    @SuppressLint("DefaultLocale")
    public void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if (detectedId == null) return;

        if (detectedId instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detectedId;
            if (clusterDet.metadata != null) {
                telemetry.addLine(String.format("\n==== %s", clusterDet.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f (cm)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f (cm, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
            } else {
                telemetry.addLine("\n==== Unknown Cluster");
            }
        }
    }

    public List<AprilTagDetection> getTagByColor(IntakeLayers.DetectedColor enemyColor) {
        String requiredPrefix = (enemyColor == IntakeLayers.DetectedColor.RED) ? "B" : "R";

        allianceColorTags.clear();
        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                if (clusterDet.metadata != null && clusterDet.metadata.name != null) {
                    if (clusterDet.metadata.name.startsWith(requiredPrefix)) {
                        allianceColorTags.add(clusterDet);
                    }
                }
            }
        }
        return allianceColorTags;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    // ---------- Alignment ----------
    public void setkP(double newKP) {
        kP = newKP;
    }
    public double getkP() {
        return kP;
    }
    public void setkD(double newKD) {
        kD = newKD;
    }
    public double getkD() {
        return kD;
    }
    public void resetTimer() {
        timer.reset();
    }

    public void updateTurret(List<AprilTagDetection> detections) {
        double deltaTime = timer.seconds();
        timer.reset();

        if (detections == null || detections.isEmpty()) {
            turnMotor.setPower(0.0);
            lastError = 0;
            return;
        }
        double bearingAvg = detections.get(0).ftcPose.bearing;

        double error = (goalX-bearingAvg);

        if (Math.abs(error) < angleTolerance) {
            turnMotor.setPower(0.0);
            lastError = error;
            return;
        }

        double pTerm = error * kP;
        double dTerm = 0;

        if (deltaTime > 0.001) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }

        double power = pTerm + dTerm;
        power = Range.clip(power, -1.0, 1.0);
        turnMotor.setPower(power);

        lastError = error;
    }

    // ---------- Intake & Shooting ----------
    public IntakeLayers.DetectedColor getDetectedColor(Telemetry telemetry) {
        if (colorSensor == null) {
            return IntakeLayers.DetectedColor.UNKNOWN;
        }

        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        int r, g, b;
        r = (int) (colors.red * 255.0f);
        g = (int) (colors.green * 255.0f);
        b = (int) (colors.blue * 255.0f);

        Color.RGBToHSV(r, g, b, hsv);

        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];

        // telemetry part only for calibration; comment-out in real game
        telemetry.addData("hue (inner)", hue);
        telemetry.addData("saturation (inner)", saturation);
        telemetry.addData("value (inner)", value);

        if (colorSensor instanceof DistanceSensor) {
            double distanceCm = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);

            telemetry.addData("Distance (cm)", distanceCm);

            if (distanceCm > 3.0) {
                return IntakeLayers.DetectedColor.UNKNOWN;
            }
            // need to adjust these values
            if (colors.alpha < 0.05 || value < 0.1) {
                return IntakeLayers.DetectedColor.UNKNOWN;
            }
            if (hue >= 40 && hue <= 85) {
                return IntakeLayers.DetectedColor.YELLOW;
            }
            else if (hue <= 25 || hue >= 330) {
                return IntakeLayers.DetectedColor.RED;
            }
            else if (hue >= 190 && hue <= 250) {
                return IntakeLayers.DetectedColor.BLUE;
            }
        }
        return IntakeLayers.DetectedColor.UNKNOWN;
    }

    public void shoot(IntakeLayers.DetectedColor color) {
        if (color == IntakeLayers.DetectedColor.UNKNOWN) {
            shooter.setPower(0.8); // values need testing
        }
        else if (color == IntakeLayers.DetectedColor.YELLOW) {
            shooter.setPower(0.7); // values need testing
        }
        else {
            shooter.setPower(1.0); // values need testing
        }
    }

    public void manualMode() {
        if (turnMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turnMotor.setTargetPosition(0);
            turnMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turnMotor.setPower(0.4);
        }
        if (turnMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION && !turnMotor.isBusy()) {
            turnMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turnMotor.setPower(0.0);
        }
    }
}