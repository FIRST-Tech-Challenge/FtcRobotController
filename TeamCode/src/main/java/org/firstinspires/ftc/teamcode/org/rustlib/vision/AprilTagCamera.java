package org.firstinspires.ftc.teamcode.org.rustlib.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.InstantCommand;
import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.Subsystem;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.org.rustlib.rustboard.Rustboard;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class AprilTagCamera extends Subsystem {
    private final Pose3d camPose;
    public final VisionPortal visionPortal;
    public boolean cameraEnabled = false;
    private boolean streaming = false;
    AprilTagProcessor aprilTagProcessor;
    private Pose2d calculatedBotPose = new Pose2d();
    private final Runnable onDetect;
    private final CameraActivationZone[] activationZones;

    private AprilTagCamera(Builder builder) {
        camPose = builder.pose;
        onDetect = builder.onDetect;
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        aprilTagProcessor.setDecimation(builder.decimation);
        visionPortal = new VisionPortal.Builder()
                .setCamera(builder.hardwareMap.get(WebcamName.class, "backCam"))
                .addProcessor(aprilTagProcessor)
                .build();
        activationZones = builder.activationZones;
        new InstantCommand(() -> setExposure(builder.exposureMS, builder.exposureGain)).scheduleOn(() -> visionPortal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY);
    }

    public interface SetHardwareMap {
        SetRelativePose setHardwareMap(HardwareMap hardwareMap);
    }

    public interface SetRelativePose {
        Builder setRelativePose(Pose3d pose);
    }

    public static class Builder implements SetHardwareMap, SetRelativePose {
        private HardwareMap hardwareMap;
        private Pose3d pose;
        private AprilTag[] tags;
        private CameraActivationZone[] activationZones = {};
        private Runnable onDetect = () -> {
        };
        private int exposureMS = 6;
        private int exposureGain = 250;
        private int decimation = 2;

        private Builder() {

        }

        @Override
        public SetRelativePose setHardwareMap(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
            return this;
        }

        @Override
        public Builder setRelativePose(Pose3d pose) {
            this.pose = pose;
            return this;
        }

        public Builder setCameraActivationZones(CameraActivationZone... activationZones) {
            this.activationZones = activationZones;
            return this;
        }

        public Builder onDetect(Runnable onDetect) {
            this.onDetect = onDetect;
            return this;
        }

        public Builder setExposureTime(int exposureMS) {
            this.exposureMS = exposureMS;
            return this;
        }

        public Builder setExposureGain(int exposureGain) {
            this.exposureGain = exposureGain;
            return this;
        }

        public Builder setDecimation(int decimation) {
            this.decimation = decimation;
            return this;
        }

        public AprilTagCamera build() {
            return new AprilTagCamera(this);
        }
    }

    public static SetHardwareMap getBuilder() {
        return new Builder();
    }

    private void calculateBotPose() {
        ArrayList<AprilTagDetection> detections = aprilTagProcessor.getFreshDetections();
        if (detections == null) {
            return;
        }
        ArrayList<Pose3d> calculatedCamPoses = new ArrayList<>();
        boolean validDetection = false;
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && AprilTag.getTag(detection.id) != null) {
                validDetection = true;
                AprilTagPoseFtc relativeTagPose = detection.ftcPose;
                AprilTag tag = AprilTag.getTag(detection.id);
                Pose3d relativeCamPose = Pose3d.toPose3d(detection.ftcPose).negate();
                calculatedCamPoses.add(relativeCamPose.relativeTo(tag.pose));
            }
        }
        if (validDetection) {
            Pose3d camPose = Pose3d.average(calculatedCamPoses.toArray(new Pose3d[]{}));
            calculatedBotPose = camPose.relativeTo(this.camPose).toPose2d();
            onDetect.run();
        }
    }

    public void setExposure(int exposureMS, int gain) {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }

    public void enable() {
        cameraEnabled = true;
    }

    public void disable() {
        cameraEnabled = false;
    }

    public Pose2d getCalculatedBotPose() {
        return calculatedBotPose;
    }

    private void resumeStream() {
        try {
            visionPortal.resumeStreaming();
            streaming = true;
        } catch (RuntimeException e) {
            Rustboard.log(e);
        }
    }

    private void stopStream() {
        try {
            visionPortal.stopStreaming();
            streaming = false;
        } catch (RuntimeException e) {
            Rustboard.log(e);
        }
    }

    boolean withinZone() {
        boolean withinRange = activationZones.length == 0;
        for (CameraActivationZone activationZone : activationZones) {
            if (activationZone.withinZone()) {
                withinRange = true;
                break;
            }
        }
        return withinRange;
    }

    @Override
    public void periodic() {
        if (cameraEnabled) {
            if (!streaming) {
                resumeStream();
            }
            if (withinZone()) {
                calculateBotPose();
            }
        } else if (streaming) {
            stopStream();
        }
    }
}
