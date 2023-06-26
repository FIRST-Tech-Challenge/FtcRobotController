package org.firstinspires.ftc.teamcode.commandBased.opmodes;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.commandBased.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.AutoDrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AutoOpMode extends BaseOpMode {

    protected AutoDrivetrainSubsystem drive;
    protected SampleMecanumDrive rrDrive;

    protected OpenCvCamera camera;
    protected AprilTagDetectionPipeline aprilTagPipeline;
    protected double fx = 578.272;
    protected double fy = 578.272;
    protected double cx = 402.145;
    protected double cy = 221.506;
    protected double tagsize = 0.166;

    protected AprilTagDetection tagOfInterest = null;

    protected double rollingSum = 0;

    protected enum TagPos {
        LEFT(0),
        MIDDLE(1),
        RIGHT(2),
        NULL(3);

        private final double value;

        TagPos(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }

    protected TagPos tagPos = TagPos.NULL;

    @Override
    public void initialize() {
        super.initialize();

        rrDrive = new SampleMecanumDrive(hardwareMap);

        drive = new AutoDrivetrainSubsystem(rrDrive, false);

        aprilTagInitialization();
    }

    @Override
    public void run() {
        super.run();
    }

    protected void aprilTagInitialization() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName()
        );

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(
                        WebcamName.class,
                        "Webcam 1"
                ),
                cameraMonitorViewId
        );

        aprilTagPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    protected void updateCurrentTag() {
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;
                for (AprilTagDetection tag : currentDetections) {
                    if(tag.id == TagPos.LEFT.value ||
                       tag.id == TagPos.MIDDLE.value ||
                       tag.id == TagPos.RIGHT.value) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            } else {
                telemetry.addLine("Don't see tag of interest");

                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }
            telemetry.update();
        }
        sleep(20);
    }
    
    protected void determinePathFromTag() {
        camera.closeCameraDevice();
        if (tagOfInterest == null) {
            tagPos = TagPos.NULL;
        } else if (tagOfInterest.id == TagPos.LEFT.value) {
            tagPos = TagPos.LEFT;
        } else if (tagOfInterest.id == TagPos.MIDDLE.value) {
            tagPos = TagPos.MIDDLE;
        } else if (tagOfInterest.id == TagPos.RIGHT.value) {
            tagPos = TagPos.RIGHT;
        }
    }

    @SuppressLint("DefaultLocale")
    protected void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }

}
