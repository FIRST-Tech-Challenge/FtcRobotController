package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.vision.pipeline.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp
@Config
public class AprilTagTest extends LinearOpMode {

    public static int invX=-1;
    public static int invZ=-1;
    static final double FEET_PER_METER = 3.28084;
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    double a1;

    double pythag1;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    VectorF tagPos;
    Orientation tagOrientation;
    AprilTagLibrary curLib = AprilTagGameDatabase.getCurrentGameTagLibrary();
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    /*double fx = 635.906168592f;
    double fy = 635.906168592f;
    double cx = 451.512327554f;
    double cy = 232.379726059f;*/
    double fx = 883.846030309f;
    double fy = 883.846030309f;
    double cx = 416.860113959f;
    double cy = 295.808977317f;
    // UNITS ARE METERS
    double tagsize = 0.0508f;
    int numFramesWithoutDetection = 0;

    @Override
    public void runOpMode() {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy,curLib);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        telemetry.setMsTransmissionInterval(50);

        while (opModeIsActive()) {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if (detections != null) {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if (detections.isEmpty()) {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for (AprilTagDetection detection : detections) {
                        tagPos = curLib.lookupTag(detection.id).fieldPosition;
                        tagOrientation = curLib.lookupTag(detection.id).fieldOrientation.toOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

                        double t1 = (Math.PI / 2) - Math.atan2(detection.pose.x, detection.pose.z);

                        double t2 = Math.toRadians(rot.firstAngle);

                        double t3 = (t1 + t2);

                        double c = Math.sqrt(Math.pow((detection.pose.x) * FEET_PER_METER, 2) + Math.pow((detection.pose.z) * FEET_PER_METER, 2));

                        double tx = Math.cos(t3) * c;

                        double tz = Math.sin(t3) * c;


                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
                        telemetry.addLine(String.format("Bearing: %.2f degrees", Math.atan(detection.pose.x / detection.pose.z) * (180 / Math.PI)));
                        telemetry.addLine(String.format("Elevation: %.2f degrees", Math.atan(detection.pose.y / detection.pose.z) * (180 / Math.PI)));
                        telemetry.addLine("True x:" + tx);
                        telemetry.addLine("True z:" + tz);

                        try { //may error out because tag is not in current game tag library
                            //print out tag metadata
                            telemetry.addLine(tagPos.get(0) + " " + tagPos.get(1) + " " + tagPos.get(2) + " ");
                            telemetry.addLine(tagOrientation.toString());
                        } catch (Exception ignored) {

                        }


                    }
                }

                telemetry.update();
            }

            sleep(20);
        }
    }
}