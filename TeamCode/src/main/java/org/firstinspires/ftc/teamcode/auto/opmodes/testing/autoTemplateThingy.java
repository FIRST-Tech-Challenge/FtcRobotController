//package org.firstinspires.ftc.teamcode.auto.opmodes.testing;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.common.AprilTagDetectionPipeline;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.firstinspires.ftc.teamcode.auto.AutoHub;
//
//import java.util.ArrayList;
//
//@Autonomous(name = "autoTemplateThingy", group = "Routes")
//abstract public class autoTemplateThingy extends LinearOpMode{
//    OpenCvCamera camera;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;
//
//    static final double FEET_PER_METER = 3.28084;
//
//    // Lens intrinsics
//    // UNITS ARE PIXELS
//    // NOTE: this calibration is for the C920 webcam at 800x448.
//    // You will need to do your own calibration for other configurations!
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//
//    // UNITS ARE METERS
//    double tagsize = 0.045;
//
//    int numFramesWithoutDetection = 0;
//
//    final float DECIMATION_HIGH = 3;
//    final float DECIMATION_LOW = 2;
//    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
//    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
//    int aprilTagId;
//
//    AutoHub dispatch;
//
//    @Override
//    public void runOpMode()
//    {
//        dispatch = new AutoHub(this);
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cameraMonitorViewId"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });
//
//        waitForStart();
//        //telemetry.setMsTransmissionInterval(50);
//        while (!opModeIsActive())
//        {
//            // Calling getDetectionsUpdate() will only return an object if there was a new frame
//            // processed since the last time we called it. Otherwise, it will return null. This
//            // enables us to only run logic when there has been a new frame, as opposed to the
//            // getLatestDetections() method which will always return an object.
//            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
//
//            // If there's been a new frame...
//            if(detections != null)
//            {
//                telemetry.addData("FPS", camera.getFps());
//                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
//                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
//
//                // If we don't see any tags
//                if(detections.size() == 0)
//                {
//                    numFramesWithoutDetection++;
//
//                    // If we haven't seen a tag for a few frames, lower the decimation
//                    // so we can hopefully pick one up if we're e.g. far back
//                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
//                    {
//                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
//                    }
//                }
//                // We do see tags!
//                else
//                {
//                    numFramesWithoutDetection = 0;
//
//                    // If the target is within 1 meter, turn on high decimation to
//                    // increase the frame rate
//                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
//                    {
//                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
//                    }
//
//                    for(AprilTagDetection detection : detections)
//                    {
//                        aprilTagId = detection.id;
//                        /*
//                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//                        */
//                    }
//
//
//                }
//
//                telemetry.update();
//            }
//
//            sleep(20);
//        }
//        switch (aprilTagId) {
//            case 0: {
//                //stick code for one dot here
//                break;
//            }
//            case 1: {
//                //stick code for two dots here
//                break;
//            }
//            case 2: {
//                //stick code for three dots here
//                break;
//            }
//        }
//    }
//
//}
