package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.apriltesting.SkystoneDatabase;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Date;
import java.util.List;

@Autonomous(name = "Baby's First Auto")
public class babysFirstAuto extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private AprilTagProcessor aprilTag;

    public AprilTagDetection myTag;

    private VisionPortal visionPortal;
    Robot robot;

    @Override
    public void runOpMode() {

        CenterStageComputerVisionPipelines CV = new CenterStageComputerVisionPipelines(hardwareMap, telemetry);
        CenterStageComputerVisionPipelines.pos propPos = null;

        robot = new Robot(hardwareMap, telemetry, this);

        waitForStart();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 50 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            propPos = CV.propFind.position;
            telemetry.addData("Position", propPos);
        }

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            telemetry.addData("#AprilTags Detected", currentDetections.size());
            telemetry.addData("Detection String", detection.toString());
            telemetry.addData("Detection ID's", detection.id);
            if (detection.metadata != null) {
                if (detection.id == 9) {
                    myTag = detection;
                }
            }
        }

        robot.init();
        robot.forward(.3, .5);

    }

    private void initAprilTag(){

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary((SkystoneDatabase.SkystoneDatabase()))
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableCameraMonitoring(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }

}
