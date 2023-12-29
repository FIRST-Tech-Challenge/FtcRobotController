package org.firstinspires.ftc.teamcode.Toros;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.DriveRR.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import java.util.List;
import java.util.Queue;

@Autonomous(name = "Autonomus")


public class Autonomus extends LinearOpMode {
    Color cone = Color.None;

    int spike = 0;
    int servoPos = 0;

    enum Color {
        RED,
        BLUE,
        None
    }

    private static final String Model = "/sdcard/FIRST/tflitemodels/model.tflite";

    private static final String[] Labels = {
            "RedCone",
            "BluCone"
    };

    private TfodProcessor tfod;
    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        initvision();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telem();
                telemetry.update();
            }
        }
        sleep(20);
        visionPortal.close();
    }

    private void initvision() {
        tfod = new TfodProcessor.Builder()

                .setModelFileName(Model)
                .setModelLabels(Labels)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Camera"));
        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.setAutoStopLiveView(true);
        builder.addProcessors(tfod, aprilTag);
        visionPortal = builder.build();
        tfod.setMinResultConfidence(0.75f);
        tfod.setZoom(1.75);

    }

    private void telem() throws InterruptedException {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());


            String label = null;
            telemetry.addData("Color",cone );

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            telemetry.addData("Servo", servoPos);
            telemetry.addData("Spike",spike);

            while(recognition.getLabel() == null){
                servoPos += 90;
                spike +=1;
                if(recognition.getLabel() != null) {
                    label = recognition.getLabel();
                    break;
                }else {
                    sleep(2000);
                }
            }

            if(label == "RedCone"){
                cone = Color.BLUE;
            }
            else{
                cone = Color.RED;
            }

            switch (cone) {
                case RED:


                case BLUE:
                    if(spike == 1){
                        drive.trajectoryBuilder(new Pose2d(-32,62,Math.toRadians(270)))
                                .lineTo(new Vector2d(-36, 62))
                                .lineToLinearHeading(new Pose2d(-37,33))
                                .forward(13)
                                .back(15)
                                .splineToLinearHeading(new Pose2d(38,36,Math.toRadians(180)),Math.toRadians(50))
                                .back(10)
                                .build();

                    }


                }
                // end for() loop


            }

        }

}

