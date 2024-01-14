package org.firstinspires.ftc.teamcode.Toros;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
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
import org.opencv.core.Mat;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import java.util.List;
import java.util.Queue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name = "Autonomus")


public class Autonomus extends LinearOpMode {
    public static double p = 0.03, i = 0.0022, d = 0.001;
    public static double f = -0.05;
    public static int target = -100;
    private final double ticks_in_degrees = 1440 / 180;
    private PIDController controller;

    private DcMotorEx Arm1;

    private Servo servo;
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
        Arm1 = hardwareMap.get(DcMotorEx.class, "Arm1");
        initvision();
        controller = new PIDController(p, i, d); // <- Hey PID you should know what that is
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        if (opModeIsActive()) {
            resetRuntime();
            while (opModeIsActive()) {
                telem();
                if (getRuntime() < 3) {
                    servoPos = 0;
                } else if (getRuntime() > 4 && getRuntime() < 7) {
                    servoPos = 90;
                } else if (getRuntime() > 10) {
                    servoPos = 180;
                }


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
        servo = hardwareMap.get(Servo.class, "Servo");
        servo.setPosition(servoPos);
        telemetry.addData("spike", spike);
        telemetry.addData("Servo Pos", servoPos);
        telemetry.addData("Color", cone);
        telemetry.addData("Time", getRuntime());
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        telemetry.addData("# AprilTags Detected", currentDetections.size());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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

            if (recognition.getLabel() == "BluCone") {
                cone = Color.BLUE;
            } else if (recognition.getLabel() == "RedCone") {
                cone = Color.RED;
            }

            if (currentRecognitions.size() == 1 && servoPos == 0 && getRuntime() < 3) {
                spike = 1;
            }
            if (currentRecognitions.size() == 1 && servoPos == 90 && getRuntime() > 5 && getRuntime() < 7) {
                spike = 2;
            }
            if (currentRecognitions.size() == 1 && servoPos == 180 && getRuntime() > 10) {
                spike = 3;
            }
            switch (cone) {
                case RED:
                    if (spike == 1) {

                    } else if (spike == 2) {

                    } else {

                    }
                case BLUE:
                    if (spike == 1) {
                        Trajectory tra1 = drive.trajectoryBuilder(new Pose2d(12, 62, Math.toRadians(270)))
                                .forward(27)
                                .build();
                        Trajectory tra2 = drive.trajectoryBuilder(tra1.end())
                                .forward(2)
                                .back(5)
                                .build();
                        Trajectory tra3 = drive.trajectoryBuilder(tra2.end())
                                .lineTo(new Vector2d(23,54))
                                .splineToLinearHeading(new Pose2d(41,35,Math.toRadians(180)),Math.toRadians(270))
                                .back(7)
                                .build();
                        TrajectorySequence tra4 = drive.trajectorySequenceBuilder(tra3.end())
                                .strafeRight(24)
                                .back(10)
                                .build();
                        drive.followTrajectory(tra1);
                        drive.turn(Math.toRadians(90));
                        drive.followTrajectory(tra2);
                        drive.followTrajectory(tra3);
                        arm(1250);
                        drive.followTrajectorySequence(tra4);


                    } else if (spike == 2) {

                    } else {

                    }
            }

        }

    }

    private double arm(int target) {
        double powerA = 0;
        int armPos = Arm1.getCurrentPosition();

        //Now the fun begins
        controller.setPID(p, i, d); // sets the terms
        double pid = controller.calculate(armPos, target); /// Remember that very funny equation for PID. Well I told the computer to do my math homework
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f; // Creates a number to get an angle related to the target and ticks and muliplies by our f term
        powerA = pid + ff; // Gives the power to the motor

        return powerA;
    }
}






