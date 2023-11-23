package org.firstinspires.ftc.masters.apriltesting;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@Config
@TeleOp(name="AprilTagMoves")
public class aprilTagButItMoves extends LinearOpMode {

    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;

    double x = 0;
    double y = 0;
    
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private AprilTagProcessor aprilTag;

    public AprilTagDetection myTag;

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
        rightRearMotor = hardwareMap.dcMotor.get("backRight");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initAprilTag();

        waitForStart();

        while (opModeIsActive()) {

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


            if (myTag != null) {
                telemetry.addData("MyTag X", myTag.ftcPose.x);
                telemetry.addData("MyTag Y", myTag.ftcPose.y);
                telemetry.addData("MyTag Z", myTag.ftcPose.z);
                if (myTag.ftcPose.z > .8) {
                    leftFrontMotor.setPower(.1);
                    rightFrontMotor.setPower(.1);
                    leftRearMotor.setPower(.1);
                    rightRearMotor.setPower(.1);
                } else if (myTag.ftcPose.z < -.8) {
                    leftFrontMotor.setPower(-.1);
                    rightFrontMotor.setPower(-.1);
                    leftRearMotor.setPower(-.1);
                    rightRearMotor.setPower(-.1);
                } else if (myTag.ftcPose.y > 8) {
                    leftFrontMotor.setPower(-.3);
                    rightFrontMotor.setPower(.3);
                    leftRearMotor.setPower(.3);
                    rightRearMotor.setPower(-.3);
                } else {
                    leftFrontMotor.setPower(0);
                    rightFrontMotor.setPower(0);
                    leftRearMotor.setPower(0);
                    rightRearMotor.setPower(0);
                }
            }

            telemetry.update();

        }
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

            }   // end method initAprilTag()

        }
