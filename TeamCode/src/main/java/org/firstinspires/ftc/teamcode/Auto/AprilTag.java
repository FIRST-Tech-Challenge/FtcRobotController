package org.firstinspires.ftc.teamcode.Auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class AprilTag extends LinearOpMode {

    private DistanceSensor distanceSensor;
    private DcMotor fleft;
    private DcMotor fright;
    private DcMotor bright;
    private DcMotor bleft;

    @Override
    public void runOpMode() throws InterruptedException {

        bleft = hardwareMap.get(DcMotor.class, "bleft");
        bright = hardwareMap.get(DcMotor.class, "bright");
        fleft = hardwareMap.get(DcMotor.class, "fleft");
        fright = hardwareMap.get(DcMotor.class, "fright");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        bleft.setDirection(DcMotorSimple.Direction.REVERSE);
        fleft.setDirection(DcMotorSimple.Direction.REVERSE);

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                int detectedTagID = tag.id;

                if (detectedTagID == 4) {
                    // The robot has detected AprilTag ID 4, stop moving
                    bleft.setPower(0);
                    bright.setPower(0);
                    fleft.setPower(0);
                    fright.setPower(0);

                    telemetry.addData("Detected AprilTag ID 4", true);
                    telemetry.update();

                    // Add a delay to allow time for telemetry to update and for the robot to stop
                    sleep(1000); // Adjust the sleep time as needed
                } else {
                    // AprilTag not detected, keep moving
                    bleft.setPower(-0.1);
                    bright.setPower(0.1);
                    fleft.setPower(0.1);
                    fright.setPower(-0.1);
                }
            } else {
                // No AprilTags detected, keep moving
                bleft.setPower(-0.1);
                bright.setPower(0.1);
                fleft.setPower(0.1);
                fright.setPower(-0.1);
            }
        }
    }
}


//                if (detectedTagID == 4){
//                    bleft.setPower(0);
//                    bright.setPower(0);
//                    fright.setPower(0);
//                    fleft.setPower(0);
//
//                    sleep(5000);
//                } else{
//                    bleft.setPower(-0.2);
//                    bright.setPower(0.2);
//                    fleft.setPower(0.2);
//                    fright.setPower(-0.2);
//                }

//            if (tagProcessor.getDetections().size() > 0) {
//                AprilTagDetection tag = tagProcessor.getDetections().get(0);
//
//                if (tag != null) {
//                    int detectedTagID = tag.id;
//
//                    if (detectedTagID == 4) {
//
//                        telemetry.addData("Detected AprilTag ID 4", true);
//
//                        // Move the robot (example: stop the robot)
//                        bleft.setPower(0);
//                        bright.setPower(0);
//                        fleft.setPower(0);
//                        fright.setPower(0);
//                    } else {
//                        telemetry.addData("Detected AprilTag ID", detectedTagID);
//
//                        // Move the robot (example: continue moving with a certain power)
//                        bleft.setPower(-0.2);
//                        bright.setPower(0.2);
//                        fleft.setPower(0.2);
//                        fright.setPower(-0.2);
//                    }
//                } else {
//                    // Handle the case where `tag` is null (no AprilTag detected)
//                    telemetry.addData("No AprilTags detected", true);
//
//                    bleft.setPower(-0.2);
//                    bright.setPower(0.2);
//                    fleft.setPower(0.2);
//                    fright.setPower(-0.2);
//                }
//
//            } else{
//                telemetry.addData("No AprilTags detected", true);
//
//                // Keep Moving
//                bleft.setPower(-0.2);
//                bright.setPower(0.2);
//                fleft.setPower(0.2);
//                fright.setPower(-0.2);
//            }
//            telemetry.update();

