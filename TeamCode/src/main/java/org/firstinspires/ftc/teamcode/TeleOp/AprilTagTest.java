package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.sensors.AprilTagWebcam;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "AprilTag Test", group = "Test")
public class AprilTagTest extends LinearOpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();


    @Override
    public void runOpMode() throws InterruptedException {

        aprilTagWebcam.init(hardwareMap, telemetry);

        waitForStart();



        while (opModeIsActive()) {
            aprilTagWebcam.update();
            telemetry.addData("Detections", aprilTagWebcam.getDetectedTags().size());
            for (AprilTagDetection d : aprilTagWebcam.getDetectedTags()) {
                telemetry.addData("Seen ID", aprilTagWebcam.getDetectedTags());
            }
            telemetry.update();
            aprilTagWebcam.update();
            AprilTagDetection id30 = aprilTagWebcam.getSpecificID(30);
            aprilTagWebcam.myIphone(id30);


        }
    }
}