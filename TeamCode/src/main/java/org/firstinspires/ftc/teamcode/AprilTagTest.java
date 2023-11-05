package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous
public class AprilTagTest extends LinearOpMode {

    DcMotor m1, m2, m3, m4;
    BNO055IMU imu;
    ColorSensor colorSensor;
    Servo backServo;
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .enableLiveView(true)
                //.setCameraResolution(new Size(640, 480)) // not working for some reason
                .build();

        Orientation orientation;

        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("April tags detected", "%d", tagProcessor.getDetections().size());
            for (int i = 0; i < tagProcessor.getDetections().size(); i++) {
                AprilTagDetection tag = tagProcessor.getDetections().get(i);
                telemetry.addData("April tag detection " + (i + 1), "X:" + (float) Math.round(tag.ftcPose.x * 100) / 100 + ", Y:" + (float) Math.round(tag.ftcPose.y * 100) / 100 + ", Z:" + (float) Math.round(tag.ftcPose.z * 100) / 100);
            }
            telemetry.update();
        }


    }
}
