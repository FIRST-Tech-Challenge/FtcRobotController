package org.firstinspires.ftc.teamcode.JackBurr.Autonomous;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous
public class MoveToAprilTags extends OpMode {
    public SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    public OpenCvWebcam camera;
    public VisionPortal vp;
    public AprilTagProcessor processor;
    public double x = -999;
    public double y = -999;
    public double z = -999;
    @Override
    public void init() {
        processor = AprilTagProcessor.easyCreateWithDefaults();
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vp = VisionPortal.easyCreateWithDefaults(webcamName, processor);
    }

    @Override
    public void init_loop() {
        ArrayList<AprilTagDetection> detectionsList = processor.getDetections();
        for (AprilTagDetection detection : detectionsList) {
            try {
                if (detection.id == 1) {
                    x = detection.rawPose.x;
                    y = detection.rawPose.y;
                    z = detection.rawPose.z;
                    telemetry.addLine("Tag ID: " + String.valueOf(detection.id));
                    if (x != -999) {
                        telemetry.addLine(String.format("%.3f", x));
                        if(x != 0){
                            if (x > 0) {
                                moveRight(x, hardwareMap);
                            }
                            else {
                                moveLeft(x, hardwareMap);
                            }
                        }
                    }
                    if (y != -999) {
                        telemetry.addLine(String.format("%.3f", y));
                    }
                    if (z != -999) {
                        telemetry.addLine(String.format("%.3f", z));
                    }
                }
            } catch (Exception e) {
                telemetry.addLine(e.getMessage());
            }
        }

    }
    @Override
    public void loop() {
        vp.stopStreaming();
    }


    public Trajectory getNewLeftTrajectory(double distance, HardwareMap hardMap){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardMap);
        Trajectory new_trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(distance)
                .build();
        return new_trajectory;
    }
    public Trajectory getNewRightTrajectory(double distance, HardwareMap hardMap){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardMap);
        Trajectory new_trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(distance)
                .build();
        return new_trajectory;
    }
    public Trajectory getNewForwardTrajectory(double distance, HardwareMap hardMap){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardMap);
        Trajectory new_trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(distance)
                .build();
        return new_trajectory;
    }
    public Trajectory getNewBackwardTrajectory(double distance, HardwareMap hardMap){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardMap);
        Trajectory new_trajectory = drive.trajectoryBuilder(new Pose2d())
                .back(distance)
                .build();
        return new_trajectory;
    }
    public void moveLeft(double distance, HardwareMap hardwareMap){
        drive.followTrajectory(getNewLeftTrajectory(distance, hardwareMap));
    }
    public void moveRight(double distance, HardwareMap hardwareMap){
        drive.followTrajectory(getNewRightTrajectory(distance, hardwareMap));
    }
    public void moveForward(double distance, HardwareMap hardwareMap){
        drive.followTrajectory(getNewForwardTrajectory(distance, hardwareMap));
    }
    public void moveBackward(double distance, HardwareMap hardwareMap){
        drive.followTrajectory(getNewBackwardTrajectory(distance, hardwareMap));
    }

}
