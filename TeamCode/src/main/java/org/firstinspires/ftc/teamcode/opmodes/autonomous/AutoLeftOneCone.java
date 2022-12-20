package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.TurtleRobotAuto;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Autonomous(group="drive")
public class AutoLeftOneCone extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    TurtleRobotAuto robot = new TurtleRobotAuto(this);
    static final double FEET_PER_METER = 3.28084;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    int SLIDE;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    AprilTagDetection tagOfInterest = null;
    @Override

    public void runOpMode() {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

//        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
//                .forward(45)
//                .build();
//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                .strafeRight(12.5)
//                .build();
//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//                .forward(3)
//                .build();

        TrajectorySequence goToHigh = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(19)
                .forward(45)
                .strafeRight(10.5)
                .build();
        Trajectory prepareToDrop = drive.trajectoryBuilder(goToHigh.end())
                .forward(3.12)
                .build();


        TrajectorySequence parkAtOne = drive.trajectorySequenceBuilder(prepareToDrop.end())
                .strafeLeft(12)
                .back(4)
                .turn(Math.toRadians(-90))
                .build();
        TrajectorySequence parkAtTwo = drive.trajectorySequenceBuilder(prepareToDrop.end())
                .strafeRight(10)
                .back(4)
                .turn(Math.toRadians(-90))
                .build();
        TrajectorySequence parkAtThree = drive.trajectorySequenceBuilder(prepareToDrop.end())
                .strafeRight(32.1)
                .back(22)
                .build();



        waitForStart();

        if (isStopRequested()) return;
        SLIDE = 2600;
        robot.ArmServo.setPosition(0);
//        drive.followTrajectory(traj1);
//        drive.followTrajectory(traj2);
        drive.followTrajectorySequence(goToHigh);
        LinearSlide(-0.7, SLIDE);
        LinearSlide(0, 0);
        sleep(500);
        drive.followTrajectory(prepareToDrop);
        sleep(300);
        robot.ArmServo.setPosition(1);
        sleep(300);
        //drive.followTrajectory(traj3);
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            drive.followTrajectorySequence(parkAtOne);
        } else if (tagOfInterest.id == MIDDLE) {
            drive.followTrajectorySequence(parkAtTwo);
        } else {
            drive.followTrajectorySequence(parkAtThree);
        }
        sleep(100);
        LinearSlide(0.7, 1800);
        LinearSlide(0, 0);

//        drive.followTrajectory(traj4);
//          LinearSlide(-0.75, 1700);
//       LinearSlide(0, 0);
//        LinearSlide(-0.75, 1700);
//        LinearSlide(0, 0);
//        while (slide != 1100) {
//            drive.followTrajectory(traj4);
//            drive.followTrajectory(traj1);
//            robot.ArmServo.setPower(1);
//            sleep(100);
//            robot.ArmServo.setPower(0);
//            drive.followTrajectory(traj5);
//            LinearSlide(0.75, slide);
//            robot.ArmServo.setPower(-1);
//            sleep(100);
//            robot.ArmServo.setPower(0);
//            drive.followTrajectory(traj3);
//            LinearSlide(-0.75, slide);
//            LinearSlide(0, 0);
//            slide -= 300;
//        }
//
//


    }
    public void LinearSlide(double speed, long time){
        robot.leftslidemotor.setPower(speed);
        robot.rightslidemotor.setPower(speed);
        sleep(time);

    }
    public void FrontBack(double speed, long time){
        robot.leftfrontmotor.setPower(speed);
        robot.rightfrontmotor.setPower(speed);
        robot.leftbackmotor.setPower(speed);
        robot.rightbackmotor.setPower(speed);
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}






