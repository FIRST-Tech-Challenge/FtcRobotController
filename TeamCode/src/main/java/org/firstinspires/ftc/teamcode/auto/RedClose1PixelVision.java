package org.firstinspires.ftc.teamcode.auto;//package org.firstinspires.ftc.teamcode.Autos.BadAutons;
//
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Autos.TrajectorySequenceCommand;
////import org.firstinspires.ftc.teamcode.Vision.AprilTagFiles.AprilTagDetectionPipeline;
//import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.TeleOps.MainCode.Subsystems.Outtake;
//import org.firstinspires.ftc.teamcode.TeleOps.MainCode.Subsystems.Slides;
//import org.firstinspires.ftc.teamcode.TeleOps.MainCode.Subsystems.V4B;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.ArrayList;
//@Disabled
//@Autonomous(name="Red Close 1Pixel")
//public class RedClose1PixelVision extends CommandOpMode {
//    //Add Motors and servos not for drivebase here
//    SampleMecanumDrive drive;
//    GamepadEx gamepad;
//
//    OpenCvCamera camera;
////    AprilTagDetectionPipeline aprilTagDetectionPipeline;
//
//    static final double FEET_PER_METER = 3.28084;
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//    double tagSize = 0.035;
//    AprilTagDetection tagOfInterest = null;
//
//    int tagId = 0; //Dw about it rn
//
//    @Override
//    public void initialize() {
//        Slides slides = new Slides(hardwareMap);
//        Outtake outtake = new Outtake(gamepad, hardwareMap);
//        V4B v4b = new V4B(hardwareMap);
//        telemetry.addLine("Mechanisms Initialized Correctly...");
//        telemetry.update();
//        sleep(50);
//
//        //More camera stuff
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        telemetry.addLine("here4");
//        telemetry.update();
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        telemetry.addLine("here5");
//        telemetry.update();
////        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);
//
////        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//
//        drive = new SampleMecanumDrive(hardwareMap);
//
//        while (!isStarted() && !isStopRequested()) {
////            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//            if (currentDetections.size() != 0) {
//                boolean tagFound = false;
//
//                for (AprilTagDetection tag : currentDetections) {
//                    if (tag.id == 0 || tag.id == 1 || tag.id == 2 /*|| tag.id == 4 || tag.id == 5 || tag.id == 6 */) {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                    }
//                    tagId = tag.id;
//                }
//
//                if (tagFound) {
//                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                } else {
//                    telemetry.addLine("Don't see tag of interest :(");
//
//                    if (tagOfInterest == null) {
//                        telemetry.addLine("(The tag has never been seen)");
//                    } else {
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//
//            } else {
//                telemetry.addLine("Don't see tag of interest :(");
//
//                if (tagOfInterest == null) {
//                    telemetry.addLine("(The tag has never been seen)");
//                } else {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//
//            }
//
//            telemetry.update();
//            sleep(20);
//        }
//
//        UnworkingTrajectories.generateTrajectories(drive); //Loads trajectories from trajectories file
//
//        TrajectorySequence prop;
//        TrajectorySequence score;
//        TrajectorySequence park;
//
//        switch(tagId) {
//            case 0:
//                prop = UnworkingTrajectories.propCloseLeftRed;
//                score = UnworkingTrajectories.scoreCloseLeftRed;
//                park = UnworkingTrajectories.parkCloseLeftRed;
//                break;
//            case 1:
//                prop = UnworkingTrajectories.propCloseMidRed;
//                score = UnworkingTrajectories.scoreCloseMidRed;
//                park = UnworkingTrajectories.parkCloseMidRed;
//                break;
//            case 2:
//                prop = UnworkingTrajectories.propCloseRightRed;
//                score = UnworkingTrajectories.scoreCloseRightRed;
//                park = UnworkingTrajectories.parkCloseRightRed;
//                break;
//            default:
//                prop = UnworkingTrajectories.propCloseLeftRed;
//                score = UnworkingTrajectories.scoreCloseLeftRed;
//                park = UnworkingTrajectories.parkCloseLeftRed;
//        }
//
//
//        schedule(new SequentialCommandGroup ( //Makes the following code run one after another, like normal
//                new ParallelCommandGroup(
//                        new TrajectorySequenceCommand(drive, prop),
//                        new TrajectorySequenceCommand(drive, score),
//                        new InstantCommand(() -> {
//                            slides.goToPosition(Slides.SlidePos.LOW);
////                            v4b.togglePosition();
//                            outtake.open();
//                            new WaitCommand(2000);
//                            outtake.close();
////                            v4b.togglePosition();
//                            slides.switchMaxMin();
//                        })
//                ),
//                new TrajectorySequenceCommand(drive, park)
//                //add ur code here, look back at last year if u need help
//            )
//        );
//    }
//
//    private void tagToTelemetry(AprilTagDetection detection) {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("\nParking spot %d", detection.id + 1));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
//    }
//}
