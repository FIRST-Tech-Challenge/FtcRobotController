package org.firstinspires.ftc.masters.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.firstinspires.ftc.masters.world.BackDropOpMode;
import org.firstinspires.ftc.masters.world.paths.BlueBackDropPath;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Config
@Autonomous(name = "Align on stack", group = "competition")
public class AlignStackTag extends BackDropOpMode {


    @Override
    protected void initializeProp(){
        drive.initializePropFindLeftProcessing();
    }
    @Override
    public void runOpMode() throws InterruptedException {

        initAuto();

        Pose2d startPose = new Pose2d(new Vector2d(16, 61.2), Math.toRadians(270)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);


        waitForStart();



        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            drive.update();

            toStackTag();


            }



        }

    protected void toStack(){

        if (dropTime!=null && dropTime.milliseconds()>300){
            outtakeTarget=0;
            drive.outtakeToTransfer();
        }

        if (!drive.isBusy()){
            List<AprilTagDetection> currentDetections = drive.getAprilTag().getDetections();
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
            telemetry.addData("April Tag Pos Es", drive.aprilTagCoarsePosEstimate(currentDetections));
            Pose2d robotPosition =drive.aprilTagCoarsePosEstimate(currentDetections);
            while (robotPosition.epsilonEquals(drive.getPoseEstimate())){
                currentDetections = drive.getAprilTag().getDetections();
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
                telemetry.addData("April Tag Pos Es", drive.aprilTagCoarsePosEstimate(currentDetections));
                robotPosition =drive.aprilTagCoarsePosEstimate(currentDetections);
            }
            currentState= State.TO_STACK_TAG;
            drive.setPoseEstimate(robotPosition);
            drive.followTrajectorySequenceAsync(BlueBackDropPath.toStackWing(drive, robotPosition));

        }
    }

    protected void toStackTag() {

        if (!drive.isBusy()) {
            if (pickupElapsedTime == null) {
                drive.intakeToTopStack();
                pickupElapsedTime = new ElapsedTime();
                drive.getMyVisionPortal().getActiveCamera().close();
            }
            //            if (has2Pixels() ){
//                pickupElapsedTime = new ElapsedTime();
//            }
//
//            if (pickupElapsedTime!=null && (pickupElapsedTime.milliseconds()>1000 || (has2Pixels() && pickupElapsedTime.milliseconds()>100))  ){
//                drive.stopIntake();
//                drive.raiseIntake();
//                drive.outtakeToPickup();
//                pickupElapsedTime = new ElapsedTime();
//                currentState = BackDropOpMode.State.BACKDROP_DEPOSIT_PATH;
//                drive.followTrajectorySequenceAsync(toBackBoard);
//
//            }
        }
    }


    }


