package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.yise.LiftArm;
import org.firstinspires.ftc.teamcode.yise.PoseStorage;
import org.firstinspires.ftc.teamcode.yise.TensorflowVision;

@Autonomous(name="Red Exterior Left", group="Linear Opmode")
public class AutoRedExteriorBackdropLeft extends LinearOpMode {
    //Initialize timer
    private ElapsedTime runtime = new ElapsedTime();
    int prop;

    @Override
    public void runOpMode() {
        //Initialize RR
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        LiftArm arm = new LiftArm(hardwareMap);

        TensorflowVision vision = new TensorflowVision(hardwareMap);

        //Sense cones
        while (!isStarted()) {
            prop = vision.getPropPosition();

            telemetry.addData("Prop: ", prop);
            telemetry.update();
        }

        if (isStopRequested()) return;


        //Bot starting position
        Pose2d startPose = new Pose2d(-41.5, -61, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        //Trajectory sequences contain driving instructions

        TrajectorySequence dropPixelLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-43, -46, Math.toRadians(-60)))
                .build();

        TrajectorySequence dropPixelCenter = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-41, -37, Math.toRadians(-90)))
                .build();

        TrajectorySequence dropPixelRight = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-33, -46, Math.toRadians(-130)))
                .build();

        TrajectorySequence driveToBoardLeft = drive.trajectorySequenceBuilder(dropPixelLeft.end())
                .lineToLinearHeading(new Pose2d(-32, -42, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-32, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(48, -30, Math.toRadians(180)))
                .addDisplacementMarker(60, () -> {
                    arm.extendAndDrop(LiftArm.Distance.AUTO);
                })
                .build();

        TrajectorySequence driveToBoardCenter = drive.trajectorySequenceBuilder(dropPixelCenter.end())
                .lineToLinearHeading(new Pose2d(-54, -35, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-54, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(180)))
                .addDisplacementMarker(60, () -> {
                    arm.extendAndDrop(LiftArm.Distance.AUTO);
                })
                .build();

        TrajectorySequence driveToBoardRight = drive.trajectorySequenceBuilder(dropPixelRight.end())
                .lineToLinearHeading(new Pose2d(-42, -43, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-42, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(48, -42, Math.toRadians(180)))
                .addDisplacementMarker(60, () -> {
                    arm.extendAndDrop(LiftArm.Distance.AUTO);
                })
                .build();

        TrajectorySequence driveForwardLeft = drive.trajectorySequenceBuilder(driveToBoardLeft.end())
                .back(-4)
                .strafeLeft(30)
                .back(6)
                .build();
        TrajectorySequence driveForwardCenter = drive.trajectorySequenceBuilder(driveToBoardCenter.end())
                .back(-4)
                .strafeLeft(25)
                .back(6)
                .build();
        TrajectorySequence driveForwardRight = drive.trajectorySequenceBuilder(driveToBoardRight.end())
                .back(-4)
                .strafeLeft(25)
                .back(6)
                .build();


        //Follow trajectories in order
        //switch between parking
        if(prop == 1) {
            drive.followTrajectorySequence(dropPixelLeft);
            sleep(500);
            drive.followTrajectorySequence(driveToBoardLeft);
            arm.openTrapdoor();
            sleep(500);
            arm.closeTrapdoor();
            arm.retract();
            sleep(2000);
            drive.followTrajectorySequence(driveForwardLeft);
            PoseStorage.currentPose = drive.getPoseEstimate();

        } else if (prop == 0) {
            drive.followTrajectorySequence(dropPixelCenter);
            sleep(500);
            drive.followTrajectorySequence(driveToBoardCenter);
            arm.openTrapdoor();
            sleep(500);
            arm.closeTrapdoor();
            arm.retract();
            sleep(2000);
            drive.followTrajectorySequence(driveForwardCenter);
            PoseStorage.currentPose = drive.getPoseEstimate();

        } else if (prop == 2) {
            drive.followTrajectorySequence(dropPixelRight);
            sleep(500);
            drive.followTrajectorySequence(driveToBoardRight);
            arm.openTrapdoor();
            sleep(500);
            arm.closeTrapdoor();
            arm.retract();
            sleep(2000);
            drive.followTrajectorySequence(driveForwardRight);
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
    }
}
