//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
//import com.acmerobotics.roadrunner.util.Angle;
//
//import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
//import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
//import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
//import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//
//@Autonomous(name="World Championship Auto", group="Autonomous")
//public class WorldChampionshipAuto extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        Pose2d startPose = new Pose2d(0, 0, 0);
//        drive.setPoseEstimate(startPose);
//
//
//        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
//                .splineTo(new Vector2d(10, 10), 0)
//                .turn(Math.toRadians(90))
//                .splineTo(new Vector2d(25, -15), 0)
//                .waitSeconds(3)
//                .turn(Math.toRadians(45))
//                .forward(10)
//                .strafeRight(5)
//                .turn(Math.toRadians(90))
//                .strafeLeft(5)
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(-10, -10, Math.toRadians(45)), 0)
//                .build();
//
//
//        waitForStart();
//
//        if (!isStopRequested()) {
//
//            drive.followTrajectorySequence(trajSeq);
//        }
//    }
//}
