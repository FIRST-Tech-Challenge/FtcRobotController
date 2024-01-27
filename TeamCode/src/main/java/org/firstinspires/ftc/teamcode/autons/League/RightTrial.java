package org.firstinspires.ftc.teamcode.autons.League;

import androidx.core.os.TraceKt;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.commands.ShooterCom;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subbys.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

import java.lang.reflect.WildcardType;

@Autonomous(name = "RIGHTTRIAL")
public class RightTrial extends CommandOpMode {
    private MecanumDriveSubsystem drive;
    private SimpleServo left_claw, right_claw;
    private Motor hang, arm;
//    private Motor frontLeft, frontRight, backLeft, backRight;

    private Pose2d startPose = new Pose2d(12, -61, Math.toRadians(90));


    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        drive.setPoseEstimate(startPose);

        left_claw = new SimpleServo(hardwareMap, "lc", -180, 180);
        right_claw = new SimpleServo(hardwareMap, "rc", -180, 180);

        hang = new Motor(hardwareMap, "hang");
//        sensor = new SensorRevTOFDistance(hardwareMap, "commonSense");
        arm = new Motor(hardwareMap, "arm");
//        frontLeft = new Motor(hardwareMap, "fL");
//        frontRight = new Motor(hardwareMap, "fR");
//        backLeft = new Motor(hardwareMap, "bL");
//        backRight = new Motor(hardwareMap, "bR");
//
//        frontLeft.motor.setDirection(DcMotor.Direction.FORWARD);
//        frontRight.motor.setDirection(DcMotor.Direction.FORWARD);
//        backLeft.motor.setDirection(DcMotor.Direction.FORWARD);
//        backRight.motor.setDirection(DcMotor.Direction.FORWARD);
//
//        frontLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        Trajectory traj = drive.trajectoryBuilder(startPose)
//                .splineTo(new Vector2d(42, -31), 0)
//                .build();
//
//        drive.followTrajectory(traj);
//
//        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
//                .forward(10)
//                .build();
//
//        drive.followTrajectory(traj2);


        Trajectory path1 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(28.5, -36))
                .build();
        Trajectory path2 = drive.trajectoryBuilder(path1.end())
                .back(17.5)
                .build();
        Trajectory path3 = drive.trajectoryBuilder(path2.end())
                .splineToLinearHeading(new Pose2d(46, -33.5, Math.toRadians(0)), Math.toRadians(0))
                .build();
        Trajectory path4 = drive.trajectoryBuilder(path3.end())
                .forward(14)
                .build();
        Trajectory path5 = drive.trajectoryBuilder(path4.end())
                .back(2.45)
                .build();
        Trajectory path6 = drive.trajectoryBuilder(path5.end())
                .back(8)
                .build();
        Trajectory path7 = drive.trajectoryBuilder(path6.end())
                .strafeRight(30)
                .build();
        Trajectory path8 = drive.trajectoryBuilder(path7.end())
                .forward(15)
                .build();


        TrajectoryFollowerCommand autonomous = new TrajectoryFollowerCommand(drive, path1);
        TrajectoryFollowerCommand autonomous2 = new TrajectoryFollowerCommand(drive, path2);
        TrajectoryFollowerCommand autonomous3 = new TrajectoryFollowerCommand(drive, path3);
        TrajectoryFollowerCommand autonomous4 = new TrajectoryFollowerCommand(drive, path4);
        TrajectoryFollowerCommand autonomous5 = new TrajectoryFollowerCommand(drive, path5);


        if(isStopRequested()){

        }

        schedule(new WaitUntilCommand(this::isStarted).andThen(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> hang.set(-1)),
                                new WaitCommand(700),
                                new InstantCommand(() -> hang.set(0)),
                                new WaitCommand(500),
                                new InstantCommand(() -> left_claw.turnToAngle(5)),
                                new InstantCommand(() -> right_claw.turnToAngle(0)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> hang.set(1)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> hang.set(0)),
                                new TrajectoryFollowerCommand(drive, path1),
                                new InstantCommand(() -> left_claw.turnToAngle(-85)),
                                new TrajectoryFollowerCommand(drive, path2),
                                new InstantCommand(() -> arm.set(-0.65)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> arm.set(-0.5)),
                                new WaitCommand(400),
                                new InstantCommand(() -> arm.set(0)),
                                new TrajectoryFollowerCommand(drive, path3),
                                new TrajectoryFollowerCommand(drive, path4),
                                new InstantCommand(() -> arm.set(0.5)),
                                new WaitCommand(1600),
                                new InstantCommand(() -> arm.set(0))),
                new TrajectoryFollowerCommand(drive, path5),
                        new WaitCommand(1000),
                        new InstantCommand(() -> right_claw.turnToAngle(90)),
                        new WaitCommand(500),
                        new TrajectoryFollowerCommand(drive, path6),
                        new TrajectoryFollowerCommand(drive, path7),
                        new TrajectoryFollowerCommand(drive, path8)
                )
        );

    }
}


//package org.firstinspires.ftc.teamcode.autons.League;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.drive.opmode.StrafeTest;
//
///*
// * This is an example of a more complex path to really test the tuning.
// */
//@Autonomous(name = "LEAGUE")
//public class CenterTrial extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(30, 30), 0)
//                .build();
//
//        drive.followTrajectory(traj);
//
//        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
//                .forward(10)
//                .build();
//
//        drive.followTrajectory(traj2);

//        sleep(2000);
//
//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
//        );
//    }
//}


