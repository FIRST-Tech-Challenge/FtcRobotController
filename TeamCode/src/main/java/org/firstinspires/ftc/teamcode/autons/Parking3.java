package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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

@Autonomous(name = "Parking3")
public class Parking3 extends CommandOpMode {
    private MecanumDriveSubsystem drive;

//    private SimpleServo left_claw;
//    private SimpleServo right_claw;
//    private Motor frontLeft, frontRight, backLeft, backRight;
//    private Motor arm;
//
//    private Pose2d startPose = new Pose2d(35.0, -62.0, Math.toRadians(0.0));

    @Override
    public void initialize() {
//        frontLeft = new Motor(hardwareMap, "fL");
//        frontRight = new Motor(hardwareMap, "fR");
//        backLeft = new Motor(hardwareMap, "bL");
//        backRight = new Motor(hardwareMap, "bR");

//        left_claw = new SimpleServo(hardwareMap, "lc", -180, 180);
//        right_claw = new SimpleServo(hardwareMap, "rc", -180, 180);
//
//        arm = new Motor(hardwareMap, "arm");
//
//        frontLeft.motor.setDirection(DcMotor.Direction.FORWARD);
//        frontRight.motor.setDirection(DcMotor.Direction.FORWARD);
//        backLeft.motor.setDirection(DcMotor.Direction.FORWARD);
//        backRight.motor.setDirection(DcMotor.Direction.FORWARD);

//        arm.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        arm.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);


//        Trajectory center = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
//                .forward(26)
//                .build();
        Trajectory center = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
                .forward(31)
                .build();
//        Trajectory strafe = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
//                .strafeRight(27)
//                .build();
//        Trajectory back = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
//                .forward(4)
//                .build();
//        Trajectory back2 = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
//                .back(4)
//                .build();
//        Trajectory strafeLeft = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
//                .strafeLeft(54)
//                .build();
//        Trajectory back4 = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
//                .back(20)
//                .build();
//
//        Trajectory lineTo = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90)))
//                .lineToSplineHeading(new Pose2d(4, -60.31, Math.toRadians(180)))
//                .build();
//
//        Trajectory forward3 = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90)))
//                .forward(12)
//                .build();
//
//        Trajectory back3 = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
//                .back(15)
//                .build();
//
//        Trajectory strafeLeft2 = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
//                .strafeLeft(40)
//                .build();
//
//        Trajectory forward2 = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
//                .forward(30)
//                .build();

        TrajectoryFollowerCommand autonomous = new TrajectoryFollowerCommand(drive, center);
//        TrajectoryFollowerCommand autonomous2 = new TrajectoryFollowerCommand(drive, strafe);
//        TrajectoryFollowerCommand autonomous3 = new TrajectoryFollowerCommand(drive, back);
//        TrajectoryFollowerCommand autonomous34 = new TrajectoryFollowerCommand(drive, back2);
//        TrajectoryFollowerCommand autonomous4 = new TrajectoryFollowerCommand(drive, strafeLeft);
//        TrajectoryFollowerCommand autonomous45 = new TrajectoryFollowerCommand(drive, back4);
//        TrajectoryFollowerCommand autonomous5 = new TrajectoryFollowerCommand(drive, lineTo);
//        TrajectoryFollowerCommand autonomous56 = new TrajectoryFollowerCommand(drive, forward3);
//        TrajectoryFollowerCommand autonomous6 = new TrajectoryFollowerCommand(drive, back3);
//        TrajectoryFollowerCommand autonomous7 = new TrajectoryFollowerCommand(drive, strafeLeft2);
//        TrajectoryFollowerCommand autonomous8 = new TrajectoryFollowerCommand(drive, forward2);

        if(isStopRequested()){
        }

//        schedule(new ParallelCommandGroup(
//                        new SequentialCommandGroup(
//                                new WaitCommand(250),
//                                autonomous,
//                                autonomous2,
//                                autonomous3,
//                                autonomous34,
//                                autonomous4,
//                                autonomous5,
//                                autonomous56,
//                                new WaitCommand(3000),
//                                autonomous6,
//                                autonomous7,
//                                autonomous8
//                        ),
//                        new SequentialCommandGroup(
//                                new InstantCommand(() -> left_claw.turnToAngle(-95)),
//                                new InstantCommand(() -> right_claw.setPosition(0.67)),
//                                new WaitCommand(750),
//                                new InstantCommand(() -> arm.set(-0.5)),
//                                new WaitCommand(2100),
//                                new InstantCommand(() -> arm.set(0)),
//                                new WaitCommand(9000),
//                                new InstantCommand(() -> left_claw.turnToAngle(-5)),
//                                new WaitCommand(1000),
//                                new InstantCommand(() -> arm.set(0.5)),
//                                new WaitCommand(1500),
//                                new InstantCommand(() -> arm.set(0))
//                        )
//                )
//        );
        schedule(autonomous);
    }
}