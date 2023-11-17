import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Config
@Autonomous(name = "Blue Auto at Backstage")
public class AutoBlueBack extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence Middle1 = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(31, 0, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .back(26,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .strafeLeft(35,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .build();

        TrajectorySequence Left1 = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(25, 13, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .back(20,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .strafeLeft(20,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .build();

        TrajectorySequence Middle2 = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(31, 0, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .back(7,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .strafeRight(20,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .forward(23,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 3))
                .turn(Math.toRadians(90))
                .forward(109,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 3))
                .build();

        TrajectorySequence Right2 = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(25, -9, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .back(7,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .strafeLeft(10,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .forward(31,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 3))
                .turn(Math.toRadians(90))
                .forward(84,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 3))
                .build();

        TrajectorySequence Right1 = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .back(4,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .strafeLeft(14,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .turn(Math.toRadians(-90))
                .strafeLeft(6,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 3))
                .forward(18,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 3))
                .back(40,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .build();
        TrajectorySequence Left2 = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(22, 0, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .back(4,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .strafeRight(14,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .turn(Math.toRadians(90))
                .strafeRight(8,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 3))
                .forward(20,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 3))
                .back(6,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .strafeRight(24,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .forward(84,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 3))
                .build();

        waitForStart();

        if(gamepad1.a) {drive.followTrajectorySequence(Left1);}
        if(gamepad1.b) {drive.followTrajectorySequence(Middle1);}
        if(gamepad1.x) {drive.followTrajectorySequence(Middle2);}
        if(gamepad1.y) {drive.followTrajectorySequence(Right2);}

        if(gamepad1.left_bumper) {drive.followTrajectorySequence(Right1);}
        if(gamepad1.right_bumper) {drive.followTrajectorySequence(Left2);}

    }
}